# Recon-Platform-R2 — Prompturi pentru generarea diagramelor

> Document destinat să fie predat unui alt LLM (generator de imagini / diagrame).
> Fiecare secțiune este un prompt **autonom**: copiezi blocul respectiv și îl dai
> modelului. Datele tehnice sunt extrase 1:1 din cod și din documentația canonică
> (`firmware/esp32/src/*`, `docs/SPEC.md`, `docs/ARCHITECTURE.md`,
> `docs/UART_PROTOCOL.md`, `recon_db/postprocess.py`).

---

## 0. Stil global (prefixează-l la fiecare prompt)

> **Stil:** diagramă tehnică curată, vectorială, fundal alb, aspect de manual de
> inginerie / lucrare de licență. Font sans-serif lizibil. Etichetele tehnice
> (denumiri de pini, semnale, topic-uri ROS) rămân **în engleză exact ca în text**;
> textul explicativ poate fi în română. Folosește cutii dreptunghiulare cu colțuri
> rotunjite pentru module, săgeți cu vârf clar pentru fluxul de date (etichetate cu
> semnalul/baud-ul), linii punctate pentru legături opționale/viitoare. Paletă
> sobră (gri/albastru/verde), fără gradient agresiv, fără 3D, fără skeuomorfism.
> Nu inventa componente care nu sunt listate. Raport de aspect 16:9 dacă nu se
> specifică altfel.

---

## 1. Schemă bloc hardware (system block diagram)

**Scop:** vederea de ansamblu a întregului dispozitiv handheld de cartografiere LIDAR.

**Trei blocuri mari, conectate în lanț stânga → dreapta:**

1. **LD14P LIDAR** (senzor, 4 fire), 230 400 baud, ~10 Hz rotație.
2. **ESP32-D DevKit V1** (coprocesor I/O hub) — în mijloc, blocul central.
3. **Raspberry Pi 5** (Ubuntu Server 24.04 · ROS 2 Jazzy) — calculatorul principal.
4. **Browser** (telefon/laptop al operatorului) — în dreapta, conectat prin WiFi.

**Conexiuni de desenat (etichetate exact așa):**

- LD14P **Black = VCC 5 V** → ESP32 **5V (VIN)**.
- LD14P **White = TX (date)** → ESP32 **GPIO16 (Serial2 RX) @ 230400 8N1**.
- LD14P **Red = GND** și **Green = RX** → ambele la **colectorul tranzistorului S8050**
  (comutator low-side comandat de **ESP32 GPIO4**). Etichetă: „LIDAR power gating”.
- ESP32 ⇄ Pi: un singur link **USB-Serial (CP2102/CH340) @ 460800 8N1**, **bidirecțional**
  (săgeată dublă), apare la Pi ca **/dev/ttyUSB0** sau **/dev/ttyACM0**.
- În interiorul blocului ESP32, listează ca sub-elemente (bullet-uri mici):
  - „MPU-6050 IMU @ 100 Hz pe I²C, adresă 0x68”
  - „3 butoane: SHUTDOWN / START-STOP / SAVE pe GPIO 25 / 26 / 27”
  - „Status LED (GPIO2)”
  - „Releu UART LD14P → USB-CDC (chunk-uri LIDAR_FRAME)”
  - „LIDAR_EN (GPIO4) cu watchdog 3 s pe refresh-ul de la Pi”
- În interiorul blocului Pi, listează: „esp32_uart_bridge”, „ldlidar_stl_ros2”,
  „slam_toolbox”, „recon_webui (Flask+SocketIO :80)”, „PostgreSQL (container Docker)”.
- Pi → Browser: WiFi, **AP „Recon” pe ap0 = 10.0.0.1** + **wlan0 LAN**,
  acces la `http://recon.local`.

**Bloc separat de alimentare (jos):** `Power bank USB-C 22,5 W → Raspberry Pi 5 → (prin USB) ESP32 → (rail 5 V) LD14P`.
Notă: o singură sursă alimentează tot (fără buck, fără comutator SPST); power bank-ul are
propriul buton de pornit/oprit. Butonul SHUTDOWN este doar semnal (declanșează `shutdown -h now`),
nu taie curentul.

---

## 2. Conexiune MPU-6050 ↔ ESP32

**Scop:** schemă de cablare punct-la-punct între senzorul IMU și ESP32 (4 fire I²C + adresă).

**Două cutii: „MPU-6050 (GY-521)” în stânga, „ESP32-D DevKit V1” în dreapta.**

**Fire de desenat (4 conexiuni + nota de adresă):**

| Pin MPU-6050 | → | Pin ESP32 | Etichetă |
| --- | --- | --- | --- |
| **VCC** | → | **3V3** | „3.3 V — NU 5 V” (avertisment, evidențiat) |
| **GND** | → | **GND** | masă comună |
| **SDA** | → | **GPIO21** | I²C date |
| **SCL** | → | **GPIO22** | I²C ceas — „400 kHz (Fast-mode)” |
| **AD0** | → | **GND** | „selectează adresa I²C = 0x68” |

**Casetă de note tehnice (lateral, ca legendă):**
- Magistrală **I²C @ 400 kHz**, adresă **0x68** (AD0 la masă).
- Configurație senzor (scrisă de firmware la `imu::begin()`):
  - Accelerometru **±4 g** (AFS_SEL=1 → 8192 LSB/g)
  - Giroscop **±500 °/s** (FS_SEL=1 → 65.5 LSB/(°/s))
  - **DLPF ~44 Hz** (CONFIG=0x03), **SMPLRT_DIV=9 → 100 Hz** ieșire
  - Ceas PLL cu referință giro X (CLKSEL=1)
- Citire în rafală de 14 octeți de la registrul 0x3B (ACCEL_XOUT_H), conversie în
  unități SI (m/s², rad/s) **pe ESP32**.

---

## 3. Circuit S8050 pentru motorul LIDAR (comutator low-side)

**Scop:** schemă electronică (schematic, nu bloc) a comutatorului cu tranzistor care
pornește/oprește alimentarea LIDAR-ului.

**Componente și conexiuni (desenează-le ca schematic clasic, cu simbol de tranzistor NPN):**

- Tranzistor **S8050 (NPN)** în centru, cu cele trei terminale etichetate **B / C / E**.
- **ESP32 GPIO4** ──[ rezistor **1 kΩ** ]── **bază (B)**.
- **ESP32 GND** ───────────────────── **emitor (E)**.
- **colector (C)** ── la nodul comun unde se leagă **LD14P Red (GND)** ȘI **LD14P Green (RX)**
  (ambele fire împreună). Etichetă: „comutare low-side a masei LIDAR-ului”.
- Separat (nu prin tranzistor): **LD14P Black (VCC 5 V)** ── direct la **ESP32 5V (VIN)**;
  **LD14P White (TX)** ── la **ESP32 GPIO16 (Serial2 RX @ 230400)**.

**Adnotări de funcționare (text lângă schemă):**
- **GPIO4 = HIGH (3.3 V)** ⇒ S8050 saturat ⇒ masa LIDAR se închide ⇒ **motorul se rotește**.
- **GPIO4 = LOW** ⇒ tranzistor blocat ⇒ LIDAR nealimentat ⇒ **motorul oprit**.
- Motiv pentru comutarea **simultană** a Red (GND) și Green (RX): previne curentul
  parazit („sneak current”) prin pull-up-ul de pe pinul RX al LIDAR-ului când motorul e oprit.
- GPIO4 este setat **LOW ca prima instrucțiune din `setup()`** → motorul rămâne oprit
  pe toată fereastra de boot.
- Comandat de la Pi prin opcode-ul **LIDAR_EN**; firmware-ul are watchdog 3 s.

---

## 4. Format cadru UART (binary frame layout)

**Scop:** diagramă de tip „byte-field / packet layout” pentru formatul binar ESP32 ⇄ Pi.

**Rândul principal — cutii consecutive orizontale, fiecare câmp etichetat cu offset și dimensiune:**

```
 byte 0   byte 1   byte 2   byte 3   bytes 4 .. 4+LEN-1     byte 4+LEN
[ 0xA5 ] [ 0x5A ] [ TYPE ] [ LEN ] [ PAYLOAD (LEN octeți) ] [ CRC8 ]
  SYNC0    SYNC1    1 B      1 B        0..64 B               1 B
```

- Sub câmpurile TYPE, LEN, PAYLOAD desenează o acoladă cu textul:
  **„CRC8 acoperă TYPE + LEN + PAYLOAD (octeții SYNC NU sunt incluși)”**.
- Notă: dimensiune totală cadru = **5 + LEN** octeți; LEN ∈ [0, 64]; **little-endian**;
  **460800 baud, 8N1**, fără control de flux.
- Casetă CRC: „Dallas/Maxim CRC-8, polinom 0x07, init 0x00, fără reflectare, XOR-out 0x00”.

**Tabel secundar cu tipurile de cadru (desenează-l ca legendă sub schemă):**

| TYPE | Nume | Direcție | LEN | Rată | Payload |
| --- | --- | --- | --- | --- | --- |
| 0x01 | IMU | ESP→Pi | 24 | 100 Hz | 6×float32: ax,ay,az,gx,gy,gz |
| 0x02 | BUTTON | ESP→Pi | 2 | la eveniment | uint8 id, uint8 state |
| 0x03 | HEARTBEAT | ESP→Pi | 4 | 1 Hz | uint32 uptime_ms |
| 0x04 | STATUS | ESP→Pi | 2 sau 8 | boot + eroare IMU | flags + diagnostic |
| 0x05 | LIDAR_FRAME | ESP→Pi | 1..64 | pe măsură ce sosesc | octeți LD14P brut |
| 0x06 | LIDAR_EN | **Pi→ESP** | 1 | eveniment + refresh 1 Hz | uint8 enable (0/1) |
| 0x07 | LIDAR_ACK | ESP→Pi | 1 | la schimbare de stare | uint8 enabled |

**Exemplu concret de evidențiat (cadru IMU, dispozitiv plat, staționar):**
`A5 5A 01 18 | 00000000 00000000 DB0F1D41 (az≈9.80665) 00000000 00000000 00000000 | CRC` — 30 octeți.

---

## 5. Mașina de stări a parserului (frame parser FSM)

**Scop:** diagramă de automat finit (state machine) pentru parserul de cadre care
re-sincronizează după pierderi de octeți. Cercuri/dreptunghiuri rotunjite = stări;
săgeți etichetate cu condiția pe octetul curent.

**Stări (6):** `HUNT0` (start) → `HUNT1` → `TYPE` → `LEN` → `PAY` → `CRC`.

**Tranziții de desenat (etichetează fiecare săgeată exact așa):**

- **HUNT0**: dacă `byte == 0xA5` → **HUNT1**. Altfel rămâne în HUNT0 (self-loop).
- **HUNT1**: dacă `byte == 0x5A` → **TYPE**; dacă `byte == 0xA5` → rămâne în HUNT1
  (self-loop, tratează `A5 A5 5A`); altfel → **HUNT0**.
- **TYPE**: salvează `type` → **LEN** (necondiționat).
- **LEN**: salvează `len`; dacă `len > 64 (MAX_PAYLOAD)` → `bad_len_count++` și → **HUNT0**;
  altfel: dacă `len > 0` → **PAY**, dacă `len == 0` → **CRC**.
- **PAY**: `buf[idx++] = byte`; rămâne în PAY (self-loop) până când `idx == len`, apoi → **CRC**.
- **CRC**: calculează `crc8(type,len,payload)`; dacă **se potrivește** → emite cadrul
  (callback `handle_frame`) și → **HUNT0**; dacă **NU** → `crc_fail_count++` și → **HUNT0**.

**Note pe margine:** „re-sincronizare fără intervenție externă: octet de sync pierdut ⇒
caută din nou 0xA5 0x5A; CRC greșit ⇒ aruncă cadrul, log la nivel debug”. Cele două
contoare `crc_fail_count` și `bad_len_count` sunt expuse pentru diagnostic.
Aceeași mașină de stări există identic în C++ (ESP32) și în Python (bridge-ul Pi).

---

## 6. Bucla cooperativă `loop()` ESP32

**Scop:** flowchart vertical al funcției `loop()` din firmware — planificare cooperativă
pe deadline-uri `millis()`, **fără FreeRTOS**, rulează la >1 kHz.

**Casetă `setup()` în capul diagramei (înainte de buclă), cu pașii în ordine:**
1. `pinMode(GPIO4, OUTPUT); digitalWrite(LOW)` — **PRIMA linie**, motor oprit la boot.
2. Status LED LOW.
3. `Serial.begin(460800)` (link Pi).
4. `Serial2.setRxBufferSize(1024); Serial2.begin(230400, …, RX=GPIO16)` (intrare LD14P).
5. `Button.begin × 3`.
6. `imu::begin()`.
7. `framing::send_status_diag(...)` (flags boot + identitate IMU).
8. programează deadline-urile IMU / heartbeat / LED.

**Bucla `loop()` — pași secvențiali (desenează-i ca lanț vertical de cutii cu romburi de decizie):**

1. **Drain Serial (Pi → ESP):** `while (Serial.available())` → parser de cadre; la
   `LIDAR_EN` → setează GPIO4 + actualizează deadline-ul de refresh al watchdog-ului.
2. **Drain Serial2 (LD14P → Pi), DOAR dacă motor pornit:** citește până la 64 B și le
   trimite ca un cadru **LIDAR_FRAME**.
3. **Watchdog LIDAR:** romb de decizie „motor pornit ȘI niciun refresh în 3 s?”
   (cu aritmetică **`(int32_t)` signed** pe `millis()`) → DA: forțează motor OFF.
4. **Butoane ×3:** `Button.update()` (debounce 25 ms + detecție de front → `send_button`).
5. **IMU @ 100 Hz:** romb „`now ≥ next_imu_ms`?” → DA: `imu::read()` → `send_imu()`;
   la eșec → `imu::begin()` retry + `send_status()`.
6. **Heartbeat @ 1 Hz:** romb „`now ≥ next_heartbeat_ms`?” → DA: `send_heartbeat(uptime)`.
7. **`update_status_led()`** (LED fix = IMU OK; clipire lentă ~1 Hz = IMU absent).

Bucla se închide înapoi la pasul 1 (săgeată de retur „rulează cât de des posibil, ~kHz”).
Casetă de buget de trafic: „IMU ~3 KB/s + LIDAR_FRAME ~23 KB/s ≈ 26 KB/s total, sub
bugetul de ~46 KB/s al link-ului @ 460800 baud”.

---

## 7. Graful ROS 2 + arborele TF2

**Scop:** graf de noduri ROS 2 (elipse = noduri, dreptunghiuri = topic-uri/servicii,
săgeți = publish/subscribe) plus, separat, arborele de transformări TF2.

**Partea A — graful de noduri (flux de sus în jos):**

- **esp32_uart_bridge** (nod rclpy Python) publică:
  - `/imu/data_raw` (sensor_msgs/Imu, 100 Hz, BEST_EFFORT)
  - `/buttons/save`, `/buttons/startstop`, `/buttons/shutdown_*`
  - `/esp32/diagnostics` (1 Hz, JSON)
  - expune serviciul `/lidar_enable` (std_srvs/SetBool)
  - scrie octeții **LIDAR_FRAME → pty master → `/tmp/lidar_pty`** (săgeată specială, nu topic ROS).
- **ldlidar_stl_ros2** citește `/tmp/lidar_pty` → publică **`/scan`** (sensor_msgs/LaserScan,
  geometrie fixă 720 raze).
- `/imu/data_raw` → **imu_filter_madgwick** (sau imu_yaw_integrator) → **`/imu/data`**.
- `/imu/data` → **robot_localization ekf_node** → **`/odom`** + TF `odom→base_link`
  (deocamdată un `static_transform_publisher` cu **identitate**; linie punctată = viitor H3).
- **slam_toolbox** (online_async, solver Ceres) consumă `/scan` → publică **`/map`**
  (nav_msgs/OccupancyGrid, la ~1 s) + TF **`map→odom`** (~50 Hz).
- **recon_webui_bridge** (rclpy pe thread OS real) abonat la `/map`, `/tf`, `/robot/events`
  → compune poza scannerului (`map→odom ∘ odom→base_link`).
- **recon_webui** (Flask + SocketIO pe :80) → **Browser** (WebSocket).
- **db_node** abonat la `/robot/events` → scrie în **PostgreSQL**.

Marchează cu linie punctată / etichetă „H2.1 / H3 (planificat)” legăturile încă neactive.

**Partea B — arborele TF2 (lanț simplu de frame-uri, desenat separat în dreapta/jos):**

```
map ──►(slam_toolbox)──► odom ──►(static identitate / EKF H3)──► base_link ──►(static)──► laser_frame
```

Note: azi `odom→base_link` este **identitate** (nu există estimare de mișcare), deci poza
scannerului în frame-ul `map` este chiar `map→odom`, citită direct din `/tf`.

---

## 8. Model de threading `recon_webui`

**Scop:** diagramă a celor două contexte de execuție concurente din procesul Python
`recon_webui` și a cozii care le leagă (subliniază regula „o singură direcție de emit”).

**Două cutii mari verticale + o coadă între ele:**

1. **Sus: bucla de green-thread eventlet** (cutie mare). Conține:
   - Flask + SocketIO „trăiesc aici”
   - handlere de request HTTP
   - bucla de emit WebSocket (`emit_loop`)
   - interogări DB prin **`eventlet.tpool.execute()`** (apeluri blocante)
   - helperele sincrone de serviciu ROS (`_call_slam_pause`, `_call_lidar_enable`,
     `clear_map`) — **rulează DOAR pe greenlet**.

2. **Jos: thread OS real `ros2_bridge_spin`** (cutie mare). Conține:
   - `rclpy.spin(node)` (obținut via `eventlet.patcher.original("threading")`)
   - callback-urile de subscriber
   - împinge evenimente în coadă.

3. **Între ele: `queue.Queue(maxsize=64)`** — desenat ca un canal cu o singură săgeată
   **de jos în sus** (rclpy → eventlet).

**Adnotări critice (casete de avertizare):**
- Săgeată **interzisă** (X roșu) de jos în sus etichetată: **„`socketio.emit()` NU se apelează
  niciodată din thread-ul rclpy — folosește coada”**.
- „`eventlet.monkey_patch()` este **prima linie executabilă** din `app.py`; importurile vin după.”
- „Auto-pauza e programată cu `eventlet.spawn_after(...)` ca să moștenească contextul de greenlet.”
- Motiv (notă mică): apelarea semaforului de greenlet dintr-un thread OS real produce
  `greenlet.error: Cannot switch to a different thread`.

---

## 9. Hartă brută vs. Tier-2 (doar ilustrativ)

**Scop:** comparație **side-by-side, pur ilustrativă** (NU date reale) între o hartă de
ocupare brută de la SLAM și aceeași hartă după pipeline-ul de post-procesare Tier-2.
Două panouri etichetate „Hartă brută (/map de la slam_toolbox)” și „Hartă procesată Tier-2”.

**Panoul STÂNGA — „Hartă brută”:**
- Grid de ocupare 2D al unei camere/încăperi, văzut de sus (top-down).
- Pereți reprezentați ca celule negre/închise, **zgomotoși**: subțiri (1–2 celule),
  cu margini zimțate, întreruperi/goluri, **celule izolate de zgomot „salt-and-pepper”**
  împrăștiate prin spațiul liber.
- Spațiu liber = gri deschis; necunoscut = gri mediu.
- Întreaga hartă **ușor înclinată** (rotită cu câteva grade față de orizontală).
- Aspect „brut, nefinisat”.

**Panoul DREAPTA — „Tier-2 procesat”:**
- Aceeași încăpere, dar **curată**: zgomotul salt-and-pepper eliminat, golurile mici din
  pereți închise (morfologic), pereți continui.
- Pereții grupați în **componente conexe colorate distinct** (fiecare „blob”/cluster o
  culoare diferită — albastru, verde, portocaliu, mov etc.).
- **Segmente de perete suprapuse ca linii cyan crispe, drepte** (rezultatul Hough Line Transform).
- Harta **îndreptată (deskew Manhattan)** — pereții dominanti aliniați la axele orizontală/verticală;
  etichetă mică „deskewed N°”.
- Aspect „finisat, vectorizat”.

**Bandă centrală cu etapele pipeline-ului (săgeată stânga→dreapta, NumPy pur):**
`filtru median 3×3 (opțional)` → `deschidere morfologică (opțional)` →
`închidere morfologică (1 iter)` → `etichetare componente conexe (8-conex, BFS)` →
`Hough probabilistic → segmente de perete` → `regularizare Manhattan (deskew + snap la H/V,
doar dacă concentrația unghiulară R ≥ 0.2)`.

**Disclaimer obligatoriu pe imagine:** text mic „Figură ilustrativă — nu reprezintă date
de scanare reale”.

---

### Note de utilizare

- Dacă generatorul de imagini acceptă un singur prompt, prefixează **Secțiunea 0 (stil global)**
  și apoi lipește una dintre secțiunile 1–9.
- Pentru consistență vizuală între figuri, cere explicit aceeași paletă și același font la fiecare.
- Tabelele (secțiunile 2, 4, 7) pot fi redate fie ca tabele desenate, fie ca legende lângă schemă.
