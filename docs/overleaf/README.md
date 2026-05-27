# Lucrare de licență — `docs/overleaf/`

Schelet LaTeX pentru lucrarea de finalizare a studiilor pe proiectul
**Recon-Platform-R2**, redactat după șablonul oficial al Facultății de
Automatică și Calculatoare, UPT (`docs/Template AC ro v2.docx`).

## Fișiere

| Fișier            | Rol                                                                                                  |
| ----------------- | ---------------------------------------------------------------------------------------------------- |
| `main.tex`        | Documentul principal cu coperta, rezumat, abstract, cuprins, 12 capitole, declarația de autenticitate |
| `references.bib`  | Baza de date BibTeX cu ~35 de referințe IEEE (ROS2, SLAM, EKF, MPU-6050, Hough, morfologie, Flask, PostgreSQL, ESP32, etc.) |
| `README.md`       | Acest fișier — instrucțiuni de import în Overleaf și ghid de utilizare                                |

## Caracteristici care respectă șablonul UPT

- **Font**: Helvetica (`helvet`) ca aproximație Arial — fontul de bază.
- **Margini oglindite** (`twoside`): interior 2 cm, exterior 2 cm,
  sus 2.5 cm, jos 2 cm.
- **Spațiere între rânduri**: 1.15 (`\setstretch{1.15}` din `setspace`).
- **Aliniere**: justify pe tot corpul textului, primul rând indentat 1.5 cm.
- **Titluri capitole** (Heading 1): 14 pt, **bold**, MAJUSCULE, centrat.
- **Titluri secțiuni** (Heading 2): 12 pt, **bold**, MAJUSCULE, la stânga.
- **Antet**: începe cu *Universitatea Politehnica Timișoara* + program +
  candidat + titlu lucrare (apare pe paginile de conținut, suprimat pe
  copertă/rezumat/abstract via stilul `coperta`).
- **Numerotare pagini**: centrată în subsol, ascunsă pe paginile de
  copertă/rezumat/abstract.
- **Figuri**: numerotate, caption sub figură, centrate (`captionsetup[figure]`).
- **Tabele**: numerotate, caption deasupra tabelului, centrate
  (`captionsetup[table]`).
- **Formule**: numerotate automat în paranteze rotunde, aliniate la dreapta
  (comportament implicit `equation`).
- **Citări**: stil IEEE numeric (`\bibliographystyle{IEEEtran}`), numerele
  apar în ordinea apariției în text — exact cum cere UPT.
- **Diacritice românești**: caractere Unicode corecte
  (ă, â, î, ș `U+0219`, ț `U+021B`) prin `inputenc[utf8]` + `babel[romanian]`.

## Schema capitolelor (Cuprins)

```
Rezumat
Abstract
Cuprins

1. Introducere
   1.1 Motivația alegerii temei
   1.2 Obiectivele lucrării
   1.3 Structura lucrării

2. Stadiul actual al tehnologiei
   2.1 Sisteme SLAM 2D existente
   2.2 Senzori LIDAR low-cost pentru cartografierea indoor
   2.3 Fuziune senzorială inerțială pentru estimarea pozițiilor
   2.4 Soluții comerciale concurente

3. Fundamente teoretice
   3.1 Algoritmul Karto SLAM și optimizarea grafului de poze
   3.2 Filtre Kalman extinse pentru fuziune senzorială
   3.3 Integrarea giroscopică și conversia în cuaternioni
   3.4 Transformata Hough probabilistică
   3.5 Operațiuni morfologice pe imagini binare
   3.6 Etichetarea componentelor conectate
   3.7 Filtrul median 3×3
   3.8 Verificarea integrității datelor cu CRC-8 Dallas/Maxim

4. Arhitectura hardware a sistemului
   4.1 Privire de ansamblu (cu schemă TikZ)
   4.2 Raspberry Pi 5 — unitatea centrală de procesare
   4.3 ESP32 DevKit V1 — microcontroler coprocesor
   4.4 Senzorul LIDAR LD14P (LD-D200)
       4.4.1 Problema culorilor de fir non-standard
   4.5 Unitatea inerțială MPU-6050
   4.6 Circuitul de control al motorului LIDAR (S8050) (cu schemă TikZ)
   4.7 Butoanele și interfața utilizatorului
   4.8 Bilanțul energetic și alimentarea

5. Protocolul de comunicație ESP32 ↔ Raspberry Pi
   5.1 Stratul fizic — UART prin USB-CDC
   5.2 Formatul cadrului binar (cu schemă TikZ)
   5.3 Tipuri de cadre
   5.4 Calculul CRC-8
   5.5 Algoritmul de resincronizare a parserului
   5.6 Watchdog-ul motorului LIDAR

6. Firmware-ul ESP32
   6.1 Mediul de dezvoltare PlatformIO
   6.2 Planificatorul cooperativ (cu schemă TikZ — fluxul loop())
   6.3 Driverul MPU-6050
   6.4 Driverul butoanelor cu debounce
   6.5 Releul datelor LIDAR

7. Stiva software pe Raspberry Pi (ROS2)
   7.1 ROS2 Jazzy Jalisco — cadrul de lucru (cu schemă TikZ — arbore TF)
   7.2 Pachetul recon_hardware
   7.3 Pachetul recon_db
   7.4 slam_toolbox — cartografierea
   7.5 robot_localization — filtrul Kalman extins
   7.6 Driverul vendorat ldlidar_stl_ros2

8. Interfața web (recon_webui)
   8.1 Arhitectura Flask + SocketIO + eventlet
   8.2 Modelul de threading și sincronizare (cu schemă TikZ)
   8.3 Pattern-ul DataChannel cu fallback automat
   8.4 Paginile interfeței
   8.5 Punctul de acces WiFi

9. Stratul de persistență și post-procesarea hărților
   9.1 Containerul Docker PostgreSQL
   9.2 Schema bazei de date
   9.3 Pipeline-ul Tier-2 (cu schemă TikZ)

10. Integrarea sistemului și desfășurarea
    10.1 Scriptul setup.sh
    10.2 Scriptul environment.sh
    10.3 Autostart prin systemd

11. Testarea și validarea
    11.1 Strategia de testare
    11.2 Teste unitare Python (tabel)
    11.3 Teste unitare C++
    11.4 Validarea pe hardware real

12. Concluzii
    12.1 Sumarul realizărilor
    12.2 Contribuții originale
    12.3 Limitări cunoscute
    12.4 Direcții viitoare

Bibliografie

Declarație de autenticitate
```

Fiecare secțiune marcată `[De dezvoltat: ...]` este o ancoră peste care
puteți adăuga conținutul propriu — comentariile din sursă listează
ideile principale și referințele relevante.

## Cum se importă în Overleaf direct din GitHub

### Opțiunea A — Overleaf premium / instituțional (cont UPT)

Overleaf oferă integrare nativă cu GitHub pe planurile premium /
instituționale.

1. Deschideți [https://www.overleaf.com/](https://www.overleaf.com/).
2. Logați-vă cu contul instituțional UPT (dacă există) sau cu un cont
   personal abonat.
3. Apăsați **New Project → Import from GitHub**.
4. Autorizați Overleaf să acceseze contul vostru GitHub la prima
   utilizare.
5. Alegeți repository-ul **`Recon-Platform-R2`**.
6. Overleaf va clona întregul repo. Deschideți proiectul, apăsați
   meniul stânga, și setați:
   - **Compiler**: `pdfLaTeX` (default)
   - **Main document**: `docs/overleaf/main.tex`
   - **TeX Live version**: `2024` sau ultima disponibilă

### Opțiunea B — orice cont Overleaf gratuit (recomandat dacă nu aveți premium)

Importul direct GitHub e disabled pe planul gratuit, dar funcționează
metoda ZIP:

1. Pe GitHub, navigați la repo-ul `Recon-Platform-R2`.
2. Apăsați **Code → Download ZIP**.
3. Extrageți arhiva local.
4. Mergeți pe [https://www.overleaf.com/](https://www.overleaf.com/),
   **New Project → Upload Project**.
5. Selectați **doar conținutul folderului `docs/overleaf/`** (nu tot
   repo-ul), comprimat ca ZIP. Adică:
   ```
   docs/overleaf/main.tex
   docs/overleaf/references.bib
   docs/overleaf/README.md
   ```
6. Overleaf va deschide proiectul cu `main.tex` ca fișier principal.
7. Compilatorul `pdfLaTeX` este default — apăsați **Recompile**.

### Opțiunea C — sincronizare bidirectională cu Git (avansat)

Dacă vreți să editați și pe Overleaf, și local, și să sincronizați prin
GitHub:

1. Creați proiectul prin Opțiunea A sau B.
2. Pe Overleaf, deschideți **Menu (stânga sus) → Sync → GitHub**.
3. Conectați-l la fork-ul vostru personal al `Recon-Platform-R2`.
4. Folosiți butoanele **Push** și **Pull** din Overleaf pentru a
   sincroniza.

> ⚠️ Pe Overleaf gratuit, sincronizarea bidirecțională Git **nu este
> disponibilă** — aveți nevoie de abonament Standard sau mai mare.

## Compilare locală (opțional)

Dacă vreți să compilați și local cu TeX Live (Ubuntu/Debian):

```bash
sudo apt install texlive-latex-extra texlive-fonts-recommended \
                 texlive-bibtex-extra texlive-lang-other biber

cd docs/overleaf
pdflatex main
bibtex main
pdflatex main
pdflatex main
```

Sau cu `latexmk` (mai simplu):

```bash
sudo apt install latexmk
cd docs/overleaf
latexmk -pdf main.tex
```

## Personalizare rapidă

Înainte să trimiteți lucrarea, editați următoarele macro-uri la începutul
fișierului `main.tex` (linia ~115):

```latex
\newcommand{\candidatname}{Prenume NUME}        % <<< COMPLETAȚI
\newcommand{\coordinatorname}{Asist. dr. ing. Prenume NUME}
\newcommand{\sessionname}{Iunie 2026}
\newcommand{\specializare}{Calculatoare și Tehnologii Informaționale}
\newcommand{\workTitle}{Sistem portabil de cartografiere indoor cu LIDAR 2D și fuziune senzorială inerțială}
```

Aceste valori apar automat pe coperta, în antet, în declarația de
autenticitate, și în meta-datele PDF-ului final.

## Bibliografia — verificare rapidă

Fișierul `references.bib` conține referințe pentru fiecare tehnologie /
algoritm / formulă matematică folosită în proiect:

| Categorie                     | Referințe principale                                                                                              |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| **ROS2**                      | `ros2jazzy`, `ros2macenski`, `ros2why`, `maruyama2016ros2`                                                         |
| **SLAM / Karto / Ceres**      | `macenski2021slamtoolbox`, `konolige2010karto`, `hess2016cartographer`, `kohlbrecher2011hector`, `ceres_solver`     |
| **EKF / filtre orientare**    | `moore2014robotlocalization`, `welchbishop2006kalman`, `kalman1960`, `madgwick2014`, `mahony2008`                  |
| **Senzori**                   | `mpu6050`, `mpu6050regmap`, `ld14p`, `rpi5brief`                                                                   |
| **ESP32 / firmware**          | `esp32idf`, `platformio`, `arduinoesp32`                                                                            |
| **Procesare imagini**         | `hough1962`, `dudahart1972`, `matas2000probhough`, `serra1982morpho`, `soille2003morpho`, `rosenfeldpfaltz1966`, `tukey1977eda`, `harris2020numpy` |
| **CRC-8 Dallas/Maxim**        | `maxim1wirecrc`, `linuxcrc8`, `koopman2002crc`                                                                     |
| **Stack web**                 | `flask`, `flasksocketio`, `eventlet`, `socketio`                                                                    |
| **Baze de date**              | `postgresql`, `sqlalchemy`, `alembic`, `docker`                                                                     |
| **System tooling**            | `systemd`, `hostapd`, `dnsmasq`, `linuxpty`, `usbcdc`                                                               |
| **Cărți de referință**        | `thrun2005probrobotics`, `quigley2015programmingros`, `li2003realtime`, `barr1999embedded`, `stevens2013unp`        |
| **Cerințe UPT**               | `ieeecitation`, `uptregulament`                                                                                     |

Citarea se face cu `\cite{cheia}`. Exemple deja prezente în
`main.tex`:

```latex
... folosește algoritmul \textbf{Karto SLAM}~\cite{konolige2010karto},
un sistem 2D bazat pe optimizarea unui graf de poze.

... pachetul \texttt{slam\_toolbox}~\cite{macenski2021slamtoolbox} din
\textit{ROS2 Jazzy Jalisco}~\cite{ros2jazzy} ...

... biasului ZA\_OFFSET al cipului MPU-6050 folosit, document
în~\cite{mpu6050regmap}.
```

## Diagrame TikZ derivate din cod

Documentul include 7 diagrame TikZ care reflectă arhitectura proiectului:

1. **Figura 4.1**: Arhitectura hardware de nivel înalt
   (LIDAR + IMU + buttons → ESP32 → Pi → DB + Browser)
2. **Tabelul 4.1**: Cablajul LD14P → ESP32 (firele non-standard)
3. **Figura 4.2**: Circuitul S8050 de control al motorului LIDAR
4. **Figura 5.1**: Formatul cadrului binar UART
5. **Tabelul 5.1**: Tipurile de cadre (cu LEN-uri)
6. **Figura 6.1**: Fluxul `loop()` al firmware-ului
7. **Figura 7.1**: Arborele de transformări TF2
8. **Figura 8.1**: Modelul de threading (eventlet vs rclpy)
9. **Figura 9.1**: Pipeline-ul Tier-2

Diagramele sunt redactabile direct în sursa LaTeX (mutați noduri,
schimbați culori, adăugați elemente fără editor extern).

## Note finale

- **Limita de pagini**: șablonul UPT recomandă ≤ 100 pagini. Scheletul
  curent ocupă ~12 pagini compilat; conținutul propriu adăugat va
  ajunge la ~60–80 pagini estimat.
- **Fonturi**: dacă vreți să folosiți Arial real în loc de Helvetica,
  treceți la XeLaTeX și înlocuiți preambulul cu `\usepackage{fontspec}`
  + `\setmainfont{Arimo}` (Arimo este font Arial-metric-compatibil,
  disponibil în TeXLive 2024).
- **Cuvinte despărțite**: `babel[romanian]` activează automat regulile
  de despărțire în silabe pentru limba română.
- **Cod sursă în text**: dacă includeți blocuri de cod (Python, C++),
  pachetul `listings` este deja preîncărcat — vedeți comentariile din
  preambulul `main.tex`.

Pentru întrebări legate de șablon vs.\ implementare, consultați
[`docs/Template AC ro v2.docx`](../Template%20AC%20ro%20v2.docx).
