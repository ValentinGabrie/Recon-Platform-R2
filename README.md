# Recon-Platform-R2

A handheld LIDAR mapping device built on a Raspberry Pi 5. Walk through a
space holding the device; it produces a saved 2-D occupancy grid map of
where the walls are.

```
        ┌─────────┐    USB-Serial         ┌──────────────────────┐
        │  ESP32  │  ─────────────────►   │   Raspberry Pi 5     │
        │  + IMU  │   (binary framing)    │   ROS2 Jazzy         │
        │  + 3 ×  │                       │   slam_toolbox       │
        │ buttons │                       │   Flask + SocketIO   │
        └─────────┘                       │   PostgreSQL (Docker)│
                                          └─────┬────────────────┘
        ┌─────────┐    UART (PL011)             │
        │ LD14P   │  ─────────────────►         │   WiFi AP "Recon"
        │ LIDAR   │   /scan @ 10 Hz             ▼
        └─────────┘                          Phone / laptop
                                              http://recon.local/map
```

The Pi runs SLAM on the LIDAR feed, fuses IMU data from the ESP32 for
robust scan-matching, and serves a web UI that lets you watch the map
build in real time and save scans to PostgreSQL.

---

## Repository layout

```
Recon-Platform-R2/
├── README.md                  ← you are here
├── docs/                      ← canonical project documentation
│   ├── SPEC.md                ← current technical spec
│   ├── ARCHITECTURE.md        ← system architecture, data flow, TF tree
│   ├── STATUS.md              ← what's done, what's pending
│   ├── ROADMAP.md             ← H1–H6 staged plan
│   ├── AGENT_RULES.md         ← binding rules for LLM/agent contributors
│   ├── UART_PROTOCOL.md       ← ESP32 ↔ Pi binary framing reference
│   └── archive/               ← snapshot of pre-pivot docs (autonomous-robot era)
├── firmware/
│   └── esp32/                 ← PlatformIO project for the ESP32 I/O hub
│       └── README.md          ← wiring + flash workflow
└── roomba_ws/                 ← ROS2 colcon workspace (folder name kept for git history)
    ├── setup.sh               ← canonical entry point — kill / demo / web / sensor-test
    ├── environment.sh         ← idempotent provisioning (apt, ROS2, venv, Docker, …)
    ├── config/                ← all runtime YAML
    ├── src/recon_*            ← 5 ROS2 packages: hardware, control, db, webui, bringup
    └── tests/                 ← C++ gtest + Python pytest suites
```

The colcon workspace folder name is **`roomba_ws/`** for git-history
continuity. The packages inside it were renamed `roomba_*` → `recon_*` in
Stage H1 (2026-05-10).

---

## Quick start (already-provisioned Pi)

```bash
cd ~/Recon-Platform-R2/roomba_ws
bash setup.sh kill              # tear down anything stale
bash setup.sh demo              # web UI only, mock data — http://<pi>/map
bash setup.sh web               # web UI + DB node, ROS2 running, no hardware
bash setup.sh sensor-test       # full LIDAR → SLAM → web UI pipeline
```

`setup.sh` is the only blessed way to start the stack. See
[`docs/AGENT_RULES.md`](docs/AGENT_RULES.md) section "Process" for the why.

## Provisioning a fresh Pi 5

```bash
cd ~/Recon-Platform-R2/roomba_ws
bash environment.sh             # full install + verify
bash environment.sh --check     # verify-only on an already-set-up device
```

Then build the ROS2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Using it

Once `setup.sh sensor-test` is running, point a browser at
`http://<pi-ip>/map` (or `http://recon.local/map` over the AP). Walk the
device through a room slowly. The map builds itself; click **Save Map** when
you're done. See [`docs/STATUS.md`](docs/STATUS.md) for what's wired up
today and [`docs/ROADMAP.md`](docs/ROADMAP.md) for what's coming.

---

## Status at a glance

| Stage | Title                                       | State        |
| ----- | ------------------------------------------- | ------------ |
| H1    | Pivot to handheld — rename + delete         | ✅ Done       |
| H1.5  | Cleanup — vestigial autonomy code           | ✅ Done       |
| H1.6  | Live-pose viewport, faster web, SLAM tune   | ✅ Done       |
| H2.0  | ESP32 firmware skeleton                     | ✅ Done       |
| H2.1  | Pi-side ESP32 bridge + `/stats` page        | ✅ Done       |
| H3    | IMU fusion (yaw integrator + EKF + /odom)   | ✅ Done       |
| H3.1  | Madgwick roll/pitch + bench calibration     | ⏳ Planned    |
| H4    | IMU-aided SLAM, first walking test          | ⏳ Planned    |
| H5    | Scan-session state machine + UI rework      | ⏳ Planned    |
| H6    | Enclosure + power integration (22.5 W USB-C power bank) | ⏳ Planned    |

Full detail: [`docs/STATUS.md`](docs/STATUS.md), [`docs/ROADMAP.md`](docs/ROADMAP.md).

---

## License

MIT. See LICENSE (TBA).
