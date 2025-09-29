Here’s a **complete `README.md`** with your content plus **GitHub-safe Mermaid graphs** for the Node (NA) and Gateway (GA) behavior, and an **end-to-end event/clear sequence**. You can copy-paste this straight into your repo.

---

# Forest Guard — Decentralized Edge-AI LoRa Mesh Network for Forest Surveillance

![Hero](https://github.com/MukeshSankhla/Forest-Guard/blob/main/Images/Header.png)

> Solar-powered sensor nodes + LoRa Meshtastic mesh + Edge Impulse gunshot detection + Firebase dashboard.
---

## 🌲 What is Forest Guard?

**Forest Guard** is a field-deployable forest monitoring system designed for **zero-infrastructure** locations (no grid power / no cellular). It uses:

* **Solar-powered ESP32-S3 nodes** with sensors (Env + Smoke + Mic) and optional **GNSS**
* **LoRa Meshtastic** radios for long-range **mesh** communication
* **Edge Impulse** on-device **gunshot detection** (audio classification)
* A **Gateway** (Arduino UNO R4 WiFi) that aggregates mesh traffic, logs to **Firebase RTDB**, drives a **TFT UI**, and sounds a **buzzer**
* A **web dashboard** visualizing nodes on a **map** with **alerts** and quick stats

![NextPCB](https://github.com/MukeshSankhla/Forest-Guard/blob/main/Images/NextPCB.JPG)
![PCBA](https://github.com/MukeshSankhla/Forest-Guard/blob/main/Images/PCBA.JPG)
![PCB](https://github.com/MukeshSankhla/Forest-Guard/blob/main/Images/DSC02566.JPG)

---

## ✨ Key Features

* **Decentralized mesh:** LoRa (Meshtastic) lets nodes relay data without internet
* **Edge AI at the node:** Real-time gunshot detection via **Edge Impulse**
* **Early fire alerts:** **Smoke sensor** triggers a LoRa alert
* **GNSS (optional):** Location + time when satellites > 3
* **Power-aware:** Solar + battery design
* **Robust gateway:** NTP time gate; dedup events; Firebase sync; non-blocking buzzer
* **Dashboard:** Map, node cards, charts, and alert acknowledgment

---

## 🧱 Hardware

### Node (each unit)

* ESP32-S3-WROOM-1 (16MB/8MB) on **Custom PCB**
* **RP2040 LoRa** (Meshtastic firmware)
* Env sensor (Temp/Humidity/UV/Light/Pressure), **Smoke** sensor, **I²S mic**
* **Optional GNSS** module
* **Li-Po + Solar** (charging/power management on PCB)
* 3D printed enclosure (Fusion 360 + STLs included)

![Node BOM Overview](https://github.com/MukeshSankhla/Forest-Guard/blob/main/Images/N_BOM.JPG)

### Gateway

* **Arduino UNO R4 WiFi**
* **3.5” 480×320 TFT** (Fermion)
* **LoRa** module (UART)
* **Buzzer** + **Power switch**
* 3D printed enclosure

![Gateway BOM Overview](https://github.com/MukeshSankhla/Forest-Guard/blob/main/Images/G_BOM.JPG)

---

## 🔗 Network & Message Protocol

* Frames are **parsed strictly between `#` and `*`** to ignore LoRa noise/garbage.
* **Registration:** Node sends `#<NODE_ID>*` every **10 s** until GA replies `#<NODE_ID>+OK*`.
* **Telemetry (every 5 s):**

  * `#E,<ID>,temp,humidity,uv,lux,pressure,alt*`
  * `#L,<ID>,lat,lon*` *(send only when GNSS sats > 3)*
* **Events (retry every 10 s until cleared):**

  * Fire: `#F+<randId>,<ID>,<smoke>,YYYY/MM/DD,HH:MM:SS*` or `NT`
  * Gun:  `#G+<randId>,<ID>,<score>,YYYY/MM/DD,HH:MM:SS*` or `NT`
* **Clear:** GA broadcasts `#<NODE_ID>+C*` after operator clears the event in the dashboard.

---

## ☁️ Firebase Schema (exact keys/paths)

The Gateway logs only when **NTP epoch ≥ 2025-01-01** and when values **change**:

```
nodes/<ID>/env/<epoch>   { "temp", "humi", "uvi", "li", "pres", "alt" }
nodes/<ID>/Loc/<epoch>   { "lat", "lon" }                # Note capital 'L' in 'Loc'
nodes/<ID>/fire/<epoch>  { "value", "NodeTime" }
nodes/<ID>/gun/<epoch>   { "score", "NodeTime" }
nodes/<ID>/meta          { "Event", "lastSeenAt" }       # Event=true causes buzzer
```

The dashboard toggles `nodes/<ID>/meta/Event` to **false** to acknowledge/clear.

---

## 🧠 Edge Impulse Model

* **Data capture:** Flask tool reads ESP32 mic over **Serial** and uploads to EI using API key.
* **Dataset:** Collect `Noise` and `Gunshot` audio in varied environments; split **80:20** train:test.
* **Preprocessing:** **MFCC** (Audio) features; split 10-s recordings into **1-s** windows.
* **Model:** **Classification** (1D CNN works well); export **Arduino library** and include in Node.
* **On-device:** Runs on ESP32-S3 (Core 1) so LoRa/sensors remain non-blocking.

---

## 📈 How It Works (Behavior Graphs)

> These Mermaid diagrams render on GitHub and document the exact behavior.
> **Note:** ASCII only, one edge per line, to avoid parser errors.

### NA (ESP32-S3 Node) – Behavior Logic

```mermaid
graph TD
    A[Boot] --> B[LED Blue (breath)]
    B --> C[Init sensors + LoRa + optional GNSS]
    C --> D{GNSS_AVAILABLE?}
    D -->|No| E[Set time = NT; loc = INITIAL_LAT/LON]
    E --> R[Registration Loop]
    D -->|Yes| F{Satellites > 3?}
    F -->|No| G[Use GNSS time if available; keep trying; loc = INITIAL]
    G --> R
    F -->|Yes| H[Use GNSS time + location]
    H --> R

    R[Send '#NODE_ID*' every 10s] --> I{ACK '#NODE_ID+OK*' from GA?}
    I -->|No| R
    I -->|Yes| J[Registered; start timers]

    J --> K[Every 5s: read ENV]
    K --> L[Send '#E,ID,t,h,uv,lux,p,alt*' ; LED Green (breath)]
    L --> M{Satellites > 3?}
    M -->|Yes| N[Send '#L,ID,lat,lon*']
    M -->|No| O[Skip LOC this cycle]
    N --> P[Main loop]
    O --> P

    P --> Q{Event? Gunshot score >= THRESH OR Smoke >= THRESH}
    Q -->|No| P
    Q -->|Yes| S[Latch EVENT; LED Red (breath); eventId = random(0..100)]
    S --> T[Every 10s send F+ or G+ frame with date/time or NT]
    T --> U{Received '#NODE_ID+C*' ?}
    U -->|No| T
    U -->|Yes| V[Clear event; reset flags; allow new events]
    V --> P
```

### GA (UNO R4 WiFi Gateway) – Behavior Logic

```mermaid
graph TD
    A[Boot] --> B[Init WiFi + TFT + LoRa + NTP]
    B --> C{NTP epoch >= 2025?}
    C -->|No| B
    C -->|Yes| D[Ready to log with real time]

    D --> E[Read LoRa; parse only '#...*']
    E --> F{Frame type?}

    F -->|"#<NODE_ID>*"| G[Save nodeId; send '#<NODE_ID>+OK*']
    G --> D

    F -->|E,ID,t,h,uv,lux,p,alt| H[Update ENV cache]
    H --> I{Delta >= +/-1.0 on any field?}
    I -->|Yes| J[PUT /nodes/ID/env/<epoch>; PATCH /nodes/ID/meta{lastSeen}]
    I -->|No| D
    J --> D

    F -->|L,ID,lat,lon| K[Update last location]
    K --> L{Moved >= 0.00010 deg (~11 m)?}
    L -->|Yes| M[PUT /nodes/ID/Loc/<epoch>]
    L -->|No| D
    M --> D

    F -->|F+id or G+id| N{Already logged or lastCleared?}
    N -->|Yes| O[Re-broadcast '#ID+C*' (duplicate)]
    O --> D
    N -->|No| P[Set /nodes/ID/meta/Event = true; Log to fire/gun; Start buzzer]
    P --> Q[Poll /nodes/ID/meta/Event every 2s]
    Q --> R{Event flag false?}
    R -->|No| Q
    R -->|Yes| S[Broadcast '#ID+C*' x3; stop buzzer; remember lastClearedId]
    S --> D
```

### End-to-End Event & Clear Sequence

```mermaid
sequenceDiagram
    autonumber
    participant NA as Node (ESP32-S3)
    participant GA as Gateway (UNO R4 WiFi)
    participant FB as Firebase RTDB
    participant DB as Web Dashboard

    NA->>GA: #G+eid,ID,score,DATE/TIME|NT*
    GA->>FB: set nodes/ID/meta/Event = true
    GA->>FB: push nodes/ID/gun/<epoch> {score, NodeTime}
    GA->>GA: Buzzer ON (non-blocking)
    GA->>DB: Data visible on dashboard

    loop Poll
        GA->>FB: read nodes/ID/meta/Event
    end

    DB->>FB: operator sets nodes/ID/meta/Event = false
    GA->>GA: Buzzer OFF
    GA-->>NA: #ID+C* (repeat a few times)
    NA->>NA: clear event; resume telemetry
```

---

## 🧪 Quick Start

### 1) Edge Impulse

1. Create a project at [https://studio.edgeimpulse.com/](https://studio.edgeimpulse.com/)
2. Get **Project API Key**: **Dashboard → Keys**
3. Collect data with the Flask tool:

   ```bash
   cd "Edge Impulse Data Tool"
   pip install -r requirements.txt
   python app.py
   # open http://127.0.0.1:5000, select COM port, paste API key, set label, click Capture
   ```
4. Split samples to **1 s**, generate **MFCC features**, train **Classifier**, then **Deploy → Arduino library** (.zip)

### 2) Node Firmware (ESP32-S3)

* **Arduino IDE → Preferences → Additional Boards URLs:**

  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```
* Install **esp32 by Espressif Systems**
* Open `Node/Node_V2.ino`
* **Libraries:**

  * Import EI **Arduino .zip** (from Deploy)
  * DFRobot GNSS: [https://github.com/DFRobot/DFRobot_GNSS](https://github.com/DFRobot/DFRobot_GNSS)
  * DFRobot Environmental Sensor: [https://github.com/DFRobot/DFRobot_EnvironmentalSensor](https://github.com/DFRobot/DFRobot_EnvironmentalSensor)
  * Adafruit NeoPixel (Library Manager)
* **Configure in code:**

  ```cpp
  #include <Your_EI_Model_inferencing.h>
  #define NODE_ID "01"
  static const bool GNSS_AVAILABLE = true;      // or false
  static const float INITIAL_LAT = 17.6783352f; // fallback when GNSS false
  static const float INITIAL_LON = 77.6058542f;
  ```
* **Board/Port:**

  * Tools → Board → **ESP32 → DFRobot FireBeetle 2 ESP32-S3**
  * Tools → USB CDC On Boot → **Disable**
  * Upload

### 3) Gateway (Arduino UNO R4 WiFi)

* Open `Gateway/Gateway_Vx.ino`
* Install: **WiFiS3**, **ArduinoHttpClient**, **NTPClient**, **DFRobot UI/Display**
* Set Wi-Fi + Firebase:

  ```cpp
  const char* WIFI_SSID = "your_ssid";
  const char* WIFI_PASS = "your_pass";
  const char* FB_HOST   = "your-project-id.asia-southeast1.firebasedatabase.app";
  const char* FB_AUTH   = "your_firebase_database_secret_or_token";
  ```
* Upload; wait for **NTP ≥ 2025-01-01**; GA will then log to Firebase.

---

## 🖥️ Dashboard

Built with **Lovable.dev**, the dashboard:

* Shows **map** with node markers (Active=Green, Inactive=Gray, Alert=Red)
* Provides **quick cards** (Total nodes, Active/Inactive, Recent Alerts)
* Lets you **open a node** to view ENV trends and event history
* **Acknowledge** an alert by toggling `meta/Event=false`, which triggers GA to broadcast `#<ID>+C*`

![Dashboard – Map](https://github.com/MukeshSankhla/Forest-Guard/blob/main/Images/Screenshot%202025-09-16%20101348.png)
![Dashboard – Node Details](https://github.com/MukeshSankhla/Forest-Guard/blob/main/Images/Screenshot%202025-09-16%20101044.png)

---

## ⚙️ Reliability Details

* **Non-blocking timers** for LoRa events & sensor reads
* **Dedup events** by `eventId` on GA; re-ACK `CLEAR` if NA resends same event
* **Telemetry throttle:** upload to Firebase **only when changed**

  * ENV delta ≥ **±1.0**
  * LOC moved by ≥ **0.00010°** (~11 m)
* **NTP warm-up gate:** no cloud writes until epoch ≥ **2025-01-01**
* **LoRa framing:** parse **only `#...*`** to ignore garbage

---

## 🧑‍💻 Development Notes

* **LoRa UART:** Prefer hardware UART if available; otherwise align to module baud + buffer sizes.
* **Edge Impulse thresholds:** tune decision threshold to control sensitivity / false positives.
* **Power:** size solar + battery for your latitude and canopy; log voltage if available.

---

## 🤝 Contributing

PRs welcome! Please open an **issue** for bugs/requests, or discuss features (additional classifiers like chainsaw detection, species calls, etc.).

---

## 📜 License

**MIT License** — see [LICENSE].
Hardware designs and 3D models are shared for educational and non-commercial replication; please attribute the project.

---

## 🙌 Acknowledgments

* **Edge Impulse** for on-device ML
* **Meshtastic** community for the LoRa mesh firmware
* **DFRobot** sensors & display libs
* **NextPCB** for PCB fabrication/assembly support

---

### Contact

**Author:** Mukesh Sankhla — [https://www.makerbrains.com](https://www.makerbrains.com)
**Hackster:** [https://www.hackster.io/Mukesh_Sankhla](https://www.hackster.io/Mukesh_Sankhla)

---

> If you want, I can also create a **banner SVG** (system overview) that matches your color scheme and add it to `/Images/` for the top of this README.
