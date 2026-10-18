# ImageAI Controlled Rover

An AI-powered rover controlled via a web dashboard.  
The Raspberry Pi runs YOLO object detection on a live ESP32-CAM stream and steers an Arduino-driven chassis over WiFi.

---

## Folder Structure

```
imageAI controlled rover/
├── esp32cam_wifi_stream/           ← Arduino sketch for the ESP32-CAM
│   ├── board_config.h              #   pin / camera model definitions
│   ├── esp32cam_wifi_stream.ino    #   main sketch (flash + MJPEG stream + UART bridge)
│   └── wifi_secrets.h              #   WiFi SSID & password (keep private)
│
├── motor_controller/               ← Arduino sketch for the motor-driver board
│   └── motor_controller.ino        #   listens on UART; drives L298N / Motor Shield
│
└── server/                         ← Raspberry Pi Python server (self-contained)
    ├── templates/
    │   └── index.html              #   Web dashboard (Flask Jinja2 template)
    ├── app.py                      #   Flask routes – video feed, commands, status
    ├── requirements.txt            #   pip dependencies
    └── rover_ai.py                 #   Camera, Vision (YOLO), Arduino bridge, behaviours
```

---

## Architecture

```
[Browser] ──HTTP──► [Pi / Flask server]
                         │
                    rover_ai.py
                         │
              ┌──────────┴──────────┐
              │                     │
         Camera thread         Arduino class
         (MJPEG stream)        (HTTP bridge)
              │                     │
         Vision thread              │
         (YOLO detect)              │
              └──────────┬──────────┘
                         │  WiFi
                    [ESP32-CAM]
                         │  UART (TX/RX)
                    [Arduino + Motor Shield]
                         │
                      [Motors]
```

---

## Quick Start

### 1. Flash the Arduino sketches
| Sketch | Board |
|---|---|
| `motor_controller/motor_controller.ino` | Arduino Uno / Nano |
| `esp32cam_wifi_stream/esp32cam_wifi_stream.ino` | AI-Thinker ESP32-CAM |

Edit `wifi_secrets.h` with your WiFi credentials before flashing the ESP32-CAM.

### 2. Install Python dependencies (on the Pi / PC)
```bash
cd server
pip install -r requirements.txt
```

### 3. Download the YOLO model
Place `yolo11n.onnx` inside the `server/` directory (same folder as `app.py`).

### 4. Run the server
```bash
cd server
python app.py
```

Open `http://<pi-ip>:5000` in your browser.

### CLI flags
| Flag | Default | Description |
|---|---|---|
| `--cam-url` | `http://esp32cam.local:81/stream` | ESP32-CAM MJPEG stream URL |
| `--model` | `yolo11n.onnx` | ONNX model file |
| `--conf` | `0.40` | Detection confidence threshold |
| `--speed` | `190` | Forward speed (0–255) |
| `--turn` | `175` | Turning speed (0–255) |
| `--flip` | *(off)* | Rotate camera image 180° |
| `--dry-run` | *(off)* | Print motor commands, skip serial |
| `--no-auto-flash` | *(off)* | Disable automatic flash LED |
