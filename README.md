# BackHack

A wearable posture tracker built for **HackPSU Fall 2026**. BackHack combines an **ESP32 + MPU6050 accelerometer** with optional **webcam pose detection** to detect sustained changes from your calibrated posture. It provides a live phone/computer dashboard, a wearable buzzer, and laptop audio alerts.

When both sensors are tracking, sound alerts require agreement. If only one is available, that system can alert independently. The current implementation uses threshold-based agreement, not a numerical confidence score.

## What you need

- ESP32 DOIT DevKit V1, MPU6050 module, compatible buzzer, jumper wires, and a **USB data cable**.
- Windows or macOS computer with [VS Code](https://code.visualstudio.com/).
- Optional: webcam and [Python](https://www.python.org/downloads/) for camera tracking; Python 3.11 is a reasonable starting point.

| Component | Windows | macOS |
| --- | --- | --- |
| Firmware build/upload | Supported through PlatformIO | Supported through PlatformIO |
| Wi-Fi dashboard | Supported | Supported |
| `CameraVision.py` | Uses Windows audio (`winsound`) | **Requires a code change:** replace `winsound` import and playback/stop calls with macOS-compatible audio |

## 1. Install VS Code extensions and USB drivers

The extensions are the same on both operating systems. USB drivers are separate system installations, not VS Code extensions.

| Extension | Purpose |
| --- | --- |
| [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) | Required for the documented firmware build/upload workflow; already recommended by this repo. |
| [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python) | Recommended for running/debugging the optional camera script and selecting its interpreter. Not required to run Python from a terminal. |

PlatformIO includes its own Core CLI and installs the configured ESP32 toolchain. Arduino IDE and Live Server are not required. Use the repo's PlatformIO configuration; it explicitly discourages installing the competing pioarduino extension and the full C/C++ Extension Pack.

### Windows USB setup

For boards with a **CP2102/CP210x** USB bridge:

1. Download the **CP210x Universal Windows Driver** from [Silicon Labs](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers).
2. Extract the ZIP. In **Device Manager**, find the connected CP210x device, choose **Update driver → Browse my computer for drivers**, and select the extracted folder, including subfolders.
3. Reconnect the ESP32. Under **Ports (COM & LPT)**, note its COM port, such as `COM3`.

Choose the USB port that appears when the board is plugged in—not a **Standard Serial over Bluetooth** port.

### macOS USB setup

For CP2102/CP210x boards, use the **macOS VCP driver** from the same [Silicon Labs download page](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers) if the serial device is not already available. Follow the package's installation instructions and approve the driver in System Settings if prompted, then reconnect the board.

Find its port in PlatformIO's device list; it commonly resembles `/dev/cu.SLAB_USBtoUART`. You can also check:

```bash
ls /dev/cu.*
```

These drivers apply only to CP210x hardware. If your board uses a different USB bridge, install that chip manufacturer's driver instead.

## 2. Download and open the project

On [GitHub](https://github.com/js-2507/backhack), select **Code → Download ZIP**, extract it, then choose **File → Open Folder** in VS Code. Open the folder containing `platformio.ini`.

Alternatively, with [Git](https://git-scm.com/downloads) installed:

```bash
git clone https://github.com/js-2507/backhack.git
cd backhack
```

Open that folder in VS Code and allow PlatformIO to finish installing its dependencies while you have internet access.

## 3. Connect the hardware and upload

The firmware uses these signal pins:

| Component signal | ESP32 GPIO |
| --- | --- |
| MPU6050 SDA | 21 |
| MPU6050 SCL | 22 |
| Buzzer signal | 23 |

Connect common ground and power your sensor module according to its specifications, using 3.3 V-compatible I²C levels. Use a suitable driver circuit if your buzzer needs more current than a GPIO can supply.

1. Connect the ESP32 by USB.
2. In **PlatformIO → Project Tasks → esp32doit-devkit-v1 → General**, select **Build**, then **Upload**.
3. Open **Monitor** at **115200 baud** to see calibration, sensor, and connection messages.

The existing `platformio.ini` selects `espressif32@6.10.0`, the Arduino framework, and `huge_app.csv` partitions. Keep these settings for the current firmware.

If automatic port detection fails, add your actual port under the existing environment in `platformio.ini`:

```ini
; Windows example — replace COM3 with your port
upload_port = COM3
monitor_port = COM3
```

On macOS, use the detected `/dev/cu.*` path instead. If upload stays at `Connecting...`, try holding **BOOT** while upload begins, then release it once writing starts. Close other serial monitors before uploading.

## 4. Start tracking

1. Secure the sensor so it moves with your torso. Sit upright and hold still during startup: automatic calibration begins after a three-second wait.
2. On your phone or computer, join Wi-Fi **`Posture-ESP32`**, password **`Posture2026`**.
3. Open **[http://192.168.4.1](http://192.168.4.1)** in your browser. Stay connected if the operating system reports that this network has no internet.
4. Use the dashboard's start/calibration and pause controls. You can also send **S** or **R** to calibrate and **P** to pause through the serial monitor.

The wearable triggers after a tilt greater than **10° for two seconds**, and resets below **7°**. Change `TILT_THRESHOLD`, `RESET_THRESHOLD`, and `ALERT_DELAY` in `src/main.cpp` to tune these settings, then upload again.

The ESP32 serves the dashboard directly; no separate web server or HTML upload is needed. Its served page is embedded in `src/main.cpp`, so editing `index.html` or `web/index.html` alone does not change the page on the device.

## 5. Optional camera tracking — Windows

Install Python and the packages **before joining the ESP32's Wi-Fi**, which does not provide internet. In a VS Code PowerShell terminal at the repository root:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install opencv-python mediapipe numpy requests
.\.venv\Scripts\python.exe CameraVision.py
```

The first run downloads `pose_landmarker_lite.task` automatically. Once downloaded, press **Q**, join **Posture-ESP32**, and rerun the last command. The script sends camera measurements to `http://192.168.4.1` and reads wearable status to coordinate audio alerts.

Sit upright with both shoulders and an ear visible while the camera gathers **30 calibration frames**. With the camera window focused, press **C** to recalibrate or **Q** to quit. The default camera threshold is **10° for three seconds**. Adjust `CAMERA_INDEX` if the wrong webcam opens.

In VS Code, use **Python: Select Interpreter** to choose `.venv` if you prefer the editor's Run button. The explicit commands above do not require activating the environment.

**macOS:** firmware and the dashboard work independently of this script. The camera script currently imports Windows-only `winsound`; installing more VS Code extensions will not fix that. Its audio implementation must be ported before it can run on macOS.

## Bluetooth and current limitations

The firmware also advertises **Posture-ESP32** over BLE using the Nordic UART Service. For a desktop BLE-only dashboard, serve the repository locally with `python -m http.server 8000` (or `python3` on macOS), open `http://localhost:8000`, and select the Bluetooth tab in a Web Bluetooth-capable browser such as Chrome. Web Bluetooth needs a secure context; the ESP32's plain HTTP address is intended for Wi-Fi mode.

Camera and agreement panels still use HTTP endpoints and will not work through BLE alone. The standalone HTML currently resolves those endpoints relative to its own server, so use the ESP32-hosted dashboard for the complete Wi-Fi experience. Wi-Fi mode is also the simplest phone setup.

The current repo provides dashboard status and buzzer/laptop sounds; it does **not** implement background phone push notifications or SMS.

## Troubleshooting

- **No serial port:** try a known USB data cable, another USB port, and the driver matching your board's USB bridge.
- **`MPU6050 NOT FOUND!`:** check power, ground, SDA/SCL wiring, and the sensor's address (`0x68` or `0x69`).
- **Camera upload fails:** connect the laptop to `Posture-ESP32`, confirm `http://192.168.4.1` opens, and upload the current `src/main.cpp`.
- **Camera will not open:** enable camera access for desktop applications, close other webcam apps, or try `CAMERA_INDEX = 1`.
- **Python package is missing:** install it with the same `.venv` interpreter used to run the script.

## Project files

| File | Purpose |
| --- | --- |
| `src/main.cpp` | ESP32 sensing, calibration, buzzer, Wi-Fi server, BLE, and embedded dashboard |
| `platformio.ini` | Board, framework, platform version, and build settings |
| `CameraVision.py` | Webcam pose detection, calibration, and laptop audio |
| `index.html`, `web/index.html` | Standalone dashboard copies |
| `old versions/` | Earlier firmware versions; not the active upload target |

Setup references: [PlatformIO for VS Code](https://docs.platformio.org/en/latest/integration/ide/vscode.html), [CP210x drivers](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers), [MediaPipe Python setup](https://developers.google.com/edge/mediapipe/solutions/setup_python), and [Web Bluetooth](https://developer.chrome.com/docs/capabilities/bluetooth).
