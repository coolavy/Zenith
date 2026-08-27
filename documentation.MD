# Zenith
Zenith is a high-performance, NPU-accelerated object detection system designed for the Orange Pi 5 developed by FRC Team 9015. It detects game pieces, calculates their 3D position relative to the robot's center, and handles clustering for multiple targets.

## Features
* **NPU Accelerated:** Runs YOLOv8 models at 60+ FPS using the RK3588 NPU (INT8).
* **Pose Estimation:** Calculates distance and yaw using camera intrinsics and trigonometry.
* **Angled Mount Compensation:** Built-in math to handle cameras mounted on non-level surfaces.
* **Target Clustering:** Automatically groups multiple game pieces and finds the center of the "cluster".
* **NT4 Integration:** Communicates directly with the RoboRIO using high-speed NetworkTables.

---

## Hardware Requirements
* **Coprocessor:** Orange Pi 5 / 5B / 5 Plus.
* **Camera:** Arducam B0495 (USB 3.0 Global Shutter).
* **Storage:** 16GB+ MicroSD or NVMe SSD.
* **Connection:** Ethernet to the FRC Robot Radio.

---

## Setup & Installation

### 1. Install System Dependencies
```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev libntcore-dev libwpiutil-dev libwpinet-dev
```

### 2. Install Rockchip NPU Drivers
Ensure the RKNN API files are in your system paths:
* **Headers:** `rknn_api.h` should be in `/usr/include/`.
* **Libraries:** `librknnrt.so` should be in `/usr/lib/`.
* Run `sudo ldconfig` after moving files.

### 3. Required File Structure
```text
~/Zenith/
├── main.cpp              # The core engine
├── CMakeLists.txt        # Build instructions
├── model.rknn            # YOLOv8 INT8 Model
└── camera_params.yaml    # Generated via calibration
```

---

## Building and Running

1. **Compile:**
   ```bash
   cd ~/Zenith
   mkdir -p build && cd build
   cmake ..
   make
   ```

2. **Run:**
   ```bash
   ./zenith_vision
   ```

---

## Configuration & Tuning

Before competition, you **must** adjust the physical constants in `main.cpp` to match your robot:

| Constant | Description |
|----------|-------------|
| `MOUNT_PITCH` | The angle (degrees) the camera is tilted **UP** from the floor. |
| `ROBOT_X_OFFSET` | Distance from robot center to lens (meters). |
| `CAM_H` | Height of the lens center from the floor (meters). |
| `CONF_THRESH` | Confidence value (0.0 - 1.0) to filter detections. |

**Tip:** If the reported distance is shorter than a tape measure, increase `MOUNT_PITCH`.

---

## Robot Side Implementation (Java)

1. **Subsystem:** Create a `Vision.java` subsystem. It should subscribe to the `ZenithVision` NetworkTable.
2. **Command:** Use the `ZenithPursuit.java` command to control the drivetrain.
   * **Search State:** Robot spins if `hasTarget` is false.
   * **Chase State:** Robot uses PID to drive to the published `targetX` and `targetY`.

```java
// Example Trigger in RobotContainer.java
m_driverController.a().whileTrue(new ZenithPursuit(m_drive, m_vision));
```

---

## Deployment
To make Zenith start automatically when the robot powers on, use the provided `zenith.service` file and enable it via systemd:
```bash
sudo cp zenith.service /etc/systemd/system/
sudo systemctl enable zenith.service
```

Developed by FRC Team 9015 - Questionable Engineering.
