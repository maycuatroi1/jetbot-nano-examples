# JetBot Nano - Control Examples

Source code and notebooks that teach how to control a **JetBot** robot car running on an **NVIDIA Jetson Nano** (JetBot / Waveshare JetBot ROS AI Kit class). The control library is rewritten from scratch and does not depend on NVIDIA's `jetbot` package, so it works on a stock JetPack image. Good for learners getting started with the camera, LIDAR, motors and power sensor.

## Supported hardware

| Component | Connection | Notes |
|---|---|---|
| CSI camera IMX219-160 | `/dev/video0` | read via OpenCV + GStreamer (`nvarguscamerasrc`) |
| RPLIDAR A1M8 | `/dev/ttyUSB0` (CP2102) | 360 degree scan |
| Motor driver PCA9685 | I2C bus 1, `0x60` | Adafruit MotorHAT pin layout |
| OLED display SSD1306 | I2C bus 1, `0x3C` | status display |
| Power sensor INA219 | I2C bus 1, `0x41` | battery voltage (3S ~12V) |

## Install on the Jetson Nano

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-smbus python3-matplotlib jupyter-notebook python3-ipywidgets
pip3 install rplidar-roboticia
sudo usermod -aG dialout,i2c,video $USER
```

Log out and back in for the new group permissions to apply.

## Quick start from the command line

```bash
python3 jbot.py battery        # read battery voltage
python3 jbot.py lidar          # distance in 4 sectors (front/right/back/left)
python3 jbot.py snap out.jpg   # capture a single frame
python3 jbot.py fwd 0.3 1      # drive forward at speed 0.3 for 1 second
python3 jbot.py test           # run a forward/back/left/right demo
```

> Safety: always **lift the wheels off the ground** before testing the motors.

## Use from Python

```python
from jbot import Robot, read_battery, snapshot, lidar_sectors

robot = Robot()
robot.forward(0.3)
robot.stop()

print(read_battery(), "V")
print(lidar_sectors())
snapshot("frame.jpg")
```

`Robot.set_motors(left, right)` takes values from -1.0 to 1.0 for each wheel.

## Lesson notebooks

Open the `notebooks/` folder in Jupyter to follow the lessons step by step, each with visualizations:

1. `01_system_and_battery.ipynb` - system info, read and plot INA219 battery voltage
2. `02_camera.ipynb` - live camera view, capture a frame, color histogram
3. `03_motors.ipynb` - drive the motors with sliders and buttons, plot the motion vector
4. `04_lidar.ipynb` - scan the LIDAR and draw a 360 degree radar map
5. `05_obstacle_avoidance.ipynb` - combine LIDAR and motors for reactive obstacle avoidance

```bash
cd notebooks
jupyter notebook --ip=0.0.0.0 --no-browser
```

## Live dashboard

A single-page web dashboard shows the camera feed and the LIDAR radar at the same
time, plus battery and the four sector distances. It is a self-contained server
using only the Python standard library (no Flask), with a minimalist light theme.

```bash
python3 dashboard.py
```

Then open `http://<jetson-ip>:8000/` from any device on the same network.

It also has **drive controls**: a hold-to-move directional pad (release to stop), a
speed slider, and keyboard support (W A S D or arrow keys). A server-side watchdog
stops the motors automatically if no command arrives for 0.6 s, so the robot halts
if the connection drops. Lift the wheels off the ground before driving.

The dashboard holds the camera and LIDAR continuously, so close the camera/LIDAR
cells in the notebooks while it runs (only one process can use each device).

## LIDAR notes: dead zone and calibration

- **Dead zone**: the RPLIDAR A1 has a minimum range of about 15 cm. Objects closer
  than that return no measurement at all, so a wall pressed against the robot is
  invisible to the LIDAR. The obstacle-avoidance logic therefore treats "no return
  in front" as blocked, not clear.
- **Angle calibration**: the radar assumes LIDAR angle 0 points to the front of the
  robot. If it is mounted rotated, place an object straight ahead (about 40 cm) and
  run:

  ```bash
  python3 jbot.py lidar-calib
  ```

  Read the reported `nearest angle`, then set `LIDAR_ANGLE_OFFSET` in `jbot.py` to
  that value. After that, `bucket()`, `lidar_sectors()` and the dashboard radar all
  align "front" with the real front.

## Motor pin map (PCA9685, Adafruit MotorHAT style)

| Motor | PWM | IN1 | IN2 |
|---|---|---|---|
| 1 (left) | 8 | 10 | 9 |
| 2 (right) | 13 | 11 | 12 |

If a wheel spins the wrong way, swap `left`/`right` or flip `left_alpha`/`right_alpha` when constructing `Robot`.

## License

MIT - see the `LICENSE` file.
