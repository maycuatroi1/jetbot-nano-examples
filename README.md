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

## Motor pin map (PCA9685, Adafruit MotorHAT style)

| Motor | PWM | IN1 | IN2 |
|---|---|---|---|
| 1 (left) | 8 | 10 | 9 |
| 2 (right) | 13 | 11 | 12 |

If a wheel spins the wrong way, swap `left`/`right` or flip `left_alpha`/`right_alpha` when constructing `Robot`.

## License

MIT - see the `LICENSE` file.
