import sys
import time

import smbus

PCA9685_ADDR = 0x60
INA219_ADDR = 0x41

LIDAR_ANGLE_OFFSET = 0.0
LIDAR_MIN_RANGE = 0.15

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

_MOTORS = {
    1: dict(pwm=8, in2=9, in1=10),
    2: dict(pwm=13, in2=12, in1=11),
    3: dict(pwm=2, in2=3, in1=4),
    4: dict(pwm=7, in2=6, in1=5),
}


class PCA9685:
    def __init__(self, bus=1, addr=PCA9685_ADDR, freq=1600):
        self.bus = smbus.SMBus(bus)
        self.addr = addr
        self.set_all_pwm(0, 0)
        self.bus.write_byte_data(self.addr, MODE1, 0x00)
        time.sleep(0.01)
        self.set_freq(freq)

    def set_freq(self, freq):
        prescale = int(round(25000000.0 / (4096.0 * freq)) - 1)
        old = self.bus.read_byte_data(self.addr, MODE1)
        self.bus.write_byte_data(self.addr, MODE1, (old & 0x7F) | 0x10)
        self.bus.write_byte_data(self.addr, PRESCALE, prescale)
        self.bus.write_byte_data(self.addr, MODE1, old)
        time.sleep(0.005)
        self.bus.write_byte_data(self.addr, MODE1, old | 0xA0)

    def set_pwm(self, ch, on, off):
        base = LED0_ON_L + 4 * ch
        self.bus.write_byte_data(self.addr, base, on & 0xFF)
        self.bus.write_byte_data(self.addr, base + 1, on >> 8)
        self.bus.write_byte_data(self.addr, base + 2, off & 0xFF)
        self.bus.write_byte_data(self.addr, base + 3, off >> 8)

    def set_all_pwm(self, on, off):
        for reg in (0xFA, 0xFB, 0xFC, 0xFD):
            pass
        self.bus.write_byte_data(self.addr, 0xFA, on & 0xFF)
        self.bus.write_byte_data(self.addr, 0xFB, on >> 8)
        self.bus.write_byte_data(self.addr, 0xFC, off & 0xFF)
        self.bus.write_byte_data(self.addr, 0xFD, off >> 8)

    def set_pin(self, pin, value):
        if value:
            self.set_pwm(pin, 4096, 0)
        else:
            self.set_pwm(pin, 0, 4096)


class Robot:
    def __init__(self, bus=1, left=1, right=2, left_alpha=1, right_alpha=1):
        self.pca = PCA9685(bus=bus)
        self.left = _MOTORS[left]
        self.right = _MOTORS[right]
        self.left_alpha = left_alpha
        self.right_alpha = right_alpha

    def _drive(self, motor, value):
        value = max(-1.0, min(1.0, value))
        speed = int(min(abs(value) * 255, 255))
        self.pca.set_pwm(motor["pwm"], 0, speed * 16)
        if value > 0:
            self.pca.set_pin(motor["in2"], 0)
            self.pca.set_pin(motor["in1"], 1)
        elif value < 0:
            self.pca.set_pin(motor["in1"], 0)
            self.pca.set_pin(motor["in2"], 1)
        else:
            self.pca.set_pin(motor["in1"], 0)
            self.pca.set_pin(motor["in2"], 0)

    def set_motors(self, left_value, right_value):
        self._drive(self.left, left_value * self.left_alpha)
        self._drive(self.right, right_value * self.right_alpha)

    def forward(self, speed=0.4):
        self.set_motors(speed, speed)

    def backward(self, speed=0.4):
        self.set_motors(-speed, -speed)

    def left_turn(self, speed=0.4):
        self.set_motors(-speed, speed)

    def right_turn(self, speed=0.4):
        self.set_motors(speed, -speed)

    def stop(self):
        self.set_motors(0, 0)


def read_battery(bus=1, addr=INA219_ADDR):
    b = smbus.SMBus(bus)
    raw = b.read_word_data(addr, 0x02)
    swapped = ((raw << 8) & 0xFF00) + (raw >> 8)
    return (swapped >> 3) * 0.004


def gst_pipeline(width=1280, height=720, fps=30, flip=0):
    return (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1 ! "
        "nvvidconv flip-method=%d ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink drop=1 max-buffers=1"
        % (width, height, fps, flip)
    )


def snapshot(path="/tmp/snap.jpg", width=1280, height=720):
    import cv2

    cap = cv2.VideoCapture(gst_pipeline(width, height), cv2.CAP_GSTREAMER)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("camera read failed")
    cv2.imwrite(path, frame)
    return path


def bucket(angle):
    a = (angle - LIDAR_ANGLE_OFFSET) % 360
    if a >= 315 or a < 45:
        return "front"
    if a < 135:
        return "right"
    if a < 225:
        return "back"
    return "left"


def lidar_sectors(port="/dev/ttyUSB0", max_scans=3):
    from rplidar import RPLidar

    lidar = RPLidar(port, baudrate=115200)
    points = []
    try:
        for i, scan in enumerate(lidar.iter_scans(max_buf_meas=2000)):
            points = [(angle, dist) for (_, angle, dist) in scan if dist > 0]
            if i + 1 >= max_scans:
                break
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    sectors = {"front": [], "right": [], "back": [], "left": []}
    for angle, dist in points:
        sectors[bucket(angle)].append(dist)
    return {k: (min(v) / 1000.0 if v else None) for k, v in sectors.items()}


def lidar_calib(port="/dev/ttyUSB0"):
    from rplidar import RPLidar

    lidar = RPLidar(port, baudrate=115200)
    try:
        for scan in lidar.iter_scans(max_buf_meas=2000):
            valid = [(a, d) for (_, a, d) in scan if d > 0]
            if not valid:
                print("no returns (object may be inside the ~15cm dead zone)")
                continue
            a, d = min(valid, key=lambda x: x[1])
            print(
                "nearest angle=%6.1f deg  dist=%6.0f mm  ->  current sector: %s"
                % (a, d, bucket(a))
            )
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()


def focus_meter(samples=0, width=1280, height=720):
    import cv2

    cap = cv2.VideoCapture(gst_pipeline(width, height), cv2.CAP_GSTREAMER)
    count = 0
    best = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
            best = max(best, sharp)
            bar = "#" * int(min(sharp, 400) / 8)
            print("sharpness %7.1f  best %7.1f  %s" % (sharp, best, bar))
            count += 1
            if samples and count >= samples:
                return best
    finally:
        cap.release()


def _cli():
    args = sys.argv[1:]
    if not args:
        print("usage: jbot.py [battery|snap|lidar|lidar-calib|focus|fwd|back|left|right|stop|test] [speed] [secs]")
        return
    cmd = args[0]
    if cmd == "battery":
        print("%.2f V" % read_battery())
        return
    if cmd == "snap":
        print(snapshot(args[1] if len(args) > 1 else "/tmp/snap.jpg"))
        return
    if cmd == "lidar":
        for k, v in lidar_sectors().items():
            print("%-6s %s" % (k, ("%.2f m" % v) if v else "n/a"))
        return
    if cmd == "focus":
        focus_meter(int(args[1]) if len(args) > 1 else 0)
        return
    if cmd == "lidar-calib":
        lidar_calib()
        return
    speed = float(args[1]) if len(args) > 1 else 0.4
    secs = float(args[2]) if len(args) > 2 else 1.0
    r = Robot()
    moves = dict(fwd=r.forward, back=r.backward, left=r.left_turn, right=r.right_turn)
    if cmd == "stop":
        r.stop()
        return
    if cmd == "test":
        for name in ("fwd", "back", "left", "right"):
            print(name)
            moves[name](0.3)
            time.sleep(0.6)
            r.stop()
            time.sleep(0.4)
        return
    if cmd in moves:
        moves[cmd](speed)
        time.sleep(secs)
        r.stop()
        return
    print("unknown command:", cmd)


if __name__ == "__main__":
    _cli()
