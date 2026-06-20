import sys
import time

sys.path.insert(0, "..")

from jbot import read_battery


def main():
    while True:
        voltage = read_battery()
        percent = max(0, min(100, (voltage - 9.0) / (12.6 - 9.0) * 100))
        print("battery: %.2f V  (~%.0f%%)" % (voltage, percent))
        time.sleep(1.0)


if __name__ == "__main__":
    main()
