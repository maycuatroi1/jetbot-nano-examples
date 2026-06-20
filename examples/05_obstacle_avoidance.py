import sys
import time

sys.path.insert(0, "..")

from jbot import Robot, lidar_sectors

STOP_DISTANCE = 0.5
SPEED = 0.3


def main():
    robot = Robot()
    try:
        while True:
            sectors = lidar_sectors(max_scans=1)
            front = sectors.get("front")
            left = sectors.get("left")
            right = sectors.get("right")
            print(sectors)
            if front is None or front > STOP_DISTANCE:
                robot.forward(SPEED)
            else:
                robot.stop()
                time.sleep(0.2)
                if (left or 0) > (right or 0):
                    robot.left_turn(SPEED)
                else:
                    robot.right_turn(SPEED)
                time.sleep(0.4)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
