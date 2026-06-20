import sys
import time

sys.path.insert(0, "..")

from jbot import Robot


def main():
    robot = Robot()
    sequence = [
        ("forward", lambda: robot.forward(0.3)),
        ("backward", lambda: robot.backward(0.3)),
        ("left", lambda: robot.left_turn(0.3)),
        ("right", lambda: robot.right_turn(0.3)),
    ]
    try:
        for name, move in sequence:
            print(name)
            move()
            time.sleep(0.6)
            robot.stop()
            time.sleep(0.4)
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
