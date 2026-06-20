import sys

sys.path.insert(0, "..")

from jbot import lidar_sectors


def main():
    sectors = lidar_sectors()
    for name, distance in sectors.items():
        text = "%.2f m" % distance if distance is not None else "n/a"
        print("%-6s %s" % (name, text))


if __name__ == "__main__":
    main()
