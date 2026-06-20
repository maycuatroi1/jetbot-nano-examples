import sys

sys.path.insert(0, "..")

from jbot import snapshot


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "frame.jpg"
    saved = snapshot(path)
    print("saved", saved)


if __name__ == "__main__":
    main()
