import time
import sys

from ..pico.pico_comms import PicoComms
from steamcontroller import SteamController


PICO = PicoComms()
MAX = 32767
LAST_PACKET = -1
DELAY = 0.25


def joystick(_, sci):
    global LAST_PACKET
    global DELAY
    if time.perf_counter() - LAST_PACKET < DELAY:
        return
    LAST_PACKET = time.perf_counter()

    x = sci.lpad_x
    y = sci.lpad_y

    x = x / MAX
    y = y / MAX

    x = max(min(1.0, x), -1.0)
    y = max(min(1.0, y), -1.0)

    print(f"{x},{y}")

    PICO.send_move_command(x, y)


def main():
    try:
        sc = SteamController(callback=joystick)
        sc.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        sys.stderr.write(str(e) + "\n")
        sys.exit(-1)


if __name__ == "__main__":
    main()
