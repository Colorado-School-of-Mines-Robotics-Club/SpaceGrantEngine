# pylint: skip-file

from ..pico.pico_comms import PicoComms

try:
    from RPi import GPIO
except ModuleNotFoundError:
    print("ERROR: Could not load RPi library!")
import time

BOTH = False

try:
    pico1 = PicoComms(interruptPin=5, baud=9600, port="/dev/ttyS0")
    if BOTH:
        pico2 = PicoComms(interruptPin=6, baud=9600, port="/dev/ttyAMA0")

    while True:
        pico1.send_move_command(65000, "forward")
        print("Pico1 sent, forward")
        if BOTH:
            pico2.send_move_command(65000, "forward")
            print("Pico2 sent, forward")
        time.sleep(2)

        pico1.send_move_command(65000, "backward")
        print("Pico1 sent, backward")
        if BOTH:
            pico2.send_move_command(65000, "backward")
            print("Pico2 sent, backward")
        time.sleep(2)
except KeyboardInterrupt:
    pass
except Exception as e:
    print(e)
finally:
    GPIO.cleanup()
