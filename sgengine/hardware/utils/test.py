from pi_comms import Pico
from RPi import GPIO
import time

try:
    pico = Pico()

    while True:
        pico.send_instruction(10, "forward")
        print("forward")
        time.sleep(1)

        pico.send_instruction(10, "backward")
        print("backward")
        time.sleep(1)
except KeyboardInterrupt:
    pass
except:
    pass
finally:
    GPIO.cleanup()
