# pylint: skip-file

import sys
import time

try:
    import serial
    from RPi import GPIO

    GPIO.setmode(GPIO.BCM)
except ModuleNotFoundError:
    print("WARNING: Could not load serial or RPi library, commands will not be sent")


class PicoComms:
    def __init__(self, serialPort="/dev/ttyS0", interruptPin=5, baud=112500):
        """Creates a cummunication line to send instructions to the pi pico.
        The interrupt pin is the pin that will be used to tell the pico that it has received an instruction
        """
        if "serial" not in sys.modules:
            print("WARNING: PicoComms running in dummy mode")
            return
        # UART
        self.__serialLine = serial.Serial(
            port=serialPort,
            baudrate=baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
        )
        # interrupt request pin
        self.__interruptPin = interruptPin
        GPIO.setup(interruptPin, GPIO.OUT)
        GPIO.output(interruptPin, 0)

    def send_move_command(self, angular: float, linear: float):
        """sends an instruction consisting of speed and direction to the pi pico"""
        # format
        instruction = str(angular) + "," + str(linear)
        encoded = instruction.encode()
        if "serial" not in sys.modules:
            print(f"PicoComms could not send {encoded}")
            return
        print(f"PicoComms sending {encoded}")
        # send
        self.__serialLine.write(encoded)
        # raise IRQ
        GPIO.output(self.__interruptPin, 1)
        time.sleep(0.001)
        GPIO.output(self.__interruptPin, 0)

    def send_str_direct(self, msg: str) -> None:
        """directly sends string message to pico"""
        encoded = msg.encode()
        if "serial" not in sys.modules:
            print(f"PicoComms sends {encoded}")
            return
        # send
        self.__serialLine.write(encoded)
        # raise IRQ
        GPIO.output(self.__interruptPin, 1)
        time.sleep(0.001)
        GPIO.output(self.__interruptPin, 0)
