# pylint: skip-file

import time
import serial
from RPi import GPIO

GPIO.setmode(GPIO.BCM)


class PicoComms:
    def __init__(self, serialPort="/dev/ttyS0", interruptPin=5, baud=9600):
        """Creates a cummunication line to send instructions to the pi pico.
        The interrupt pin is the pin that will be used to tell the pico that it has received an instruction
        """
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

    def send_instruction(self, speed=0, direction="forward"):
        """sends an instruction consisting of speed and direction to the pi pico"""
        # format
        instruction = str(speed) + "," + str(direction)
        # send
        self.__serialLine.write(instruction.encode())
        # raise IRQ
        GPIO.output(self.__interruptPin, 1)
        time.sleep(0.001)
        GPIO.output(self.__interruptPin, 0)
