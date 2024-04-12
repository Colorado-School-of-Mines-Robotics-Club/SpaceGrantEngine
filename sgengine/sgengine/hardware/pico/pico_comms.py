import logging
import sys

import serial

from ...sg_logger import SG_Logger

try:
    from RPi import GPIO

    GPIO.setmode(GPIO.BCM)
except RuntimeError as e:
    logging.error(f'Could not load RPi library with error "{e}"')
    sys.exit(1)


class PicoComms(SG_Logger):
    """Class to manage serial communication with Picos"""

    def __init__(self, serial_port="/dev/ttyACM0", enable_pin=21, baud=9600):
        """Creates a cummunication line to send instructions to the pi pico"""
        SG_Logger.__init__(self)

        # UART
        self._serial_line = serial.Serial(
            port=serial_port,
            baudrate=baud,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
        )
        # enable pin
        self._enable_pin = enable_pin
        self._interrupt_pin = enable_pin
        GPIO.setup(enable_pin, GPIO.OUT)
        GPIO.output(enable_pin, 0)

    def set_enable_pin(self, enabled: bool):
        if enabled:
            GPIO.output(self._enable_pin, 1)
        else:
            GPIO.output(self._enable_pin, 0)

    def send_move_command(self, left: int, right: int):
        """sends an instruction consisting of a left and right motor speed to pico"""
        self.send_str_direct(str(left) + "," + str(right) + "/")

    def send_str_direct(self, msg: str) -> None:
        """Directly sends string message to pico"""
        encoded = msg.encode()
        logging.debug(f"PicoComms sending {encoded}")
        GPIO.output(self._enable_pin, 1)
        self._serial_line.write(encoded)
