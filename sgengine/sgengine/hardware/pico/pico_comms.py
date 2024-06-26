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

    def __init__(self, serial_port="/dev/ttyACM0", enable_pin=21, baud=115200):
        """Creates a cummunication line to send instructions to the pi pico"""
        SG_Logger.__init__(self)

        # UART
        self._serial_line = serial.Serial(
            port=serial_port,
            baudrate=baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5,
            write_timeout=0.5,
        )
        # enable pin
        self._enable_pin = enable_pin
        self._interrupt_pin = enable_pin
        GPIO.setup(enable_pin, GPIO.OUT)
        GPIO.output(enable_pin, GPIO.HIGH)

    def send_move_command(self, left: int, right: int):
        """sends an instruction consisting of a left and right motor speed to pico"""
        self.send_str_direct(str(left) + "," + str(right) + "\0")

    def send_str_direct(self, msg: str) -> None:
        """Directly sends string message to pico"""
        encoded = msg.encode()
        logging.debug(f"PicoComms sending {encoded}")
        try:
            self._serial_line.write(encoded)
        except serial.SerialTimeoutException:
            logging.warn("Serial timeout!")
