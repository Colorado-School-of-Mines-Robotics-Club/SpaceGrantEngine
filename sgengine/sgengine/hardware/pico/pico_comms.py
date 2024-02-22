import logging
import sys
import time

import serial

from ...sg_logger import SG_Logger

try:
    from RPi import GPIO

    GPIO.setmode(GPIO.BCM)
except RuntimeError as e:

    # left as non logger print statements since logger is not fully setup at this point
    print(f'Could not load RPi library with error "{e}"', file=sys.stderr)
    print("Commands will not be sent!", file=sys.stderr)


class PicoComms(SG_Logger):
    """Class to manage serial communication with Picos"""

    def __init__(self, serial_port="/dev/ttyS0", interrupt_pin=5, baud=38400):
        """Creates a cummunication line to send instructions to the pi pico.
        The interrupt pin is the pin that will be used to tell the pico that it has received an instruction
        """

        SG_Logger.__init__(self)

        self._no_comms = False
        if "serial" not in sys.modules:
            self._no_comms = True
            logging.info('PicoComms running in dummy mode')
            return
        # UART
        try:
            self._serial_line = serial.Serial(
                port=serial_port,
                baudrate=baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1,
            )
        except serial.serialutil.SerialException as e:
            self._no_comms = True
            logging.error(f'Unable to open serial device with error "{e}"')
            logging.error('Commands will not be sent!')
            return
        # interrupt request pin
        self._interrupt_pin = interrupt_pin
        GPIO.setup(interrupt_pin, GPIO.OUT)
        GPIO.output(interrupt_pin, 0)

    def send_move_command(self, left: int, right: int):
        """sends an instruction consisting of a left and right motor speed to pico"""
        # format
        self.send_str_direct(str(left) + "," + str(right) + "\0")

    def send_str_direct(self, msg: str) -> None:
        """Directly sends string message to pico"""
        encoded = msg.encode()
        if self._no_comms:
            logging.warning(f"PicoComms cannot send {encoded}")
            return
        logging.info(f"PicoComms sending {encoded}")
        # send
        self._serial_line.write(encoded)
        # raise IRQ
        GPIO.output(self._interrupt_pin, 1)
        time.sleep(0.001)
        GPIO.output(self._interrupt_pin, 0)
