from machine import UART
from machine import Pin as pin


class PiReader:
    def __receiveInstruction(self, __InstructionRequest):
        try:
            if self.__comm.any():
                msg = self.__comm.read().decode("utf-8")
                msg = msg.split(",")
                self.__angular = float(msg[0])
                self.__linear = float(msg[1])
                self.__readable = True
        except ValueError:
            self.__angular = 0.0
            self.__linear = 0.0
            self.__readable = False
            print("msg could not be parsed")

    def __init__(self, IRQ_Pin=19, tx_pin=20, rx_pin=21, uart_bus=1):
        """Creates a new uart communication line to read data from the Pi into a buffer.
        The IRQ_Pin is used to alert the pico that a message has been transferred."""
        # UART
        self.__comm = UART(uart_bus, 112500, tx=pin(tx_pin), rx=pin(rx_pin))
        self.__comm.init(112500, bits=8, parity=None, stop=1, timeout=1)
        # clear garbage
        if self.__comm.any():
            self.__comm.read()
        # IRQ
        self.__InstructionRequest = pin(IRQ_Pin, pin.IN)
        self.__InstructionRequest.irq(
            handler=self.__receiveInstruction, trigger=pin.IRQ_FALLING
        )
        # data
        self.__angular = 0.0
        self.__linear = 0.0
        self.__readable = False

    def angular(self):
        """get the current angular instruction"""
        return self.__angular

    def linear(self):
        """get the current linear instruction"""
        return self.__linear

    def any(self):
        """returns true if there is a new readable instruction issued"""
        return self.__readable

    def read(self):
        """If a new instruction has been issued, returns a tuple with the speed and direction"""
        if self.__readable:
            self.__readable = False
            return self.__angular, self.__linear
        else:
            return None
