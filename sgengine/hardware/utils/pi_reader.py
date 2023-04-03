import machine
from machine import UART
from machine import Pin as pin
from collections import deque

class PiReader:

    def __receiveInstruction(self, __InstructionRequest):
        if self.__comm.any():
            msg = self.__comm.read().decode('utf-8')
            msg = msg.split(',')
            self.__speed = int(msg[0])
            self.__direction = msg[1]
            self.__readable = True         

    def __init__(self, IRQ_Pin=19, tx_pin=20, rx_pin=21, uart_bus=1):
        '''Creates a new uart communication line to read data from the Pi into a buffer. The IRQ_Pin is used to alert the pico that a message has been transferred.'''
        # UART
        self.__comm = UART(uart_bus, 9600, tx=pin(tx_pin), rx=pin(rx_pin))
        self.__comm.init(9600, bits=8, parity=None, stop=1, timeout=1)
        # clear garbage
        if self.__comm.any():
            self.__comm.read()
        # IRQ
        self.__InstructionRequest = pin(IRQ_Pin, pin.IN)
        self.__InstructionRequest.irq(handler=self.__receiveInstruction, trigger=pin.IRQ_FALLING)
        # data
        self.__speed = 0
        self.__direction = 'forward'
        self.__readable = False

    def speed(self):
        '''get the current speed instruction'''
        return self.__speed
    
    def direction(self):
        '''get the current direction instruction'''
        return self.__direction
    
    def any(self):
        '''returns true if there is a new readable instruction issued'''
        return self.__readable

    def read(self):
        '''If a new instruction has been issued, returns a tuple with the speed and direction'''
        if self.__readable:
            self.__readable = False
            return self.__speed, self.__direction
        else:
            return None
