from machine import Pin, PWM
import time
import math


class Motor:
    def __init__(self, pwm, la, lb, adir, bdir, pwm_freq=5000):
        self._la = Pin(la, machine.Pin.IN, machine.Pin.PULL_UP)
        self._lb = Pin(lb, machine.Pin.IN, machine.Pin.PULL_UP)
        self._ladir = Pin(adir, machine.Pin.OUT, machine.Pin.PULL_UP)
        self._lbdir = Pin(bdir, machine.Pin.OUT, machine.Pin.PULL_UP)
        self._pwm_pin = Pin(pwm, machine.Pin.OUT, machine.Pin.PULL_UP)
        self._pwm = machine.PWM(self._pwm_pin)
        self._pwm.freq(pwm_freq)

    def drive(self, pwm_val):
        if pwm_val < 0:
            self._ladir.value(1)
            self._lbdir.value(0)
        else:
            self._ladir.value(0)
            self._lbdir.value(1)
        self._pwm.duty_u16(int(math.fabs(pwm_val)))
        

class Drivetrain:
    def __init__(self, motors):
        self._motors = motors

    def drive(self, pwm_vals):
        for pwm, motor in zip(pwm_vals, self._motors):
            motor.drive(pwm)
    
    def stop(self):
        for motor in self._motors:
            motor.drive(0)

    def time_drive(self, pwm_vals, delay):
        self.drive(pwm_vals)
        time.sleep(delay)
        self.stop()

    def num_motors(self):
        return len(self._motors)
