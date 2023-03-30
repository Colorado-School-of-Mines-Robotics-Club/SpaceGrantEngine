from machine import Pin, PWM
import time
import math
# Back Left
BLA = Pin(28, machine.Pin.IN, machine.Pin.PULL_UP)
BLB = Pin(22, machine.Pin.IN, machine.Pin.PULL_UP)
BLADIR = Pin(6, machine.Pin.OUT, machine.Pin.PULL_UP)
BLBDIR = Pin(7, machine.Pin.OUT, machine.Pin.PULL_UP)
BLpwm = Pin(8, machine.Pin.OUT, machine.Pin.PULL_UP)
BLPWM = machine.PWM(BLpwm)
BLPWM.freq(5000)
#Front Left
FLA = Pin(0, machine.Pin.IN, machine.Pin.PULL_UP)
FLB = Pin(1, machine.Pin.IN, machine.Pin.PULL_UP)
FLADIR = Pin(4, machine.Pin.OUT, machine.Pin.PULL_UP)
FLBDIR = Pin(3, machine.Pin.OUT, machine.Pin.PULL_UP)
FLpwm = Pin(2, machine.Pin.OUT, machine.Pin.PULL_UP)
FLPWM = machine.PWM(FLpwm)
FLPWM.freq(5000)
LeftInst = [65000,-65000,-65000]
RightInst = [-65000,65000,-65000]
# for i in range(len(LeftInst)):
#     if LeftInst[i] < 0:
#         dirLB = 0
#         dirLA = 1
#     else:
#         dirLB = 1
#         dirLA = 0
#
#     if RightInst[i] < 0:
#         dirRB = 0
#         dirRA = 1
#     else:
#         dirRB = 1
#         dirRA = 0
#
#     BLADIR.value(dirLA)
#     BLBDIR.value(dirLB)
#     FLADIR.value(dirRA)
#     FLBDIR.value(dirRB)
#     BLPWM.duty_u16(int(math.fabs(LeftInst[i])))
#     FLPWM.duty_u16(int(math.fabs(RightInst[i])))
#
#     time.sleep(10)
pwmVal = 65000
counter = 20000
dirLB = 0
dirLA = 1
dirRB = 1
dirRA = 0
BLADIR.value(dirLA)
BLBDIR.value(dirLB)
FLADIR.value(dirRA)
FLBDIR.value(dirRB)
FLPWM.duty_u16(int(math.fabs(pwmVal)))
const = 5.5 / 10.5
while True:
    BLPWM.duty_u16(int(math.fabs(pwmVal) * const))
    counter = counter + 1
    if counter > pwmVal:
        break
BLPWM.duty_u16(0)
FLPWM.duty_u16(0)
