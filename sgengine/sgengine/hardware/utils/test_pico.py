from pi_reader import PiReader
import time
from motors import Motor, Drivetrain

# # Back Left
# BLA = Pin(28, machine.Pin.IN, machine.Pin.PULL_UP)
# BLB = Pin(22, machine.Pin.IN, machine.Pin.PULL_UP)
# BLADIR = Pin(13, machine.Pin.OUT, machine.Pin.PULL_UP)
# BLBDIR = Pin(12, machine.Pin.OUT, machine.Pin.PULL_UP)
# BLpwm = Pin(11, machine.Pin.OUT, machine.Pin.PULL_UP)
# BLPWM = machine.PWM(BLpwm)
# BLPWM.freq(5000)
motorB = Motor(pwm=8, la=28, lb=22, adir=7, bdir=6, pwm_freq=5000)

# #Front Left
# FLA = Pin(0, machine.Pin.IN, machine.Pin.PULL_UP)
# FLB = Pin(1, machine.Pin.IN, machine.Pin.PULL_UP)
# FLADIR = Pin(20, machine.Pin.OUT, machine.Pin.PULL_UP)
# FLBDIR = Pin(19, machine.Pin.OUT, machine.Pin.PULL_UP)
# FLpwm = Pin(18, machine.Pin.OUT, machine.Pin.PULL_UP)
# FLPWM = machine.PWM(FLpwm)
# FLPWM.freq(5000)
motorA = Motor(pwm=2, la=0, lb=1, adir=3, bdir=4, pwm_freq=5000)

drivetrain = Drivetrain([motorA, motorB])

pi_reader = PiReader()

MAX_PWM = 65500

while True:
    if pi_reader.any():
        print("Received data")
        data = pi_reader.read()
        print(data)

        # drive the drivetrain for the speed value as pwm and direction parsed to multiple
        pwms = [int(data[0] * MAX_PWM), int(data[1] * MAX_PWM)]

        # if data[1] == "backward":
        #     for pwm in pwms:
        #         pwm = -1 * pwm

        print(pwms)
        print()

        drivetrain.drive(pwms)
    time.sleep(0.001)

# LeftInst = [65536,-65536,-65536]
# RightInst = [-65536,65536,-65536]
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
# pwmVal = 65000
# counter = 20000
# dirLB = 0
# dirLA = 1
# dirRB = 1
# dirRA = 0
# BLADIR.value(dirLA)
# BLBDIR.value(dirLB)
# FLADIR.value(dirRA)
# FLBDIR.value(dirRB)
# FLPWM.duty_u16(int(math.fabs(pwmVal)))
# const = 5.5 / 10.5
# while True:
#     BLPWM.duty_u16(int(math.fabs(pwmVal) * const))
#     counter = counter + 1
#     if counter > pwmVal:
#         break
# BLPWM.duty_u16(0)
# FLPWM.duty_u16(0)
