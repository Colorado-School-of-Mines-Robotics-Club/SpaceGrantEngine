from pi_reader import PiReader
import time
import math
from motors import Motor, Drivetrain


MAX_PWM = 65535
TURN_CUTOFF = 0.5
TURN_NOISE = 0.2
IS_LEFT = True
REVERSE_MOTOR_1 = False
REVERSE_MOTOR_2 = False

def parse_data_to_pwm(data):
    angular, linear = data
    # have a binary decision about whether we should oppose the wheels
    # this is where we have the opposing movement
    # """
    # All wheels get driven at the same absolute pwm
    # ------    ------
    # |    |    |    |
    # |    |    |    |
    # | -> |    | -> |
    # |    |    |    |
    # |    |    |    |
    # ------    ------
    # ------    ------
    # |    |    |    |
    # |    |    |    |
    # | <- |    | <- |
    # |    |    |    |
    # |    |    |    |
    # ------    ------
    # """
    if math.fabs(angular) > TURN_CUTOFF:
        pwm = MAX_PWM * angular
        if angular > 0 and IS_LEFT:
            pwms = [pwm, pwm]
        elif angular > 0 and not IS_LEFT:
            pwms = [-pwm, -pwm]
        elif angular < 0 and IS_LEFT:
            pwms = [-pwm, -pwm]
        else:
            pwms = [pwm, pwm]
        return pwms
    # """
    # ------    ------
    # |    |    |    |
    # |    |    |    |
    # | -> |    | <- | 
    # |    |    |    |
    # |    |    |    |
    # ------    ------
    # ------    ------
    # |    |    |    |
    # |    |    |    |
    # | <- |    | -> |
    # |    |    |    |
    # |    |    |    |
    # ------    ------
    # """
    elif math.fabs(angular) > TURN_NOISE:
        pwm = MAX_PWM * ((linear + angular) / 2)
        if angular > 0 and IS_LEFT:
            pwms = [pwm, pwm]
        elif angular > 0 and not IS_LEFT:
            pwms = [pwm * angular, pwm * angular]
        elif angular < 0 and IS_LEFT:
            pwms = [pwm * angular, pwm * angular]
        else:
           pass
        return pwms
    # """
    # ------    ------
    # |    |    |    |
    # |    |    |    |
    # | -> |    | <- |
    # |    |    |    |
    # |    |    |    |
    # ------    ------
    # ------    ------
    # |    |    |    |
    # |    |    |    |
    # | <- |    | -> |
    # |    |    |    |
    # |    |    |    |
    # ------    ------
    # """
    else:
        pwm = MAX_PWM * linear
        pwms = [pwm, pwm]
        return pwms


motorB = Motor(pwm=11, la=28, lb=22, adir=12, bdir=13, pwm_freq=5000, max_pwm=65535, reverse=REVERSE_MOTOR_1)
motorA = Motor(pwm=18, la=0, lb=1, adir=19, bdir=20, pwm_freq=5000, max_pwm=65535, reverse=REVERSE_MOTOR_2)

DRIVETRAIN = Drivetrain([motorA, motorB], max_pwm=65535)

PI_READER = PiReader()

while True:
    
    if PI_READER.any():
        print("Received data")
        data = PI_READER.read()
        print(data)

        # drive the drivetrain for the speed value as pwm and direction parsed to multiple
        pwms = parse_data_to_pwm(data)
        DRIVETRAIN.drive(pwms)
        
    time.sleep(0.5)
