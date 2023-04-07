from pi_reader import PiReader
import time
from motors import Motor, Drivetrain

def parse_data_to_pwm(data):
    angular, linear = data
    


motorB = Motor(pwm=11, la=28, lb=22, adir=12, bdir=13, pwm_freq=5000, max_pwm=65535, reverse=False)
motorA = Motor(pwm=18, la=0, lb=1, adir=19, bdir=20, pwm_freq=5000, max_pwm=65535, reverse=False)

drivetrain = Drivetrain([motorA, motorB], max_pwm=65535)

pi_reader = PiReader()

while True:
    
    if pi_reader.any():
        print("Received data")
        data = pi_reader.read()
        print(data)

        # drive the drivetrain for the speed value as pwm and direction parsed to multiple
        pwms = []
        for i in range(drivetrain.num_motors()):
            pwms.append(data[0])

        if data[1] == "backward":
            for pwm in pwms:
                pwm = -1 * pwm
        
        drivetrain.drive(pwms)
        
    time.sleep(0.5)
