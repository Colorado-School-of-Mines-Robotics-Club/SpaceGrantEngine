from pi_reader import PiReader
import time

pi_reader = PiReader()
while True:
    
    if pi_reader.any():
        print("Received data")
        data = pi_reader.read()
        print(data)
        
    time.sleep(0.5)
    