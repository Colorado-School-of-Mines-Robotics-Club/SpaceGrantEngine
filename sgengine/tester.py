import time
import subprocess

command_list = ["ros2", "launch", "launch/auton_control.launch.py"]

auton_process = subprocess.Popen(command_list)
time.sleep(10)
auton_process.terminate()