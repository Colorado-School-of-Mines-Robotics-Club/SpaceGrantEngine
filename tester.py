import time
import subprocess

command_1 = 'bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch launch/auton_control.launch.py" '

auton_process = subprocess.Popen(command_1, shell=True)
time.sleep(20)

auton_process.terminate()