# SpaceGrantEngine
'Engine' for handling concurrency and other 'low-level' tasks 

See [ROS Humble](https://docs.ros.org/en/humble/index.html) documentation for details on how to use ros2.  

## Style Guide
Python code should follow snake case, with proper public/private seperation in class development.
Public/private in Python is done with an underscore at the front of an attribute. For example,

`self._example: str = "example"` is private

`self.example: str = "example"` is public

All Python code should also be type hinted. Type hints allow static checkers to analyze the 
codebase for any errors before runtime. This is done during the `make ci` command. Integrating
MyPy would enhance the static type checking further, but it is not done yet. Could be an item 
for someone to work on. (See the other workflow files in the .github folder)

## Using the Makefile
`make help` Displays all possible commands for utilizing the Makefile.

The most used commands for typical work will be 

`make` or `make engine` builds sgengine packages (they are equivalent)

`make all` builds all packages, including depthai-ros (this uses a large amount of ram and may crash your computer)

`make test_engine` runs the ROS tests for sgengine packages which are known to fail, for now

`make ci` Will run the continous integration and let you know if there are any linting errors.
These errors should be fixed before a pull request is opened to main, but are fine for the 
testing and development phase.

## Typical Development Workflow
1. Create a new branch
2. Switch to using the new branch as the active one
3. Create a subfolder with an init file
4. Develop a class to add the given functionality OR a set of functions for the funcationality
5. Create a test file which uses fake input to test the code
6. Add the functionality to an existing node OR create a new node
7. Open a pull request :)

## Resposibility of Modules
1. configuration: Reads some config file and either generates a dictionary or uses ROS param services. TBD
2. gui: Launches a basic web app for starting and stopping the robots autonomous functions, as well as allowing remote control. If progress makes it far enough this will also allow "on-the-fly" adjustments of the Raspberry Pi's GPIO pins. 
3. hardware: This will contain code for handling all physical elements we will interact with. This includes RPi.GPIO, OAK-D cameras, RPi Picos, etc..
4. map: This will handle aggregating data into a 2D map for pathfinding and/or planning to operate on.
5. object_detection: The most open ended module, find objects from sensor data! Possible pathways include computer vision, machine learning, inferring on 3D maps, etc...
6. odometry: This computes the robots current position in space.
7. pathfinding: This module revolves around pathfinding algorithms. Classic examples are A* and Breadth-First-Search. These will all operate on some 2D plane.
8. planning: This module contains tools for deciding how to move a robot given all other information. This could simply be algorithms for adjusting a map form object detections or something much more complex.
9. utils: These contain generic utilities which are not explicity tied to a single module.

## Common Github Commands
`git branch {branchname}` Creates a new local branch for development

`git checkout {branchname}` Switches your current branch to the given branch

`git add {files}` Adds files to git for tracking or adds updates

`git commit -m {message}` Makes a commit with the given files added with a given message

`git push` Pushes any local commits to the remote repository (Github) on your working branch

`git push --set-upstream origin {branchname}` Pushes changes to the given branchname remotely, if the branch was created locally

`git reset {--hard}` Resets back to the latest pushed changes in the remote repository (Github), DO NOT use if you have uncommited work.


## Building and Sourcing Package

`make`

`source install/setup.bash`

Then you can call anything defined in the package

`ros2 run sgengine test`

## Creating a new node

In the "setup.py" file at the bottom within the "setup" function there is an argument called entry_points. 

Inside of entry_points there is a dictionary with a key entry called console_scripts. 

Add an entry to console_scripts for each node you want to add. This allows you to directly call the node 
from the CLI or via a launch file. 


## Running the OAK-D S2 camera

Ensure the camera is plugged in

Navigate to the depthai-ws directory

If there is no dpethai-ros folder in src, then run `git submodule init`, and then `git submodule clone` (or equivalent command)

Execute the `./install_depthai_ros.sh` script

Whenever you open a new terminal make sure to source the setup.bash file
`source depthai-ws/install/setup.bash`

Execute: `ros2 launch depthai_examples stereo_intertial_node.launch.py extended:=True monoResolution:=400p`  

## Launching the code automatically

To copy launch service to proper directory run:  
Run `cp /home/pi/SpaceGrantEngine/scripts/engine_launch.service /etc/systemd/system`  
then `systemctl enable engine_launch.service` to enable the service on boot.  
