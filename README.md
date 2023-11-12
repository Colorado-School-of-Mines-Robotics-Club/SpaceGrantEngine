# SpaceGrantEngine
'Engine' for handling concurrency and other 'low-level' tasks 

See [ROS Humble](https://docs.ros.org/en/humble/index.html) documentation for details on how to use ros2.  

## Submodules

This engine uses external dependencies that are located in the `extern` directory.

To directly clone the repository with these submodules add the `--recursive` flag to your `git clone` command.

To clone the submodules after cloning the engine, run `git submodule update --init --recursive`

## Building and Sourcing Package

`make`

`source install/setup.bash`

Then you can call anything defined in the package.

For example: `ros2 run sgengine pico`  

## Typical Development Workflow
1. Create a new branch
2. Add/modify desired functionality
3. Run `make check` to run tests
4. Open a pull request

## Style Guide
Python code should follow snake case, with proper public/private seperation in class development.
Public/private in Python is done with an underscore at the front of an attribute. For example,

`self._example: str = "example"` is private

`self.example: str = "example"` is public

## Using the Makefile
`make help` Displays all possible commands for utilizing the Makefile.

`make` Builds all ros packages

`make check` runs all tests (including ros and formatting tests). This must pass before your code can be merged into main.

## Raspberry Pi Setup
1. Flash SD card to "Ubuntu Desktop 22.04 LTS (64-bit)" and boot
2. Go through initial setup, set username to pi
3. Clone this repository in the `$HOME` directory with the `--recursive` flag
4. Run the `scripts/pi_software_install.py` script to set up the Pi
5. Run the `scripts/install_deps.py` script to configure this repositories' dependencies
6. Reboot
7. Clone the `https://github.com/atar-axis/xpadneo` repository and follow the instructions in the `Prerequisites` and `Installation` sections of the README
8.  Use `sudo usermod -a -G dialout pi` to give pi user serial access
9. Enable serial hardware in `raspi-config` in Interface Options->Serial Port. Second option must be set to <Yes>
10. Enable SSH in `raspi-config` in Interface Options->SSH
11. Reboot

## Launching the code automatically

To enable the launch service:

Make sure that the `SpaceGrantEngine` repository is located at `/home/pi/SpaceGrantEngine`

Run `systemctl enable $HOME/SpaceGrantEngine/scripts/engine_launch.service` to enable the service on boot.  
