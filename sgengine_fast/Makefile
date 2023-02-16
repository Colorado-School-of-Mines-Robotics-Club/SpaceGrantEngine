.PHONY: all clean install test source

all: clean install source

clean:
	rm -rf build
	rm -rf log

install: 
	colcon build --symlink-install --packages-select sgengine_fast

test:
	colcon test --packages-select sgengine_fast --event-handlers console_direct+

source:
	cd ~/ros2_humble/install/ \
	source setup.bash 2> /dev/null | echo 
	cd /opt/ros/humble/ \
	source setup.bash 2> /dev/null | echo
