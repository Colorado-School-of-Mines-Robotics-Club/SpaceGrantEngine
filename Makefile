.PHONY: all main help clean install check ci test source messages fast launch

main: install source

all: main messages fast

help:
	@echo "all - Cleans, installs, and re-sources all files for ROS. Default"
	@echo "check - Checks the continous integration and tests"
	@echo "clean - Clears old build files and logs"
	@echo "install - Runs the ROS2 build system to install the project"
	@echo "ci - Runs the continous integration locally, outputs errors"
	@echo "test - Runs the ROS2 test suite with pytest"
	@echo "source - Sources all ROS2 files and packages"
	@echo "messages - Builds the sgengine_messages package which defines custom message types"

check: ci test

clean:
	rm -rf build
	rm -rf log

install: 
	colcon build --symlink-install

ci:
	python3 -m black ./sgengine/sgengine
	python3 -m pylint ./sgengine/sgengine --ignore-paths=sgengine/sgengine/hardware/utils --ignore-paths=sgengine/sgengine/aruco --disable=C0303,C0114,W0401,R0902,R0904,C0305,W0703,C0200,R0913,R0914,W0107,W1203,W0246,W0511
	python3 -m flake8 --exclude=sgengine/sgengine/hardware/utils --exclude=sgengine/sgengine/aruco ./sgengine/sgengine --count --select=E9,F63,F7,F82,F401,W292,E275,F403 --extend-ignore=W293,W291,E303,E203,W503 --show-source --statistics
	python3 -m flake8 --exclude=sgengine/sgengine/hardware/utils --exclude=sgengine/sgengine/aruco ./sgengine/sgengine --count --exit-zero --max-complexity=10 --max-line-length=140 --statistics
	@echo "DONE - CI PASSED"

test:
	colcon test --event-handlers console_direct+

source:
	$(./source.sh)

messages:
	$(MAKE) -C sgengine_messages

fast: 
	$(MAKE) -C sgengine_fast

launch:
	ros2 launch launch.xml
