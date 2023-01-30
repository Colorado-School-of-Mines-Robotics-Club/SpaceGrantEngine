.PHONY: all help clean install check ci test source messages launch

all: clean install source

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
	colcon build --symlink-install --packages-select sgengine

ci:
	python3 -m black ./sgengine
	python3 -m pylint ./sgengine --disable=C0303,C0114,W0401,R0902,R0904,C0305,W0703,C0200,R0913,R0914,W0107,W1203,W0246,W0511
	python3 -m flake8 ./sgengine --count --select=E9,F63,F7,F82,F401,W292,E275,F403 --extend-ignore=W293,W291,E303,E203,W503 --show-source --statistics
	python3 -m flake8 ./sgengine --count --exit-zero --max-complexity=10 --max-line-length=140 --statistics
	@echo "DONE - CI PASSED"

test:
	colcon test --packages-select sgengine sgengine_messages --event-handlers console_direct+

source:
	$(./source.sh)

messages:
	$(MAKE) -C sgengine_messages

launch:
	ros2 launch sgengine launch.xml
