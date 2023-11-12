.PHONY: all check clean test fmt help

SHELL=/bin/bash

all:
	source /opt/ros/humble/setup.bash
	colcon build --symlink-install

clean:
	rm -rf ./install
	rm -rf ./build
	rm -rf ./log

check: check_fmt test

test:
	source /opt/ros/humble/setup.bash
	source ./install/setup.bash
	colcon test --event-handlers console_direct+

fmt:
	source /opt/ros/humble/setup.bash
	source install/setup.bash
	python3 -m black ./sgengine/sgengine
	python3 -m isort ./sgengine/sgengine
	python3 -m ruff --fix ./sgengine/sgengine
	@echo "FORMATTING COMPLETE"

check_fmt:
	source /opt/ros/humble/setup.bash
	source install/setup.bash
	python3 -m black --check ./sgengine/sgengine
	python3 -m isort --check ./sgengine/sgengine
	python3 -m ruff ./sgengine/sgengine
	@echo "FORMATTING CHECK PASSED"

help:
	@echo "all - Builds all ros packages (Default)"
	@echo "clean - Clears the workspace (build, install, and log dirs)"
	@echo "check - Runs test & check_fmt"
	@echo "test - Runs the ROS2 test suite on packages"
	@echo "fmt - Runs formatting corrections & tests, outputs problems to fix"
	@echo "check_fmt - Runs formatting tests, outputs problems to fix. Used in CI"
