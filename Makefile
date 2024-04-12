.PHONY: all check clean test fmt help

SHELL=/bin/bash

all:
	source /opt/ros/humble/setup.bash
	colcon build --symlink-install
	@echo "BUILD COMPLETE"

clean:
	rm -rf ./install
	rm -rf ./build
	rm -rf ./log
	rm -rf ./tmp
	@echo "CLEAN COMPLETE"

check: check_fmt test
	@echo "CHECK COMPLETE"

test:
	source /opt/ros/humble/setup.bash
	source ./install/setup.bash
	colcon test --event-handlers console_direct+
	@echo "TESTING COMPLETE"

fmt:
	source /opt/ros/humble/setup.bash
	source install/setup.bash
	python3 -m ruff format ./sgengine/sgengine
	python3 -m ruff check --fix ./sgengine/sgengine
	cd sgengine_cpp && ament_clang_format --reformat
	@echo "FORMATTING COMPLETE"

check_fmt:
	source /opt/ros/humble/setup.bash
	source install/setup.bash
	python3 -m ruff format --diff ./sgengine/sgengine
	python3 -m ruff check ./sgengine/sgengine
	cd sgengine_cpp && ament_clang_format
	@echo "FORMATTING CHECK PASSED"

help:
	@echo "all - Builds all ros packages (Default)"
	@echo "clean - Clears the workspace (build, install, log, and tmp dirs)"
	@echo "check - Runs test & check_fmt"
	@echo "test - Runs the ROS2 test suite on packages"
	@echo "fmt - Runs formatting corrections & tests, outputs problems to fix"
	@echo "check_fmt - Runs formatting tests, outputs problems to fix. Used in CI"
