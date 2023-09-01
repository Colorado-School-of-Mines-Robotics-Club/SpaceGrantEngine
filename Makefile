.PHONY: engine all check clean test ci help

all:
	colcon build --symlink-install --packages-skip python-steamcontroller

clean:
	sudo rm -rf ./install
	sudo rm -rf ./build
	sudo rm -rf ./log

check: ci test

test:
	colcon test --event-handlers console_direct+ --packages-skip python-steamcontroller

ci:
	bash -c "source ./install/setup.bash && python3 -m black ./sgengine/sgengine && python3 -m pylint ./sgengine/sgengine --rcfile=.pylint && python3 -m flake8 ./sgengine/sgengine --config .flake8"
	@echo "DONE - CI PASSED"

help:
	@echo "all - Builds all ros packages (Default)"
	@echo "clean - Clears the workspace (build, install, and log dirs)"
	@echo "check - Runs test & ci"
	@echo "test_engine - Runs the ROS2 test suite on sgengine packages"
	@echo "ci - Runs the continous integration locally, outputs errors"
