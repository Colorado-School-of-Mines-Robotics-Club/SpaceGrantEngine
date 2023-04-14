.PHONY: engine all check clean test ci help

engine: 
	colcon build --symlink-install --packages-select sgengine sgengine_messages

all:
	colcon build --symlink-install

clean:
	rm -rf ./install
	rm -rf ./build
	rm -rf ./log

check: ci test

test_engine:
	colcon test --event-handlers console_direct+ --packages-select sgengine sgengine_messages

ci:
	python3 -m black ./sgengine/sgengine
	python3 -m pylint ./sgengine/sgengine --ignore-paths=sgengine/sgengine/hardware/utils --ignore-paths=sgengine/sgengine/aruco --disable=C0303,C0114,W0401,R0902,R0904,C0305,W0703,C0200,R0913,R0914,W0107,W1203,W0246,W0511
	python3 -m flake8 --exclude=sgengine/sgengine/hardware/utils --exclude=sgengine/sgengine/aruco ./sgengine/sgengine --count --select=E9,F63,F7,F82,F401,W292,E275,F403 --extend-ignore=W293,W291,E303,E203,W503 --show-source --statistics
	python3 -m flake8 --exclude=sgengine/sgengine/hardware/utils --exclude=sgengine/sgengine/aruco ./sgengine/sgengine --count --exit-zero --max-complexity=10 --max-line-length=140 --extend-ignore=E203 --statistics
	@echo "DONE - CI PASSED"

help:
	@echo "engine - Builds all sgengine packages (Default)"
	@echo "all - Builds all packages (not just sgengine)"
	@echo "clean - Clears the workspace (build, install, and log dirs)"
	@echo "check - Runs test & ci"
	@echo "test_engine - Runs the ROS2 test suite on sgengine packages"
	@echo "ci - Runs the continous integration locally, outputs errors"
