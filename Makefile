
.PHONY: all kill

SHELL := /bin/bash

all:
	colcon build --packages-select control
	source /opt/ros/humble/setup.bash ; source install/setup.bash ; ros2 launch control control.launch.py

kill:
	sudo pkill -f gzserver
	sudo pkill -f gz
	sudo pkill -f px4_sitl
	sudo pkill -f px4
	@if sudo ss -unlp | grep -q ':14540'; then \
		PID=$$(sudo ss -unlp | grep ':14540' | grep -oP 'pid=\K[0-9]+'); \
		echo "Порт 14540 занят процессом с PID $$PID. Завершаем..."; \
		sudo kill -9 $$PID; \
	else \
		echo "Порт 14540 свободен."; \
	fi