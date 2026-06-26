SHELL := /bin/bash
WS     := $(HOME)/git/ros2_ws
PKG    := nav2_teb_controller
SRC    := src

.PHONY: help build test format format-fix lint lint-fix all

help:
	@echo "Available commands:"
	@echo "  make build       		- colcon build"
	@echo "  make test        		- colcon test"
	@echo "  make test-with-log		- colcon test"
	@echo "  make format      		- clang-format check (kein Fix)"
	@echo "  make format-fix  		- clang-format mit Fix"
	@echo "  make lint        		- clang-tidy check (kein Fix)"
	@echo "  make lint-fix    		- clang-tidy mit Fix"
	@echo "  make all         		- format + lint + build + test"

build:
	source /opt/ros/jazzy/setup.bash && \
	cd $(WS) && colcon build \
		--packages-select $(PKG) \
		--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

build-clean:
	source /opt/ros/jazzy/setup.bash && \
	cd $(WS) && colcon build \
		--packages-select $(PKG) \
		--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		--cmake-clean-first

test-with-log:
	source /opt/ros/jazzy/setup.bash && \
	cd $(WS) && colcon test --packages-select $(PKG) --event-handlers console_direct+ && \
	colcon test-result

test:
	source /opt/ros/jazzy/setup.bash && \
	cd $(WS) && colcon test --packages-select $(PKG) && \
	colcon test-result

format:
	find $(SRC) -name "*.cpp" -o -name "*.hpp" | \
	xargs clang-format --dry-run --Werror --style=file

format-fix:
	find $(SRC) -name "*.cpp" -o -name "*.hpp" | \
	xargs clang-format -i --style=file

lint:
	run-clang-tidy \
		-p $(WS)/build/$(PKG) \
		-config-file .clang-tidy \
		-header-filter=".*nav2_teb_controller/(g2o_types|core|obstacles|homotopy|planner).*" \
		$(shell find $(SRC) -name "*.cpp")

lint-fix:
	run-clang-tidy \
		-p $(WS)/build/$(PKG) \
		-config-file .clang-tidy \
		-header-filter=".*nav2_teb_controller/(g2o_types|core|obstacles|homotopy|planner).*" \
		-fix \
		$(shell find $(SRC) -name "*.cpp")
		--fix-errors

all: format lint build test