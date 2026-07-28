.PHONY: build test clean

build:
	colcon build --packages-select mavlink_nav_bridge --symlink-install

test:
	./build/mavlink_nav_bridge/test_drone_physics
##	colcon test --packages-select mavlink_nav_bridge && colcon test-result --verbose

clean:
	rm -rf install/ build/ log/
