rm -r build/ install/
colcon build
source install/setup.bash
ros2 launch shesnar launch_sim.launch.py