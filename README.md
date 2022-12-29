# Robotic_HW_Solution
Solution to robotic_hw by Ryan Sandzimier

Developed on ROS2 Foxy

## Usage

Create ROS2 workspace and clone repository
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rsandzimier/robotic_hw_solution.git
```
Build package
```
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```
Run sensor simulator
```
python3 src/robotic_hw_solution/sensor.py
```
Run ROS2 nodes. In separate terminal/tab:
```
cd ~/ros2_ws
source install/setup.bash
ros2 launch robotic_hw_solution sensor_launch.py
```
