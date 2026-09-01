# meca500_ros2
ROS 2 Controllers For the Mecademic meca500 robot

# Requirements
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
* rosdep

# Installation
1. Create workspace

```bash
mkdir -p ~/workspaces/meca_ws/src
```

2. Download code

```bash
cd ~/workspaces/meca_ws/src
git clone https://github.com/Vanderbilt-Applied-Robotics-Lab/meca500_ros2.git
```

3. Install dependencies

```bash
cd ~/workspaces/meca_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build code

```bash
cd ~/workspaces/meca_ws
colcon build
```

# Run Controller

1. Source code

```bash
cd ~/workspaces/meca_ws
source install/setup.bash
```

2. Start controller

For real hardware:

```bash
ros2 launch meca500_bringup moveit.launch.py
```

For simulated hardware:

```bash
ros2 launch meca500_bringup moveit.py simulation:=true
```


