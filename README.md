# meca500_ros2
ROS 2 Controllers For the Mecademic meca500 robot

# Requirements
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
* rosdep

# Installation
1. Create workspace

```bash
mkdir -p ~/workspace/meca_ws/src
```

2. Download code

```bash
cd ~/workspace/meca_ws/src
git clone https://github.com/Vanderbilt-Applied-Robotics-Lab/meca500_ros2.git
```

3. Install dependencies

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. Build code

```bash
cd ~/workspace/meca_ws
colcon build
```

# Run Controller

1. Source code

```bash
cd ~/workspace/meca_ws
source install/setup.bash
```

2. Start controller

```bash
ros2 launch meca500_bringup moveit.py
```
