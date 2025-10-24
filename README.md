# monorepo-test

### Install Gazebo
```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

### Enabling Real Time ros2_control
```bash
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Afterwards, add the following limits to the realtime group in `/etc/security/limits.conf`:
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
```

### Moveit Installs
Install rviz moveit package:
```bash
sudo apt install ros-humble-moveit-ros-visualization
```

Download source code for the rest of MoveIt
```bash
cd src
vcs import < moveit2_tutorials/moveit2_tutorials.repos
```

Rosdep install
```bash
cd ..
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

