# ROS2

## Dependencies

Follow the ROS2 [installation
instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
To build the project at minimum the packages `ros-humble-ros-base` and
`ros-dev-tools` are required.

To run the project with visualization of the coordinate frames
`ros-humble-rviz2` is required.

A summary of the installation process is as follows:

```bash
# Install dependencies and enable required universe repository
~ $ sudo apt install software-properties-common
~ $ sudo add-apt-repository universe
~ $ sudo apt update && sudo apt install curl -y

# Add ROS2 to repository sources
~ $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
~ $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Upgrade all packages before installing ROS2
~ $ sudo apt update
~ $ sudo apt upgrade

# At minimum the following are required
~ $ sudo apt install ros-humble-ros-base
~ $ sudo apt install ros-humble-rviz2

# For ease the following can be installed rather than the above two.
# This package includes tutorials and demos as well as ros-base and rviz2
~ $ sudo apt install ros-humble-desktop
```

## Build and Run

To build in run this project run the following commands:

```bash
# Get the project and move to the root of the ros workspace
~ $ git clone https://github.com/devon-g/robot-kinematics
~ $ cd robot-kinematics/ros2/

# Build the project and source the local install
~/robot-kinematics/ros2/ $ colcon build
~/robot-kinematics/ros2/ $ source install/setup.bash

# Launch the ros system
~/robot-kinematics/ros2/ $ ros2 launch wx200_control control.launch.py
```
