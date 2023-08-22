# Husky Robot

This repository contains code for simulating the Husky robot and performing various tasks such as navigation, mapping, and SLAM using ROS 2 Humbel and the Husky UAL package.

## Prerequisites

Make sure you have the following prerequisites installed:

### ROS 2 Humble Desktop

Follow the installation instructions for ROS 2 Humble Desktop: [Humble Desktop Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Required ROS 2 Packages

Install the necessary ROS 2 packages:

```bash
sudo apt-get install ros-humble-hardware-interface ros-humble-controller-manager ros-humble-gazebo-ros2-control ros-humble-xacro ros-humble-hardware-interface ros-humble-gazebo-plugins ros-humble-gazebo-msgs ros-humble-gazebo-ros ros-humble-gazebo-ros2-control-demos ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

## Getting Started

For Ubuntu 22.04, follow these steps:

```bash
cd /<your_workspace>/src
git clone -b humbel-devel https://github.com/khaledgabr77/TFG_Husky_UAL.git
cd ../..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Warehouse Model Setup

Clone the AWS RoboMaker Small Warehouse World repository from GitHub:

```bash
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
```

Copy the models and worlds folders from aws-robomaker-small-warehouse-world and add them to your TFG_Husky_UAL package directory:

```bash
cp -r aws-robomaker-small-warehouse-world/models <your_workspace>/src/TFG_Husky_UAL/husky_ual_description/
cp -r aws-robomaker-small-warehouse-world/worlds <your_workspace>/src/TFG_Husky_UAL/husky_ual_description/
```

Update your CMakeLists.txt file inside your install directory by adding the following folders:

```bash
    worlds
    models
```

Export the GAZEBO_MODEL_PATH:

```bash
export GAZEBO_MODEL_PATH=`pwd`/models
```

Launch the Husky robot and the warehouse environment:

```bash
ros2 launch husky_ual husky_simulation.launch.py
```

### Simulation

Compile the package and set up the environment:

```bash
colcon build 
source install/setup.bash
```

Launch the Husky robot and the warehouse environment:

```bash
ros2 launch husky_ual husky_simulation.launch.py
```

Drive the robot using teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

### Mapping

To perform SLAM with this package, ensure you have built the Slam-Toolbox from source. Note that the parameters of the official slam_toolbox are not finely tuned for ROS 2 Humble, so some modifications have been made to the parameters.

Launch the mapping process:

```bash
ros2 launch husky_ual husky_simulation.py
ros2 launch husky_ual online_async_launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

After mapping, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f <name_of_the_map>
```

You will find the map saved as both a `.png` and a `.yaml` file.

## Navigation