# edu_simulation
Contains Gazebo simulation models for all EduArt's robots.

## Installation

### Native
If you want to run the simulation on your computer follow these steps.
=======

### Native
If you want to run the simulation on your computer follow these steps.

The simulation environment used for this package is gazebo

```bash
sudo apt install ros-humble-gazebo-plugins
```

For cloning this package you need "git":
```bash
sudo apt install git
```

Clone the package in your src-folder:
```bash
cd ~/<your ros2 workspace>/src
git clone https://github.com/EduArt-Robotik/edu_simulation.git
```

You need the edu_robot package. Currently you have to use the "develop"-branch. Check out the branch:
```bash
cd <your ros2 workspace>/src/edu_robot
git checkout develop
```

Build edu_robot
```bash
cd <your ros2 workspace>
colcon build --packages-select edu_robot --event-handlers console_direct+ --symlink-install
```

Build edu_simulation
```bash
cd <your ros2 workspace>
colcon build --packages-select edu_simulation --event-handlers console_direct+ --symlink-install
```

## Launching Simulator

After the package was built Gazebo it will be launched using a provided ROS launch file. This launch file adds all content coming with this package.

Source your ROS2 environment if not already done;

```bash
source ~/<your ros2 workspace>/install/setup.bash
```

Now you can launch Gazebo using following launch file:

```bash
ros2 launch edu_simulation eduard.launch.py
```

>**Note**: The first launch takes the same minutes. The next launches will take much less time.

## Models

[link](documentation/models.md)

## Launching Simulator

After the package was built Gazebo it will be launched using a provided ROS launch file. This launch file adds all content coming with this package.

Source your ROS2 environment if not already done;

```bash
source ~/<your ros2 workspace>/install/setup.bash
```

Now you can launch Gazebo using following launch file:

```bash
ros2 launch edu_simulation eduard.launch.py
```

>**Note**: The first launch takes the same minutes. The next launches will take much less time.

## Models

[link](documentation/models.md)

## World

## Usage

### Robots
