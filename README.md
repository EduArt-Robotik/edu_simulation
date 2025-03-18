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
#If not already cloned clone edu_robot
cd ~/<your ros2 workspace>/src
git clone https://github.com/EduArt-Robotik/edu_robot.git

#Now change the branch!
cd <your ros2 workspace>/src/edu_robot
git checkout develop

#Check if the branch has been changed
git branch
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

## After the launch
After launching the simulator and placing a robot into the envrionment (just drag and drop the desired model from the menu on the left), you may notice that not much is happening in there. But before we can move around we first have to enable the motors:
```bash
ros2 service call /eduard/blue/set_mode edu_robot/srv/SetMode
" mode:
  mode: 2
  drive_kinematic: 0
  feature_mode: 0
  disable_feature: 0" 
```
By default the mode is 0, which tells us the motors are disabled. We set the mode to 2, to enable the motors.  
Now the time has come to send a little movement command to our robot. For this we use the following command:
```bash
ros2 topic pub /eduard/blue/cmd_vel geometry_msgs/msg/Twist
"linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10 -p 100
```
We set the linear x velocity to 0.1 and further add -r 10 and -p 100 at the end. The option -r 10 repeats the command 10 times per second and -p 100 just shows us 1 out of every 100 commands sent this way. The reason this is necessary lies in a safety mechanism integrated in the eduard robot, if it receives no movement command for 200ms (even if it tells it to not move at all) it will cease all movement.

## Models

[link](documentation/models.md)

## World

## Usage

### Robots
