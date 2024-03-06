# edu_simulation
Contains Gazebo simulation models for all EduArt's robots.

## Installation

### Required Packages

First dependency is the ROS2 package **edu_robot**. This must be located in the same namespace.

```bash
sudo apt update
sudo apt install ros-humble-
sudo apt install ros-humble-gazebo-plugins
```

## Launching Simulator

After the package was built Gazebo it will be launched using a provided ROS launch file. This launch file adds all content coming with this package.

```bash
ros2 launch edu_simulation eduard.launch.py
```

>**Note**: The first launch takes the same minutes. The next launches will take much less time.

## Models

[link](documentation/models.md)

## World

## Problems

### Performance of the Gazebo Simulator
Although in reality the TER1 section can be filled with sand and / or gravel, in the scope of this work no simulation was performed with sand, neither with fixed sand (without real physical behavior) nor with moveable sand (with real physical behavior).
The reasons for this lie in the performance limitations of the Gazebo simulator, since based on experience it already reaches its limits when calculating the physical behavior of the robot on the track filled with only around about one hundred pebbles.
For this reason, the TER1 track was created as one solid track with the gravel beeing added during the creation of the model in the Blender software and exported without real physical behavior.

For the simulation of the robot software algorithm the following computer setup was used:
- Ubuntu 22.04 (running on virtual machine via OracleVM)
- Intel i7-12700F (12 cores with up to 4,9 GHz -> 6 cores were assigned to the vm)
- DDR4 RAM (32 GB -> 16 GB were assigned to the vm)
- RTX 3060 Ti (the vm was configured to run the VMSVGA graphics controller with 3D Acceleration enabled)

It is likely that the performance of the Gazebo simulator can be improved by running it on a native Ubuntu operating system instead of a virtual machine.
However, other simulator software should be considered for further work on this project.

### Texture error in the Gazebo Simulator
The textures added in the Blender software cause the [TER0_texture](README.md#TER0_texture) track to not work correctly in the Gazebo simuator causing it to crash upon inserting the track model into the world. 
During testing, the same track model without the textures added worked fine and as expected like all other track models exported from the Blender software and imported into the Gazebo simulator.
However the error causing the Gazebo software to crash could not be determined. Since the textures do not provide any functional benefit to the simulation per se, but only a visual improvement, the bug fix was placed lower in the prioritization of tasks.

## Team
Projektarbeit im Masterstudiengang Elektronische- und Mechatronische Systeme (MSY) an der Technischen Hochschule Nürnberg Georg-Simon-Ohm.
Diese Arbeit wurde im Wintersemester 2022 / 2023 durchgeführt. Die Teammitglieder sind:
- Katarina Weigel
- Jakob Richter
- Daniel Meissner

## Installation

### Native

### Using Docker Container

## Usage

### Robots
