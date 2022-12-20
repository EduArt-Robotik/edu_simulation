# edu_simulation

Contains Gazebo simulation models for all EduArt's robots.

## folder structure

### launch
This folder contains the eduard imulation launch file. In addition, the simulation can be launched with the following ROS2 command:
**ros2 launch edu_simulation eduard.launch.py**

### models
This folder contains subfolder for every model currently available for simulation including both the mecanum robot and the offroad robot from EduArt as well as the track models.
For the track models, every subfolder contains a blender folder with the blender file (.blend) as well as a picture of the track (.jpg) and a mesh folter with the exported models.
The models can be exported in many different files, but for simulation purposes in gazebo, every track was exported in the Collada file format (.dae) and the STL file format (.stl).

#### eduard_mecanum

#### eduard_offroad

#### gravel_plain

#### gravel_ramp1

#### gravel_ramp2

#### older_versions
This folder contains old versions of the TER0 route and shows all steps of the development of the base track.
The basic track was labeled as TER0, since all other tracks of the TER category (i.e. TER1, TER2 and TER3) can be built from it by adding the different track surfaces.
The following list explains all steps that were done to create the TER0 base track.
All TER tracks are built mainly with cubes, which is one of the default object types in blender. In addition, every single part of the track was modified to be solid.
To achieve this, every part was modified in the physics properties section to have a passive rigid body type physic, which prevents the robot from falling throught the track.
The dimension for every part of the tracks were taken from the [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf)

Development steps:
1. TER0_1: The base panel was created with the dimension
2. TER0_2:

#### pebble1

#### pebble2

#### pebble3

#### pebble4

#### TER1

#### TER2

#### TER3

### world



## Team
Katarina, Jakob, Daniel (Projektarbeit WS 2022/2023)