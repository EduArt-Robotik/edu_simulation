## Models
This folder contains subfolder for every model currently available for simulation including both the mecanum robot and the offroad robot from EduArt as well as the track models.
For the track models, every subfolder contains a Blender folder with the Blender file (.blend) as well as a picture of the track (.jpg) and a mesh folder with the exported models.
The models can be exported in many different files, but for simulation purposes in gazebo, every track was exported in the Collada file format (.dae) and the STL file format (.stl).
Furthermore every folder contains an SDFormat file (.sdf) as well as a configuration file (.config), which both are used by the Gazebo simulator to import the model exported from the Blender software
and create a gazebo model.

### Eduard_mecanum
This folder contains the model of the mechanum robot from EduArt.

### Eduard_offroad
This folder contains the model of the offroad robot from EduArt.

### Gravel
This folder contains a gravel model, in which pebble stones were arranged in a random order as a grid. 
The gravel model consists of 65 individual pebble stones, which are arranged in 5 rows with 13 pebbles per row as a grid. 
This gravel model is an exception to the other models in the model folder, as it is not exported directly from Blender, but is a combination of different Blender models merged together in the .sdf file.
The advantage is that the gravel now has somewhat realistic physical behavior, so each individual pebble can be moved or rotated independently of the others by the robot.
The models used for this gravel grid are [Pebble1](README.md#Pebble1), [Pebble2](README.md#Pebble2), [Pebble3](README.md#Pebble3) and [Pebble4](README.md#Pebble4).
![Gravel](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Gravel/gravel.png)

### Older_versions: creation process
This folder contains old versions of the TER0 track and shows all steps of the development of the base track.
The basic track was labeled as TER0, since all other tracks of the TER category (i.e. TER1, TER2 and TER3) can be built from it by adding the different track surfaces.
The following list explains all steps that were done to create the TER0 base track. <br>
All TER tracks are built mainly with cubes, which is one of the default object types in Blender and changed in dimensions. In addition, every single part of the track was modified to be solid.
To achieve this, every part was modified in the physics properties section to have a passive rigid body type physic, which prevents the robot from falling throught the track.
The dimension for every part of the tracks were taken from the [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) 
and are given below in inch with the format: length x width x hight.

Development steps:
1. TER0_1: The base panel for one track section was created with the dimension: 96 x 48 x 5/8 (= 0.625) (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 15: A).
![TER0_1](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_1.png)

2. TER0_2: The base panel was duplicated and added at the correct position to create a base panel for the whole track.
![TER0_2](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_2.png)

3. TER0_3: Two ramps were created and moved to the correct position. The dimension for the ramp are the same as the base panel except in hight.
	For a 15° ramp, a cube with the hight of 26 inch is needed. In the *edit mode* of Blender, the ramp was created by using the *edge slide tool*.
![TER0_3](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_3.png)

4. TER0_4: Two long beams were created with the dimension: 96 x 2 x 4 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 15: B).
![TER0_4](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_4.png)

5. TER0_5: Two short beams were created with the dimension: 45 x 2 x 4 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 15: C)
	Together with the two long beams from the previous step, all four beams were moved to the correct position to create a basin.
![TER0_5](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_5.png)

6. TER0_6: The four beams were duplicated and positioned to the other parts of the track.
![TER0_6](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_6.png)

7. TER0_7a: The two ends of the surrounding wall were created with the dimensions: 1/2 (= 0.5) x 48 x 24 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 15: N).
![TER0_7a](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_7a.png)

8. TER0_7b: Three long section of the wall were created with the dimension: 1/2 (= 0.5) x 96 x 24 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 15: M).
	In addition, one short section of the wall was created with the dimensions of the previous step.
![TER0_7b](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_7b.png)

8. TER0_7c: The four high section of the wall were created with the dimension: 1/2 (= 0.5) x 48 x 48 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 15: O).
![TER0_7c](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_7c.png)

9. TER0_7d: The last two sections of the wall were created with the dimension: 1/2 (= 0.5) x 48 x 24 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 15: N).
	The last missing section of the wall is the entry point of the robot, so there is no wall section needed.
![TER0_7d](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_7d.png)

10. TER0_8: Five supporting beams were created and positioned in the corners of wall of the track with the dimension: 4 x 4 x 12 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 15: R).
	In the simulation, these supporting beams provide no funcional use, but were added nevertheless, so that the simulation model better matches the real track.
![TER0_8](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_8.png)

11. TER0_9a: For the operator booth, a fifth base panel was added with the same dimension as the other four from step 1. 
	Furthermore two supporting beams were created with the dimension: 4 x 2 x 96 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 8: B).
![TER0_9a](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_9a.png)

12. TER0_9b: Three wall panels were created with the dimension: 48 x 1/2 (= 0.5) x 96 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 8: A).
![TER0_9b](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_9b.png)

13. TER0_9c: One short supporting beam was created with the dimension: 2 x 48 x 4 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 8: D) as well as the operator table
	with the dimension: 48 x 24 x 4 (see [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 8: C, E, F).
![TER0_5](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Older_versions/TER0_9c.png)

### Pebble1
This folder contains a first random single pebble stone, which can be used to manually create gravel or add to existing gravel.
This pebble stone was used to create a fully functional gravel model for the Gazebo simulator with somewhat real gravel physics. 
To create the gravel model, the four pebble stones were arranged in a random order as a grid. For more information see [Gravel](README.md#gravel).
![Pebble1](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Pebble1/blender/Pebble1.png)

### Pebble2
This folder contains a second random single pebble stone, which can be used to manually create gravel or add to existing gravel.
This pebble stone was used to create a fully functional gravel model for the Gazebo simulator with somewhat real gravel physics. 
To create the gravel model, the four pebble stones were arranged in a random order as a grid. For more information see [Gravel](README.md#Gravel).
![Pebble2](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Pebble2/blender/Pebble2.png)

### Pebble3
This folder contains a third random single pebble stone, which can be used to manually create gravel or add to existing gravel.
This pebble stone was used to create a fully functional gravel model for the Gazebo simulator with somewhat real gravel physics. 
To create the gravel model, the four pebble stones were arranged in a random order as a grid. For more information see [Gravel](README.md#Gravel).
![Pebble3](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Pebble3/blender/Pebble3.png)

### Pebble4
This folder contains a fourth random single pebble stone, which can be used to manually create gravel or add to existing gravel.
This pebble stone was used to create a fully functional gravel model for the Gazebo simulator with somewhat real gravel physics. 
To create the gravel model, the four pebble stones were arranged in a random order as a grid. For more information see [Gravel](README.md#Gravel).
![Pebble4](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/Pebble4/blender/Pebble4.png)

### TER0_basin
This folder contains the basic TER0 track without sand or gravel. 
The ramp is effectively a basin that can be manually filled with individual pebbles (see [pebble1](README.md#pebble1)) or a gravel grid (see [Gravel](README.md#Gravel)).
![TER0_basin](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/TER0_basin/blender/TER0_basin.png)

### TER0_plain
This folder contains the basic TER0 track, but the two ramps were removed to create a flat track.
![TER0_plain](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/TER0_plain/blender/TER0_plain.png)

### TER0_ramp
This folder contains the TER0 track, but the two ramps are filled with a solid material to create a simplified version of the TER1 track with no gravel or sand built into the model.
This version of the TER0 track was used 
![TER0_ramp](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/TER0_ramp/blender/TER0_ramp.png)

### TER0_texture
This folder contains the TER0 track with somehwhat realistic textures. The textures were downloaded from [AmbientCG](https://ambientcg.com/), which contains textures for rendering 
and operates under the *Creative Commons CC0 1.0 Universal License* (see: [license information](https://docs.ambientcg.com/books/website-licensing/page/license-information) ). <br>
**This track does not yet work in the Gazebo simulator, trying to use this track will result in Gazebo crashing**.
![TER0_texture](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/TER0_texture/blender/TER0_texture.png)

### TER1
This folder contains the TER1 track. To create the TER1 track model, the TER0 basin track was filled with gravel in the Blender software.
In the Gazebo simulator, the TER1 track is one entity, which means, that the pebble stones do not have real physics, but are "glued" to the track model and can not be moved etc...
This massively reduces the computing power and significantly increases the performance of the simulation.
![TER1](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/TER1/blender/TER1.png)

### TER2
This folder contains the TER2 track. To create the TER2 track model, the TER0 basin track was filled with two diagonal wooden beams per section according to the [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 16.
![TER2](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/TER2/blender/TER2.png)

### TER3
This folder contains the TER3 track. To create the TER3 track model, the TER0 basin track was filled with eight smaller ramps per section according to the [Robocub Assembly Guide](https://rrl.robocup.org/wp-content/uploads/2022/05/RoboCup2022_AssemblyGuide_Final.pdf) page 18.
![TER3](https://github.com/EduArt-Robotik/edu_simulation/blob/feature/sand_gravel_ramp/model/TER3/blender/TER3.png)

## Problems

### Texture error in the Gazebo Simulator
The textures added in the Blender software cause the [TER0_texture](README.md#TER0_texture) track to not work correctly in the Gazebo simuator causing it to crash upon inserting the track model into the world. 
During testing, the same track model without the textures added worked fine and as expected like all other track models exported from the Blender software and imported into the Gazebo simulator.
However the error causing the Gazebo software to crash could not be determined. Since the textures do not provide any functional benefit to the simulation per se, but only a visual improvement, the bug fix was placed lower in the prioritization of tasks.
