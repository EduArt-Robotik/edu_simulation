A Collection of Commands to run the fair_demonstration

- **Gazebo**
```bash
ros2 launch edu_simulation gazebo.launch.py world:=pepperl+fuchs.world
```

- **Spawn Robot**
```bash
ros2 launch edu_simulation eduard.launch.py lidar_type:=pf_R2400
```

- **Slam**
```bash
ros2 launch edu_simulation slam.launch.py
```

- **Localization with predefined map**
(change filepath if needed)
```bash
MAP_FILE=/workspaces/eduart/src/edu_simulation/model/pepperl+fuchs/map/map.yaml  ros2 launch edu_simulation slam.launch.py
```

- **Nav2**
```bash
ros2 launch edu_simulation navigation.launch.py
```

- **Rviz2**
```bash
ros2 launch edu_robot_control eduard_monitor_nav2.launch.py
```