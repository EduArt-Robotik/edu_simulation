# A Collection of Commands to run the fair_demonstration

## Gazebo
startet die Simulationsumgebung; nach dem Start unten links auf Play gehen, damit physiksimulation beginnt
```bash
ros2 launch edu_simulation gazebo.launch.py world:=pepperl+fuchs.world
```

## Spawn Robot
platziert den Roboter in die Umgebung
```bash
ros2 launch edu_simulation eduard.launch.py lidar_type:=pf_R2400
```
## Localization with predefined map
Startet die Lokalisation, sodass der Roboter anhand der vorher aufgezeichneten karte seine Position findet.  
evtl muss der Dateipfad vor dem Starten angepasst werden 
```bash
MAP_FILE=/workspaces/edu_ros2/src/edu_simulation/model/pepperl+fuchs/map/map.yaml  ros2 launch edu_simulation slam.launch.py
```

## Navigation
startet den Knoten für das automatische Abfahren des Pfades
```bash
ros2 launch edu_simulation path_following.launch.py
```

## Rviz2
startet die Visualisationsumgebung, in der die Karte, der Robo und der Pfad angezeigt werden
```bash
ros2 launch edu_robot_control eduard_monitor_nav2.launch.py
```

## Virtuel Joystick
über `docker compose up` im entsprechenden Ordner

## Slam (only for development)
```bash
ros2 launch edu_simulation slam.launch.py
```