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
- startet den Knoten für das automatische Abfahren des Pfades
- aktuell: roboter dreht sich erst aus und fährt dann vorwärts; wird noch auf omnidirektionales Fahren erweitert
```bash
ros2 launch edu_simulation path_following.launch.py
```

## Rviz2
startet die Visualisationsumgebung, in der die Karte, der Robo und der Pfad angezeigt werden
```bash
ros2 launch edu_robot_control eduard_monitor_nav2.launch.py
```

## Virtual Joystick
über `docker compose up` im entsprechenden Ordner

## Path Publisher
- Legt einen Pfad fest, der abgefahren werden soll. Das Abfahren beginnt, sobald der Roboter im Virtual-Joy auf 'Autonomous' gestellt wird.
- Wegpunkte können in der `parameter/path_pub.yaml` hinzugefügt werden. Für jeden Wegpunkt werden x und y-Koordinate, sowie die Rotation (zum Kartenurspung) angegeben
- aktuell wird der standart Progress-checker verwendet, also wenn der roboter nach 10 sekunden nicht vorwärts gefahren ist (z.B. weil der Roboter noch auf autonomous steht), bricht das Pfadfolgen ab (wird für die fertige demo auch noch überarbeitet)
- Befehl zum starten:
```bash
ros2 launch edu_simulation path_publisher.launch.py
```
- alternativ kann der Pfad auch direkt über die ROS2-action `/eduard/follow_path` geschickt werden

## Slam (only for development)
```bash
ros2 launch edu_simulation slam.launch.py
```