# Path Publisher

## run
```bash
ros2 launch edu_simulation path_publisher.launch.py 
```


## edit Waypoints

- get the current position of the robot from tf2

```bash
ros2 run tf2_ros tf2_echo eduard/pf_robot_a/map eduard/pf_robot_a/base_footprint
```

- edit [path_pub.yaml](../parameter/path_pub.yaml). Get the x and y position from translation and the yaw from rotation (RPY) in radians 