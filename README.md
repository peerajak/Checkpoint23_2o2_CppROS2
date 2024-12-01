# Checkpoint23_2o2_CppROS2

Terminal 1

```
source ~/ros2_ws/install/setup.bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True
```

Terminal 2

```
source ~/ros2_ws/install/setup.bash
ros2 launch tortoisebot_waypoints tortoisebot_action_server.launch.py
```

Terminal 3

```
source ~/ros2_ws/install/setup.bash
ros2 launch tortoisebot_waypoints tortoisebot_action_client.launch.py
```