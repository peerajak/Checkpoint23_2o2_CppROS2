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

or with library node (same result as above)

```
source ~/ros2_ws/install/setup.bash
ros2 run tortoisebot_waypoints tortoisebot_waypoint_action_server_libexec_node
```


Terminal 3

```
source ~/ros2_ws/install/setup.bash
ros2 launch tortoisebot_waypoints tortoisebot_action_client.launch.py
```

or with library node (same result as above)

```
source ~/ros2_ws/install/setup.bash
ros2 run tortoisebot_waypoints tortoisebot_waypoint_action_client_libexec_node
```


## How to do teleopt on ros2

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

## Run dummy Gtest

```
source ~/ros2_ws/install/setup.bash
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+
```
