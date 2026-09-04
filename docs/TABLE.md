# Topic Database: Personal

| Topic | Data type / format | Function | Publisher count | Subscriber count |
|---|---|---|---:|---:|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | Point selected/clicked in a visualization interface.  | — | — |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock. Publishes simulated time rather than real wall-clock time. Your output increments very rapidly. | — | — |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial pose information, likely associated with navigation/RViz-style pose initialization. | **1** | **0** |
| `/main_camera/detection/bounding_boxes` | **Bounding-box detection message; | **Vision/perception.** Gives detected objects and their normalized bounding boxes, confidence, IDs and labels. | — | — |
| `/mavros/global_position/compass_hdg` | `std_msgs/msg/Float64` | Compass heading of the AUV, in degrees. Observed value: `262.1867`. | — | — |
| `/mavros/global_position/rel_alt` | `std_msgs/msg/Float64` | Relative altitude/depth-type measurement. Observed: `-2.11265`. | — | — |
| `/mavros/imu/data` | `sensor_msgs/msg/Imu` | IMU state: orientation, angular velocity and linear acceleration. | — | — |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | **MASTER movement command.** Linear `x/y/z` + angular `x/y/z`. | — | — |
| `/mavros/setpoint_velocity/cmd_vel_unstamped/r` | `std_msgs/msg/Float32` | **Yaw movement command** according to your simulator's custom interface. | — | — |
| `/mavros/setpoint_velocity/cmd_vel_unstamped/x` | `std_msgs/msg/Float32` | **Surge:** forward/backward movement. | — | — |
| `/mavros/setpoint_velocity/cmd_vel_unstamped/y` | `std_msgs/msg/Float32` | **Sway:** left/right movement. | — | — |
| `/mavros/setpoint_velocity/cmd_vel_unstamped/z` | `std_msgs/msg/Float32` | **Heave:** up/down movement. | — | — |
| `/mavros/state` | `mavros_msgs/msg/State` | Flight-controller state: connected, armed, guided, manual input, mode, system status. Your robot was `connected=true`, `armed=true`, `guided=true`, `mode=GUIDED`. | — | — |
| `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | Navigation goal pose. Don't fully understand it yet... | — | — |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ROS 2 parameter-change events. Not relevant to gate/flare task. Unsure of Purpose| — | — |
| `/rosout` | `rcl_interfaces/msg/Log` | ROS logging/debug information. | — | — |
| `/tf` | `tf2_msgs/msg/TFMessage` | **Coordinate transforms / positions of objects and robot. Extremely important.** Contains `map → object` and `map → auv/base_link` transforms. | — | — |