# Getting started
To start this node and the corresponding test environment, just start the following launch file:
```bash
roslaunch hotaru_planner_node_teb test-planner-bare_with_obstacle.launch
```
If you start __rqt_reconfigure__ you can dynamically reconfigure paramaters of the planner.

__DISCLAIMER__: this architecture and implementation is in developmental phase, expect random errors, which will be corrected in the near future.

# Minimal installation
This architecture contains framework specific nodes, which may or may not be used. This framework currently interfaces ROS, therefore a simple ROS is required. For a minimal setup (e.g. TEB-based implementation), you might also need __ros-melodic-teb-planner__ package.

To build only the current TEB based implementation you can use the following command:
```bash
catkin build hotaru_planner_node_teb
```

# Autoware specific launch
This architecture is capable of interfacing Autoware: the bridge node awaits a lane produced by a global planner (e.g. A*) or a reference set of waypoints. To enable this interface, you can start bridge with the following launch file:
```bash
roslaunch hotaru_planner_node_teb hotaru_autoware_interface.launch
```
