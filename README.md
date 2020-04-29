# Getting started
To start this node and the corresponding test environment, just start the following launch file:
```bash
roslaunch hotaru_planner_node_teb test-planner-bare_with_obstacle.launch
```
If you start __rqt_reconfigure__ you can dynamically reconfigure paramaters of the planner.

__DISCLAIMER__: this architecture and implementation is in developmental phase, expect random errors, which will be corrected in the near future.

# Autoware specific launch
This architecture is capable of interfacing Autoware: the bridge node awaits a lane produced by a global planner (e.g. A*) or a reference set of waypoints. To enable this interface, you can start bridge with the following launch file:
```bash
roslaunch hotaru_planner_node_teb hotaru_autoware_interface.launch
```
