# Graceful Controller

This implement a controller based on "A Smooth Control Law for Graceful
Motion of Differential Wheeled Mobile Robots in 2D Environments" by Park
and Kuipers, ICRA 2011.

Currently this does not implement section IV.B of the paper (which
describes how to switch between poses along a path). Instead this controller
has a naive approach which attempts to use the farthest pose in the path
which is both A) less than some maximum lookahead distance away, and B)
reachable with our current control law parameters without collision.

## Example Config

There is an example configuration for running this controller on the UBR1
in ROS1 simulation in the
[ubr_reloaded](https://github.com/mikeferguson/ubr_reloaded/tree/ros1)
repository.

## Licensing

There are two ROS packages in this repo, with different licenses, since
we're really trying to blend two license-incompatible code bases:

 * graceful_controller - is LGPL licensed and is the low-level control
   law implementation, as originally implemented in the
   [fetch_open_auto_dock](https://github.com/fetchrobotics/fetch_open_auto_dock)
   package.
 * graceful_controller_ros - is BSD licensed and is the actual ROS wrapper
   and glue logic to connect to the ROS Navigation stack. It leverages code
   from base_local_planner and dwa_local_planner packages in the
   [ROS Navigation](https://github.com/ros-planning/navigation) stack.
