# Graceful Controller

This implement a controller based on "A Smooth Control Law for Graceful
Motion of Differential Wheeled Mobile Robots in 2D Environments" by Park
and Kuipers, ICRA 2011.

Currently this does not implement section IV.B of the paper (which
describes how to switch between poses along a path). Instead this controller
has a naive approach which attempts to use the farthest pose in the path
which is both A) less than some maximum lookahead distance away, and B)
reachable with our current control law parameters without collision.

Outstanding issues:

 * The velocity selection is not sufficiently awesome when you are
   reaching the end of the path - so sometimes you overshoot the end of the
   path if there is no curvature immediately preceding it (see lines 67-74
   of graceful_controller.cpp).
 * Currently, we only check success in X/Y - if the controller came in
   at a bad angle, the heading will be wrong. Need to still implement a
   "turn to goal" like the latched controller in DWA/TrajRollout.
 * When k1/k2 are cranked high, oscillation occurs. But if you're pointed
   in a bad direction initially, it takes a very wide arc to get back on
   the path. Need to add a parameter to turn towards the path initially.
 * There are absolutely no tests.

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
