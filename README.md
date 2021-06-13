# Graceful Controller

This implement a controller based on "A Smooth Control Law for Graceful
Motion of Differential Wheeled Mobile Robots in 2D Environments" by Park
and Kuipers, ICRA 2011.

Currently this does not implement section IV.B of the paper (which
describes how to switch between poses along a path). Instead this controller
has a naive approach which attempts to find the farthest pose in the path
which is both A) less than some maximum lookahead distance away, and B)
reachable (without collision) using our control law (as determined by
a forward simulation).

## ROS Topics

As with nearly all navigation local controllers, our controller outputs
both a global path and a local path as a _nav_msgs::Path_.

The "local path" is derived through a forward simulation of where
the robot is expected to travel by iteratively applying the control law.
Unlike the DWA controller, our forward simulation is NOT based on applying
the same command throughout a fixed simulation time, but rather based
on calling our control law multiple times as we forward simulate our
path to the "target pose". As long as the acceleration limits are
correctly specified, and the robot controllers can execute the velocities
commanded, this will be the true path that the robot will follow.

An additional topic that is unique to our controller is the "target_pose"
topic which is a _geometry_msgs::PoseStamped_. This is published every
time we compute the command velocity for the robot and tells you which
pose in the global plan we are using for computing the control law. It
is potentially helpful in debugging parameter tuning issues.

## Parameters

The underlying control law has several parameters which are best described
by the original paper, these include:

 * **k1** - controls convergence of control law.
 * **k2** - controls convergence of control law.
 * **lambda** - controls speed scaling based on curvature.
 * **beta** - controls speed scaling based on curvature.

Several parameters are used for selecting and simulating the target pose used
to compute the control law:

 * **max_lookahead** - the target pose cannot be further than this distance
   away from the robot. A good starting value for this is usually about 1m.
   Using poses that are further away will generally result in smoother
   operations, but simulating poses that are very far away can result in
   reduced performance, especially in tight or cluttered environments.
   If the controller cannot forward simulate to a pose this far away without
   colliding, it will iteratively select a target pose that is closer to the
   robot.
* **acc_dt** - this parameter is used to set the maximum velocity that the
   control law can use when generating velocities during the path simulation.

There are several major "features" that are optional and configured through
one or more parameters:

 * **odom_topic** - by setting this parameter to the name of the odometry
   topic, the controller will subscribe to the odometry topic and limit
   the control law velocities to those that are feasible based on the current
   robot velocity. Without odometry, performance will be degraded.
 * **use_vel_topic** - in ROS1 there is no standard way of defining a
   "speed limit map". If this parameter is set to true, the controller will
   subscribe to a _std_msgs::Float32_ topic called "max_vel_x" and use this
   to supercede our _max_vel_x_ parameter.
 * **initial_rotate_tolerance** - when the robot is pointed in a very
   different direction from the path, the control law (depending on k1 and k2)
   may generate large sweeping arcs. To avoid this potentially undesired behavior
   when the robot begins executing a new path, the controller can produce an
   initial in-place rotation to get the robot headed towards the path before it
   starts using the control law to follow it. The initial rotate tolerance is
   the maximum angular difference (in radians) between robot heading and the
   target pose. This feature can be disabled by setting _initial_rotate_tolerance_
   to 0.0.
 * **prefer_final_rotation** - similar to the initial rotation issue many
   plans may cause large arcs at the end of a path because the orientation of
   the final pose is significantly different from the direction the robot would
   drive to follow the path as planned. By setting _prefer_final_rotation_ to
   true, the orientation of the final pose will be ignored and the robot will
   more closely follow the path as planned and then produce a final in-place
   rotation to align the robot heading with the goal.
 * **yaw_filter_tolerance** - since the controller is highly dependent on the
   angular heading of the target pose it is important the angular values are
   smooth. Some global planners may produce very unsmooth headings due to
   discretization errors.

Most of the above parameters can be left to their defaults and work well
on a majority of robots. The following parameters, however, really do
depend on the robot platform itself and must be set as accurately as
possible:

 * **acc_lim_x** - this is likely the **most important** parameter to set
   properly. If the acceleration limits are overestimated the robot will not
   be able to follow the generated commands. During braking, this can cause
   the robot to crash into obstacles. Units: meters/sec^2.
 * **acc_lim_theta** - the same notes as for _acc_lim_x_, but in rad/sec^2.
 * **max_vel_x** - this is the maximum velocity the controller is allowed to
   specify. Units: meters/sec.
 * **min_vel_x** - when the commanded velocities have a high curvature, the
   robot is naturally slowed down (see the ICRA paper for details). The minimum
   velocity is the lowest velocity that will be generated when not executing
   an in-place rotation. Most robots have some amount of friction in their
   drivetrain and casters and cannot accurately move forward at speeds below
   some threshold, especially if also trying to turn slightly at the same time.
   Units: meters/sec.
 * **max_vel_theta** - the maximum rotation velocity that will be produced by
   the control law. Units: rad/sec.
 * **min_in_place_vel_theta** - this is the minimum rotation velocity that
   will be used for in-place rotations. As with _min_vel_x_, this is largely
   hardware dependent since the robot has some minimum rotational velocity
   that can be reliably controlled. Units: rad/sec.
 * **xy_goal_tolerance** - this is the linear distance within which we consider
   the goal reached. When _prefer_final_rotation_ is set to true, we will only
   generate final rotations once this tolerance is met. Units: meters.
 * **yaw_goal_tolerance** - this is the angular distance within which we consider
   the goal reached. Units: radians.

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
