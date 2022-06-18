# Graceful Controller

This implement a controller based on "A Smooth Control Law for Graceful
Motion of Differential Wheeled Mobile Robots in 2D Environments" by Park
and Kuipers, ICRA 2011.

Currently this does not implement section IV.B of the paper (which
describes how to switch between poses along a path). Instead this controller
has a naive approach which attempts to find the farthest pose in the path
which is both A) less than some maximum lookahead distance away, and B)
reachable (without collision) using our control law (as determined by
a forward simulation). We call this the **target_pose**.

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

 * **k1** - controls convergence of control law (slow subsystem). A value of 0 reduces the controller to pure waypoint-following with no curvature. For a high k1 value, theta will be reduced faster than r.
 * **k2** - controls convergence of control law (fast subsystem). A higher value of k2 will reduce the distance of the path to the target, thus decreasing the path's curvature.
 * **lambda** - controls speed scaling based on curvature. A higher value of lambda results in more sharply peaked curves.
 * **beta** - controls speed scaling based on curvature. A higher value of beta lets the robot's velocity drop more quickly as K increases. K is the curvature of the path resulting from the control law (based on k1 and k2).

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
* **min_lookhead** - the target pose cannot be closer than this distance
   away from the robot. This parameter avoids instability when an unexpected
   obstacle appears in the path of the robot by returning failure, which
   typically triggers replanning.
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
 * **latch_xy_goal_tolerance** - similar to many other local controllers in ROS,
   this will prevent hunting around the goal by latching the XY portion of the
   goal when the robot is within the goal tolerance. The robot will then rotate
   to the final heading (while possibly being outside of the real XY goal tolerance).
 * **compute_orientations** - if the global planner does not compute orientations
   for the poses in the path, this should be set to true.
 * **use_orientation_filter** - since the controller is highly dependent on the
   angular heading of the target pose it is important the angular values are
   smooth. Some global planners may produce very unsmooth headings due to
   discretization error. This optional filuter can be used to smooth
   out the orientations of the global path based on the **yaw_filter_tolerance**
   and **yaw_gap_tolerance**.
 * **yaw_filter_tolerance** - a higher value here allows a path to be more
   zig-zag, a lower filter value will filter out poses whose headings diverge
   from an overall "beeline" between the poses around it. units: radians
 * **yaw_gap_tolerance** - this is the maximum distance between poses, and
   so even if a pose exceeds the filter tolerance it will not be removed
   if the gap between the pose before and after it would exceed this value.
   units: meters.

Most of the above parameters can be left to their defaults and work well
on a majority of robots. The following parameters, however, really do
depend on the robot platform itself and must be set as accurately as
possible:

 * **acc_lim_x** - this is likely the **most important** parameter to set
   properly. If the acceleration limits are overestimated the robot will not
   be able to follow the generated commands. During braking, this can cause
   the robot to crash into obstacles. Units: meters/sec^2.
 * **acc_lim_theta** - the same notes as for _acc_lim_x_, but in rad/sec^2.
 * **decel_lim_x** - (new in 0.4.3) this acceleration limit is used only
   within the underlying control law. It controls how quickly the robot slows
   as it approaches the goal (or an obstacle blocking the path, which causes
   the target_pose to be closer). If this is left as the default of 0.0, the
   controller will use the same acc_lim_x. Units: meters/sec^2.
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
 * **xy_vel_goal_tolerance** - this is the maximum linear velocity the robot
   can be moving in order to switch to final in place rotation, latch the
   goal, or report that the goal has been reached. This parameter is ignored
   if no **odom_topic** is set.
 * **yaw_vel_goal_tolerance** - this is the maximum angular velocity the robot
   can be moving in order to switch to final in place rotation, latch the
   goal, or report that the goal has been reached. This parameter is ignored
   if no **odom_topic** is set.

A final feature of the controller is footprint inflation at higher speeds. This
helps avoid collisions by increasing the padding around the robot as the speed
increases. This behavior is controlled by three parameters:

 * **scaling_vel_x** - above this speed, the footprint will be scaled up.
   units: meters/sec.
 * **scaling_factor** - this is how much the footprint will be scaled when
   we are moving at max_vel_x. The actual footprint size will be:
   1.0 + scaling_factor * (vel - scaling_vel_x) / (max_vel_x - scaling_vel_x).
   By default, this is set to 0.0 and thus disabled.
 * **scaling_step** - this is how much we will drop the simulated velocity
   when retrying a particular target_pose.

Example: our robot has a max velocity of 1.0 meters/second, **scaling_vel_x**
of 0.5 meters/second, and a **scaling_factor** of 1.0. For a particular
target_pose, if we are simulating the trajectory at 1.0 meters/second, then
the footprint will be multiplied in size by a factor of 2.0. Suppose this
triggers a collision in simulation of the path and that our **scaling_step**
is 0.1 meters/second. The controller will re-simulate at 0.9 meters/second,
and now our footprint will only be scaled by a factor of 1.8. If this were
not collision free, the controller would re-simulate with a velocity of
0.8 meters/second and a scaling of 1.6.

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
