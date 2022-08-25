^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package graceful_controller_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.5 (2022-08-24)
------------------
* compute lookahead as distance along path (`#55 <https://github.com/mikeferguson/graceful_controller/issues/55>`_)
* Contributors: Michael Ferguson

0.4.4 (2022-08-24)
------------------
* always rotate to match desired heading (`#54 <https://github.com/mikeferguson/graceful_controller/issues/54>`_)
  If the goal tolerances are large (greater than 0.5m) the
  robot might end up trying to point at the goal rather than
  aligning with the goal heading
* Contributors: Michael Ferguson

0.4.3 (2022-06-18)
------------------
* use separate decel_lim_x for control law (`#45 <https://github.com/mikeferguson/graceful_controller/issues/45>`_)
* add velocity limit to goal tolerance (`#44 <https://github.com/mikeferguson/graceful_controller/issues/44>`_)
* fix deceleration time (`#43 <https://github.com/mikeferguson/graceful_controller/issues/43>`_)
  this reverts `#6 <https://github.com/mikeferguson/graceful_controller/issues/6>`_, to what is apparently the correct math
* add footprint inflation for higher speeds (`#32 <https://github.com/mikeferguson/graceful_controller/issues/32>`_)
  * add a feature to inflate the footprint at higher speeds
  * only update max_vel_x once per call
* initialze collision_points\_ to null (`#35 <https://github.com/mikeferguson/graceful_controller/issues/35>`_)
* add optional visualization of colliding points (`#34 <https://github.com/mikeferguson/graceful_controller/issues/34>`_)
* split out hpp file, clang format (`#33 <https://github.com/mikeferguson/graceful_controller/issues/33>`_)
* fix bug in latch_xy_goal_tolerance (`#31 <https://github.com/mikeferguson/graceful_controller/issues/31>`_)
  While the main control loop can latch the goal (thus it will stop trying to hunt around) - that isn't understood by the isGoalReached() function - which leads to the robot wiggling back and forth for a while
* Contributors: Michael Ferguson

0.4.2 (2022-03-29)
------------------
* fix goal tolerance reset (`#29 <https://github.com/mikeferguson/graceful_controller/issues/29>`_)
* add latch_xy_goal_tolerance parameter (`#28 <https://github.com/mikeferguson/graceful_controller/issues/28>`_)
* fix orientation issue with collision checking (`#27 <https://github.com/mikeferguson/graceful_controller/issues/27>`_)
* Contributors: Michael Ferguson

0.4.1 (2022-03-11)
------------------
* add ability to disable orientation filtering (`#26 <https://github.com/mikeferguson/graceful_controller/issues/26>`_)
  also make everything dynamic reconfigurable
* don't crash when path is empty (`#25 <https://github.com/mikeferguson/graceful_controller/issues/25>`_)
* add min_lookahead parameter (`#24 <https://github.com/mikeferguson/graceful_controller/issues/24>`_)
  when the pose is very close to the robot, we can get
  some instability (rapid side-to-side movement due to
  large angular errors over small linear distances). by
  adding this parameter, we can instead fallback to
  recovery behaviors and find a better path.
* fix occasional boost::lock crash (`#23 <https://github.com/mikeferguson/graceful_controller/issues/23>`_)
* limit the distance between poses as filter runs (`#21 <https://github.com/mikeferguson/graceful_controller/issues/21>`_)
* improve the orientation filter, add tests (`#20 <https://github.com/mikeferguson/graceful_controller/issues/20>`_)
* Contributors: Michael Ferguson

0.4.0 (2021-06-14)
------------------
* add usage documentation (`#19 <https://github.com/mikeferguson/graceful_controller/issues/19>`_)
  also remove unused min_vel_theta parameter
* implement changes from review (`#18 <https://github.com/mikeferguson/graceful_controller/issues/18>`_)
  * Add collision checking during final rotation
  * Add comments about getRobotVel returning velocities
  * Rename path to simulated_path for clarity
  * Rename pose to target_pose and goal_pose (now separate variables)
  * Get rid of magic number for when to use final rotation
* simulate intial rotation properly (`#17 <https://github.com/mikeferguson/graceful_controller/issues/17>`_)
  previously, this simulated the big arc that we wouldn't follow anyways.
  this could cause the robot to get stuck in corners or other tight areas.
* Contributors: Michael Ferguson

0.3.1 (2021-05-27)
------------------
* add feature for final rotation (`#15 <https://github.com/mikeferguson/graceful_controller/issues/15>`_)
  if the final pose orientation is very different from the end
  of the path, we get a big sweeping turn. this feature uses
  the previous orientation to avoid that sweeping turn and
  instead does a final in-place rotation. Add filter and pose
  publisher.
* not sure how build is working without this being executable
* Contributors: Michael Ferguson

0.3.0 (2021-01-14)
------------------
* unitialized limits causes test failures (`#14 <https://github.com/mikeferguson/graceful_controller/issues/14>`_)
  limits does not initialize prune_plan to a value,
  causes flaky test when it ends up true (would also
  probably be bad on a real robot)
* Contributors: Michael Ferguson

0.2.2 (2021-01-13)
------------------
* support robot footprint (`#13 <https://github.com/mikeferguson/graceful_controller/issues/13>`_)
* cleanup parameters (`#12 <https://github.com/mikeferguson/graceful_controller/issues/12>`_)
  * drop unused parameters
  * manage parameters directly
* Contributors: Michael Ferguson

0.2.1 (2021-01-11)
------------------
* update maintainer email
* fix the buildfarm build (`#8 <https://github.com/mikeferguson/graceful_controller/issues/8>`_)
* Contributors: Michael Ferguson

0.2.0 (2021-01-11)
------------------
* Initial release
* Contributors: Michael Ferguson
