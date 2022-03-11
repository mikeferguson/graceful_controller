^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package graceful_controller_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
