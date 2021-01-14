^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package graceful_controller_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
