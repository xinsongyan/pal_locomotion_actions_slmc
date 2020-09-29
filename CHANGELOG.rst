^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_locomotion_actions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2020-07-28)
------------------
* Merge branch 'documentation_review' into 'erbium-devel'
  Documentation review
  See merge request control/pal_locomotion_actions!13
* Documentation review
* Update README
* Add README
* Contributors: Adria Roig, narcismiguel

0.3.0 (2020-07-10)
------------------
* Merge branch 'robot_experiments' into 'erbium-devel'
  Robot experiments
  See merge request control/pal_locomotion_actions!12
* Merge branch 'robot_experiments' of gitlab:control/pal_locomotion_actions into robot_experiments
* Add hybrid_factor as a param
* Set hybrid factor to 0.5
* Set hybrid factor to 0.5
* Contributors: Adria Roig, victor

0.2.4 (2020-07-08)
------------------
* Merge branch 'talos_6_fixes' into 'erbium-devel'
  Static walk fixes for real robot
  See merge request control/pal_locomotion_actions!11
* Static walk fixes for real robot
* Contributors: Adria Roig, victor

0.2.3 (2020-06-06)
------------------
* Merge branch 'fix_tests' into 'erbium-devel'
  Fix tests
  See merge request control/pal_locomotion_actions!10
* fix the com error computation to only 2D
* fix the swing leg height computation
* Contributors: Sai Kishor Kothakota, victor

0.2.2 (2020-05-15)
------------------

0.2.1 (2020-05-15)
------------------
* Merge branch 'swing_interp' into 'erbium-devel'
  Add catmul-rom interpolator inside the static_walk action
  See merge request control/pal_locomotion_actions!9
* Add catmul-rom interpolator inside the static_walk action
* Contributors: Adrià Roig, victor

0.2.0 (2020-03-04)
------------------
* Merge branch 'fix-thread-stop' into 'erbium-devel'
  CHeck for thread interruption, and join on destructor
  See merge request control/pal_locomotion_actions!8
* Static walk can end if in double support and another action is queued
* Add atomic bool to synchronize wbc action thread
  A condition variable can miss a notify() if it is called when not on a
  wait() call.
  See https://stackoverflow.com/questions/17562908/what-if-notify-is-called-before-wait
* Check for thread interruption, and join on destructor
* Contributors: Victor Lopez, victor

0.1.0 (2020-03-03)
------------------
* Merge branch 'static_walking' into 'erbium-devel'
  changes for static walking on the real robot
  See merge request control/pal_locomotion_actions!7
* Add missing namespace for balancing_action_parameters
* Remove callgrind files
* fix clamp in orientation
* static walk refactor
* changes for static walking on the real robot
* Contributors: Adria Roig, Victor Lopez, victor

0.0.5 (2019-10-07)
------------------
* Merge branch 'fix-shadow-variable' into 'erbium-devel'
  Fix shadowed variable
  See merge request control/pal_locomotion_actions!6
* Fix shadowed variable
* Contributors: Victor Lopez

0.0.4 (2019-07-29)
------------------
* Merge branch 'memmo' into 'erbium-devel'
  Memmo
  See merge request control/pal_locomotion_actions!5
* fixed reference bug, changed from tf2 to tf
* finally vertical planning is continuious
* working continuity on static actions
* working up and down actions
* cleanup and restructuring of files
* Contributors: Hilario Tome

0.0.3 (2019-07-24)
------------------
* Merge branch 'inertia_id' into 'erbium-devel'
  standown action
  See merge request control/pal_locomotion_actions!3
* changed static walking action from ankle to foot reference frame
* standown action
* Contributors: Hilario Tome

0.0.2 (2019-05-23)
------------------
* Merge branch 'added_node' into 'erbium-devel'
  Added node
  See merge request control/pal_locomotion_actions!1
* added node
* Contributors: Hilario Tome

0.0.1 (2019-05-20)
------------------
* Merge branch 'erbium-devel' of gitlab:control/pal_locomotion_actions into erbium-devel
* fixed package xml
* Fix maintainer email
* updated changelog
* Contributors: Hilario Tome, Victor Lopez
