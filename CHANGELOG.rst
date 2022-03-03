^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package control_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [action_server]: fixed cancelling and aborting of goals
* [action_server]: fixed crash when cancelling goal due to bad lambda capture
* updated action_client script
* fixed goal aborting for action server
* goal cancelling added, fixed a deadlock with mission manager
* minor refactoring of action server
* set action_server to own callback_grp
* Merge branch 'action_server' of github.com:fly4future/control_interface into action_server
* working version of action_server
* minor update in enums.h
* [mission_manager]: minor refactoring for consistency and readability
* [mission_manager]: minor fix of retry attempts
* changed mission_start retry from number to interval
* updated new fog_msgs/srv/Vec4 type handling
* mavsdk debug messages will now be printed using ROS_INFO (if enabled)
* fixed home offset - now applied consistently
* added result and feedback checking into action_client_example
* added action server method and simple example
* update to new mission_progress msg
* update diagnostics message
* Contributors: Matouš Vrba, Vojtech Spurny

0.1.0 (2022-02-02)
-----------
* added an arming_ready state. Now, after the arm service is complete, takeoff can be called immediately
* the after-takeoff mission is now sent when transitioning from taking_off to autonomous_flight state
* manual takeoff during landing should no longer spam
* moved common functions to fog_lib, renamed yaw to heading for consistency
* refactorng mission-related stuff to MissionManager
* refactoring of mission-related stuff to a separate class MissionManager, WIP
* mission start retries added, no longer checking health mid-flight
* switch back from manual now checks the vehicle_state\_ instead of pixhawk's landed_state, minor refactoring
* add ID to navigation requests, send confirmation back in diagnostics
* nicer printing of waypoints
* fixed include folder export
* moved state enums to an include to allow easy including from navigation, minor fixes
* Merge branch 'matous' of github.com:fly4future/control_interface into matous
* added fog_lib dependency
* takeoff waypoint height is relative to averaged takeoff position, added a more detailed print if not healthy
* now publishes cmd_odom from pixhawk as PoseStamped, resetting mission progress counters, other minor stuff
* working version, refactoring, fixes
* updated the diagnostics message, started rewriting cmd_pose\_ to be more correct (but it's not working 100%)
* renamed desired_pose to cmd_pose for consistency
* fixed deadlock in landing
* improved reporting of current mission status, added some prints
* mission upload is only cancelled if a mission is being uploaded, other minor fixes
* removed pause_mission in stopMission that may be causing pixhawk to switch from manual
* added some prints in case mission cannot be stopped
* fixed a potential rare deadlock in mavsdk callback thread
* only callback durations over a certain threshold will be printed
* stopping mission before transitioning to the not_ready state to prevent buffering of missions in pixhawk that will be executed right after arming, which is dangerous
* fixed mission aborting, other minor QoL improvemetns
* added an extra check before taking off that the vehicle is not armed and printing of callback timings
* improved state update after takeoff to work also with instant takeoff, minor changes with takeoff/landing prints
* manual/automat fixes for real drone
* added throttle to an annoying output
* fixes in manual override after during takeoff
* now prints number of GPS samples received/required, changed GPS position buffer to circular_buffer
* mavsdk logging to file can be disabled by setting filename to empty
* throttled some output, added mavlink logging
* added more verbose output for failures, diags are published in separate thread, waiting for octomap reset, other minor fixes and updates
* minor fixes with mission cancelling and other little things
* Merge branch 'devel' into matous
* reverted -Og to -O3
* refactored to use vehicle states instead of a multitude of flags
* mission progress deadlock hotfix
* fixed callback group
* takeoff hotfix
* wip implementing vehicle state
* working. Mission state is now published. Changed some mutexes to recursive to avoid deadlocks because of how callbacks are called.
* fixed callback groups
* changed callback groups to multiple single-threaded ones
* refactoring waypoint callbacks
* wip cleanup and refactoring to squash bugs due to parallelization
* Update build scripts
* Update topic names for rebased PX4
* Add containerized build workflow
* Fix deps_ws directory in packaging script
* Update galactic support to github  workflows
* New build scripts with container build
* cherry-pick mavsdk init separation from branch galactic
* fixes of warnings found using ros2 galactic (`#17 <https://github.com/tiiuae/control_interface/issues/17>`_)
  Co-authored-by: Vojtech Spurny <vojtech.spurny@fel.cvut.cz>
* New build scripts

0.0.8 (2021-11-30)
-----------
* remove unnecessary service client
* update package versions
* cleanup on package.xml and removing unused includes
* add param loading guard
* polished config file
* renamed odometry pkg name
* wait for odometry message only, no services
* remove unused params
* report exit symbol on mission start failure
* Merge branch 'odometry2' of github.com:tiiuae/control_interface into odometry2
* reorganize params under namespaces
* Merge branch 'master' into odometry2
* more cleanup, make device_url configurable from launch file
* code cleanup
* interrupt endless loop of unsuccessful uploading
* fix uninitialized variable
* add takeoff blocking timeout
* upload whole mission at once, prevent loitering at waypoints
* Update micrortps agent topic names according to the new agent
* compensate altitude offset from home position
* add takeoff height tolerance param
* set mission waypoint after takeoff again
* prevent reuploading the same mission is start mission fails
* repeat the same point if mission upload fails
* clear mission before uploading a new one
* fix heading conversions
* add mutex for coord transform
* home position offset correction, takeoff update
* update mutex behavior
* fix formatting
* update mutexes
* set home from pixhawk telemetry
* switch to reentrant callback group
* propagate manual control to diagnostics
* change manual control flag handling
* add service to set waypoint acceptance radius directly from control
* improve takeoff, do not override manual mode
* publish desired pose again
* merge odometry into control
* Merge pull request `#14 <https://github.com/tiiuae/control_interface/issues/14>`_ from tiiuae/publish_desired_pose
  Publish desired pose

0.0.6 (2021-09-29)
-----------
* Requires fog_msgs 0.0.6
* remove hardcoded takeoff heading
* fix desired pose initialization
* publish desired pose
* add set float params for px4
* merge branch odom_pkg into global_parameters
* MavSDK parameter set and get changes
* update config
* config update, formatting
* merge global_params update to master
* README dependencies updated
* switch udp port back to 14590

0.0.5 (2021-09-06)
-----------
* version -> 0.0.5, updated diagnostics
* add heading control, continuous flying
* mavsdk -> 0.41.0, heading control
* add velocity param
* do not reupload mission with each waypoint
* fly through intermediate waypoints
* minor update
* added control_loop_rate parameter to config file
* dynamically change parameters
* Added services to change px4 parameters
* soften constrain for detection of landing -> now using ground_contact flag
* v0.1
* fixed typo
* enable octomap reset before takeoff
* Merge pull request `#11 <https://github.com/tiiuae/control_interface/issues/11>`_ from tiiuae/trigger_fog-drone_build
  trigger fog-drone build
* trigger fog-drone build
* Merge pull request `#10 <https://github.com/tiiuae/control_interface/issues/10>`_ from tiiuae/remove_pispatch_event
  remove repository dispatch events
* remove repository dispatch events
  Trigger builds only when repository is updated. Use git sha as build id
  for Artifactory builds.
* Merge pull request `#8 <https://github.com/tiiuae/control_interface/issues/8>`_ from tiiuae/reduce_takeoff_altitude
  1 meter of altitude is safer for indoor testing
* 1 meter of altitude is safer for indoor testing
* Fix initial waypoint x,y set right after takeoff
* odometry package changes
* Contributors: Esa Kulmala, Jan Bednar, Jari Nippula, Jukka Laitinen, Manuel Segarra-Abad, Vojtech Spurny, stibipet

0.0.3 (2021-06-21)
-----------
* Global coordinates control update (`#5 <https://github.com/tiiuae/control_interface/issues/5>`_)
  * add services providing global to local transformations
  * change mavsdk udp port back to 14590
  * change takeoff and landing message to trigger
* Merge pull request `#4 <https://github.com/tiiuae/control_interface/issues/4>`_ from tiiuae/ci_dispatch_event
  * add repository dispatch event
* Merge pull request `#3 <https://github.com/tiiuae/control_interface/issues/3>`_ from tiiuae/drone-integration
  * Fog drone integration
* Use system default QoS for subscribing px4 rtps topics
  * microRTPS topics are published with BEST_EFFORT reliability, so subscriber
  * needs to use the same qos settings. Easiest way is to use system default so stays in sync with publishers.
* Merge pull request `#2 <https://github.com/tiiuae/control_interface/issues/2>`_ from tiiuae/DP-852_ci_workflow
  * add CI workflow
* Make control_interface to terminate properly also in connection phase
* Use udp port 14590 defined for control_interface in fog_sw mavlink-router config
* Support node launch without tty
* Contributors: Esa Kulmala, Jari Nippula, sergey-unikie, stibipet

0.0.2 (2021-06-02)
-----------
* Robustness update (`#1 <https://github.com/tiiuae/control_interface/issues/1>`_)
  * check mission end directly
  * more robust commanding
  * replace mission clearing with mission pause -> avoid warning messages
  * mission progress checking
  * add diagnostics publisher
  * update formatting
* Contributors: Petr Stibinger

0.0.1 (2021-05-28)
------------------
