^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package control_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
