# HW Pixhawk <--> ROS2 setup
* Install dependencies using `01 and 02 scripts`[https://mrs.felk.cvut.cz/gitlab/ros2/simulation2/tree/master/installation]
* Compile firmware with RTPS bridge
```bash
make px4_fmu-v5_rtps
````
* If compilation fails because of Nuttx `config.h` or `version.h`, comment out all version checks in `/px4_firmware/build/px4_fmu-v5_rtps/NuttX/nuttx/tools/version.sh`

* Upload firmware to Pixhawk using USB cable and QGroundControl

* SET UP SAFETY IN QGROUNDCONTROL!

* Change the content of the Pixhawk SD card `etc/extras.txt` to start `micrortps_client`[https://mrs.felk.cvut.cz/gitlab/ros2/control_interface/miscellaneous/etc/extras.txt]

* Run `micrortps_agent` on the onboard computer:

```bash
micrortps_agent -d /dev/ttyS7 -b 921600
````

* Killing the agent will show the number of outgoing and incoming messages

* Make sure that Pixhawk outputs are visible as ROS2 messages in `ros2 topic list`

# Known issues
* `micrortps_agent` occasionally crashes when reading nested messages over serial line (real HW)

# Troubleshooting
If the pixhawk topics are not visible in `ros2 topic list`:
* Check that `[micrortps_client]` is started on pixhawk side
* Check that `[micrortps_agent]` is started on computer side
* Do not export `ROS_DOMAIN_ID` in `.bashrc`!
* On real HW, make sure that the baudrate is matched and correct device is used

