dumbo_control
=============
Overview
---------------------------------------------
This metapackage contains [Dumbo's][1] low level control and hardware interface of Schunk arms + parallel gripper and force-torque sensors following the [ros_control][2] framework. 

[1]: https://cvapwiki.csc.kth.se/dokuwiki/doku.php?id=dumbo
[2]: http://wiki.ros.org/ros_control

Installation
---------------------------------------------

Make sure you have **sudo rights!**. The (soft-) low level realtime hardware control loop requires sudo rights to be able to run the control thread with a **realtime scheduler**. 

To be able to run the hw control loop you need to modify the **sudoers** file.

First run the following command to edit the sudoers file:

<code>
  sudo visudo
</code>

Then add the following line at the end of the file:

<code>
    **user_name** ALL = NOPASSWD: /home/**user_name**/catkin_ws/src/dumbo_control/dumbo_hw_control_loop/scripts/run_hw_control_loop

</code>

Where **user_name**  is your user name. 


dumbo_hardware_interface
---------------------------------------------
This package contains the hardware interface to several of Dumbo's components:
* Schunk arms
* Schunk parallel gripper (PG70)
* Force-torque sensors

dumbo_hw_control_loop
---------------------------------------------
This package contains the (soft) realtime control loop that has access to Dumbo's HW components described in the **dumbo_hardware_interface** package. Controllers can be dynamically loaded to the [controller manager][3] that runs in this loop. 

This low-level control loop has been designed according to the **ros_control** specifications. Please look at the documentation of the [ros_control][4] package in the ROS wiki as well as the wiki in the package's [github repository][5].

[3]: http://wiki.ros.org/controller_manager
[4]: http://wiki.ros.org/ros_control
[5]: https://github.com/ros-controls/ros_control
