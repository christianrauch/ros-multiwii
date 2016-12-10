# ROS node for MultiWii / Cleanflight flight controllers

This ROS nodes uses the MultiWii Serial Protocol (MSP) to communicate with [MultiWii](http://www.multiwii.com/wiki/index.php?title=Main_Page) and [Cleanflight](http://cleanflight.com/) based flight controllers.

It reuses many of the [mavros](http://wiki.ros.org/mavros) message definitions and topics and is therefore compatible to some visualisation and control nodes (e.g. joystick teleoperation) that are part of the [mavros_extras](http://wiki.ros.org/mavros_extras) package.
The current implementation supports control over RC channels or direct motor access (active DYNBALANCE), arming/disarming by a service and it publishes IMU values and current RC/motor states.
