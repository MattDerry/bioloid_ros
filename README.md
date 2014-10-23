bioloid_ros
===========

This is a set of tools and ros packages for implementing wireless control of a Bioloid Premium robot kit using ROS. It was initially developed to support the EECS 295 Introduction to Robotics Lab class at Northwestern University.

Prerequisites
-------------
1. ROS (hydro or newer)
2. Bioloid kit with CM-530 microcontroller
3. Either the zigbee or BT-210 bluetooth module from Robotis
4. Bluetooth on the PC

Components
----------
1. serial library (a serial library used by the bioloid_ros_driver)
2. bioloid_ros_firmware (c project requiring cross-compilation toolchain for ARM Cortex-M3 microprocessor)
3. bioloid_ros_driver (a ROS package with a driver node providing a ROS service interface to communicate with the firmware on the bioloid)
4. bioloid_ros_examples (a ROS package with example control nodes)

Installation
------------

To do...

Usage
-----

To do...
