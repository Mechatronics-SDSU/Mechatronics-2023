# Scion CAN2ROS2 Driver

*also known as unified_can_driver*

***

## Introduction

This ROS2 package is the driver for CAN communication between the Nvidia Jetson Orin and Teensy 4.1 Embedded node. It is built to be modular, allowing for new 'device modules' to be quickly developed and deployed as Scion's sensor suite expands. The only dependency this package requires (other than ROS2...) is the `scion_types` ROS2 package for it's custom msgs. The driver can be configured to use a different CAN interface, control embedded update polling, and enable/disable certain modules without recompiling via a launch file. More information regarding modules, published topcis, services, and user configurable fields can be found below.

***

## Table of Contents

 1. [Introduction](#Introduction)
 2. [Building](#Building)
 3. [Configuration](#Configuration)
 4. [Services](#Services)
 5. [Modules](#Modules)
 6. [Development](#Development)

***

## Building

In order to build the driver as a ROS2 package, first verify the following:

 - ROS2 Foxy is installed
 - The `scion_types` package is in the same workspace as `unified_can_driver` package
 - You have sourced your setup.sh (found in `/opt/ros/foxy/setup.sh` or your workspace)

### Build steps

 1. In your workspace, first build `scion_types` (`colcon build --packages-select scion_types`)
 2. Source your local setup.sh generate from the previous step (`source install/setup.sh`)
 3. Build `unified_can_driver` (`colcon build --packages-select unified_can_driver`)
 4. Source your locak setup.sh once more.
 5. The driver can now be launched with `ros2 run unified_can_driver unified_can_driver`
 
***

## Configuration

The driver's parameters can be configured either through the command-line or a launch file, see below.

### Command Line

There are three parameters that determine the way that the driver operates:

 - `can_bus_interface` A string representing the desired interface (i.e: can0, can1, etc.) the driver will connect to.
 - `do_module_polling` A boolean determining whether modules will automatically poll their embedded counterpart for new data.
 - `module_bitfield` A bitfield determining which module(s) should be enabled/disabled at startup, see below for more information.

In order to configure the driver via the command-line, simply use the following format:\
`ros2 run unified_can_driver unified_can_driver --ros-args -p [parameter]:=[option] ... ...`\
I.E: `ros2 run unified_can_driver unified_can_driver --ros-args -p can_bus_interface:="can0" -p do_module_polling:=false -p module_bitfield:=255`\

### Launch File

***

## Services

***

## Modules

***

## Development