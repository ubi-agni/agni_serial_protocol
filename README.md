## Overview

The AGNI Serial Protocol aims to unify firmware and driver software for typical serial communication devices developed at the Neuroinformatics Grooup of Bielefeld University.
The central idea is to use a common protocol along with reusable implementations of it across all devices.
Detailed information can be found in our [internal wiki](https://projects.cit-ec.uni-bielefeld.de/projects/agni-grasplab/wiki/Serial_Protocol_Devices).

This repository hosts the PC-side software ("driver"). It is a ROS node,
- recognizing the device through an initial configuration request, 
- instantiating an appropriate parser, and 
- publishing suitable sensor messages

Devices must get registered so that a unique device corresponds to a unique parser on the host side.

## Usage

`roslaunch agni_serial_protocol sp_to_ros.launch device:=/dev/ttyACM0`
