
# Junebug CAN to ROS2 Pipeline
(Updated 1.4.23)

Status for can2ros2 driver:
| Feature | Python | CPP |
| ----- | ----- | ----- |
| Read    | X      | X |
| Write   | X      | X |
| ROS2 READ    | X      | X |
| ROS2 WRITE | | X | 
| Filter/Mailbox |  | X |

Status for embedded systems decoders:
| device | status |
| ----- | ----- |
| MS5837 | X |
| EMBEDSYS |  |
| PWRSYS |  |
| WAYFDVL |  |

## Building the Ros2 Packages

*Please note that the default interface is specified
within the can2ros_driver/src/can_mbox_pub.cpp file as "vcan0".
You must change this to "can0" in order to function on the Orin.*

1. Source Ros2 (or whatever setup.sh you're using)
`source /opt/ros/foxy/setup.sh` 
2. Navigate to the correct workspace
`cd ros2can_workspace/`
3. Build the "can_msg_interfaces" (it is a dependency and should be built first)
`colcon build --packages-select can_msg_interfaces`
4. Source the file with the newly built binary
`source install/setup.sh`
5. Build the CAN2ROS2 driver
`colcon build --packages-select can2ros_driver`
6. Re-source install/setup.sh
7. Build the MS5837 Decoder package
`colcon build --packages-select dres_ms5837_decode`
7. Run the package(s)
`ros2 run can2ros_driver can2ros_driver`
`ros2 run dres_ms5837_decode dres_ms5837_decode`

## Current Proposed CAN Protocol (Embedded <-> Orin)

* https://docs.google.com/spreadsheets/d/1Z7OvEd_o8Brstv6O6mNKoWZZ86YrVBjY30yL9ZncIfY/edit#gid=0
.
