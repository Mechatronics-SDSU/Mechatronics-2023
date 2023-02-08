# Notes regarding sending CAN frames to the can2ros_driver

Currently, there is a header file within ros2ws/src/dres_ms5837_decode/include/ titled `device_macros.hpp`.
It contains definitions of macros for EMBEDSYS, PWRSYS, WAYFDVL, and MS5837. Although it is a very simple implementation,
it is up to date (as of Feb 7, 2023). Read the header file comments for more information. The can_send_service is defined
within the ros2ws/src/can_msg_interfaces/srv/ folder and can be used to create a client (see ros2 client service example).

this is a stub, more info will be added later.
