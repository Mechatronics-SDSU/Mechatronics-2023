 ## can2ros_driver

This ros2 node acts as a pipe in-between the CAN bus (hardware) and Scion's
Ros2 network (software). Essentially, CAN frames are filtered and read by the driver,
then outputted into Ros2 subtopiccs (based on filtering). As of right now, the only
mailbox actively in use is "dres_mb_pub". For this reason, the base can2ros_driver and
dres_(device)_decoders will be merged into one package within the foreseeable future.


 ### services & messages

The base driver utilizes two custom types (found in 'scion_types'): msg/CanFrame and
srv/SendFrame. More information is found below.

 #### msg/CanFrame

This message is used by the base driver (and decoder nodes) as a means of transferring a
single frame between ros2 processes. It is structured similarly to that of the `struct can_frame`
structure founnd in linux/can.h. As a result, it is trivial to copy the contents of a ros2 CanFrame
to linux's can_frame structure. This message will mainly be used between driver components, as other
processes should have little need to decode their own can frames.

 #### srv/SendFrame

This service is the main way for outside ros2 processes to push data onto the CAN bus. It is structured
similarly to the can_frame implementation within the linux CAN driver. The implementation of this service
call is similar to any other service call (examples can be found in ros2 docs) The basic structure is as followed:

	int32_t can_id 		(usually expressed as 0xXXX)
	int8_t	can_dlc		(number of bytes being sent, 0-8)
	char\[8\] can_data	(bytearray containing message data)
