 ## dres_dvl_decode

This node is responsible for decoding DRES messages from the embedded system as
datatypes that other ros2 processes can use. Currently, the driver polls the
embedded system for data every 25ms for data. The return data is pushed to
the `"dvl_data"` node under the following float32 topics:

	`dvl_vel_x`
	`dvl_vel_y`
	`dvl_vel_z`
	`dvl_vel_e`
	`dvl_dist_bottom`
	`dvl_dist_1`
	`dvl_dist_2` 
	`dvl_dist_3` 
	`dvl_dist_4` 

 #### dependencies

As of now, this package requires the can2ros_driver package and scion_types to function.
