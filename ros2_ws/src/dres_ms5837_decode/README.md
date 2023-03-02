 ## dres_ms5837_decode

This node is responsible for decoding DRES messages from the embedded system as
datatypes that other ros2 processes can use. Currently, the driver polls the
embedded system for data every 40ms for depth and temperature. the return data
is pushed to ros2 as two float32 topics labeled `"ms5837_depth"` and `"ms5837_temp"`
within the `"ms5837_data"` node.

 #### dependencies

As of now, this package requires the can2ros_driver package and scion_types to function.
