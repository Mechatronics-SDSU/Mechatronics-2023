/* Connor Larmer
 * 1.4.23
 * Decoder package for MS5837 Depth&Temp sensor.
 * filters and decodes data from the can2ros_driver
 * node in order to output nice float values to ros2. 
 */
#include "rclcpp/rclcpp.hpp"
#include "can_msg_interfaces/msg/can_frame.hpp"
#include "dres_decode_node.hpp"

using namespace std;

/* DresDecodeNode class:
 * 	This was designed to be generic to allow for scalability.
 * 	The only "specific" parts of this code are the log messages,
 * 	node name, and decoder class.
 *
 */
DresDecodeNode::DresDecodeNode() : rclcpp::Node("ms5837_data")
{
	/* create subscription to dres mailbox and bind it to our callback */
	_dres_mb = this->create_subscription<can_msg_interfaces::msg::CanFrame>(
		"dres_mb_pub", 10, std::bind(&DresDecodeNode::decode_cb, this, std::placeholders::_1));

	ms5837_decoder = new MS5837::MS5837Decode( (rclcpp::Node*)this);
	RCLCPP_INFO(this->get_logger(), "[DresDecodeNode] Node Initialized.");
}

/* The purpose of this callback is to filter device IDs.
 * in theory it is possible to have multiple decoders within
 * this method, however that would mean many topics under one node...
 * which is BAD and STUPID and UGLY. Prior to filtering, we create
 * a can_frame to pass into our decoder for the sake of sanity and simplicity.
 */
void DresDecodeNode::decode_cb(
	const can_msg_interfaces::msg::CanFrame::SharedPtr msg) const
{
	uint16_t device;
	struct can_frame frame;
	frame.can_id = msg->can_id;
	std::copy(
		std::begin(msg->can_data),
		std::end(msg->can_data),
		std::begin(frame.data));
	
	memcpy(&device, frame.data, sizeof(char)*2);
	if(device == ms5837_decoder->device_id)
	{
		RCLCPP_INFO(this->get_logger(),
			"[DresDecodeNode::decode_cb] MS5837 DRES is being processed.");
		ms5837_decoder->dres_handler(&frame);
	}

}


int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DresDecodeNode>());
	rclcpp::shutdown();
	return 0;
}