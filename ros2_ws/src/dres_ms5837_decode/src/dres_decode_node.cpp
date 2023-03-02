/* Connor Larmer
 * 1.4.23
 * Decoder package for MS5837 Depth&Temp sensor.
 * filters and decodes data from the can2ros_driver
 * node in order to output nice float values to ros2. 
 */
#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/can_frame.hpp"
#include "dres_decode_node.hpp"

#define MBOX_INTERFACE "can0"

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
	_dres_mb = this->create_subscription<scion_types::msg::CanFrame>(
		"dres_mb_pub", 10, std::bind(&DresDecodeNode::decode_cb, this, std::placeholders::_1));

	_dreq_timer = this-> create_wall_timer(
		250ms,
		std::bind(&DresDecodeNode::_data_request, this));
	
	ms5837_decoder = new MS5837::MS5837Decode( (rclcpp::Node*)this);

	strncpy(ifr.ifr_name, MBOX_INTERFACE, sizeof(&MBOX_INTERFACE));
	_poll_mb = new Mailbox::MboxCan(&ifr, "poll_mb");
	
	RCLCPP_INFO(this->get_logger(), "[DresDecodeNode] Node Initialized.");
}

/* The purpose of this callback is to filter device IDs.
 * in theory it is possible to have multiple decoders within
 * this method, however that would mean many topics under one node...
 * which is BAD and STUPID and UGLY. Prior to filtering, we create
 * a can_frame to pass into our decoder for the sake of sanity and simplicity.
 */


/* 
 * Zix - Looked at other Connor's code for the dvl_decode and tried it to implement it for the pressure sensor. 
 * Looks like it's working for depth as well now that I made some changes. ms5837 device ID is 0x0003
 */
void DresDecodeNode::decode_cb(
	const scion_types::msg::CanFrame::SharedPtr msg) const
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
		ms5837_decoder->dres_handler(&frame);
	}

}

/* 
 * Send to CAN with ID 020# and data as device and subtopic
 * Device is ms5837 0x0003
 * Subtopic is depth which is 0x0002 
 * Write in little endian so 0300|0200
 */
void DresDecodeNode::_data_request()
{
	char depth_dreq_frame[4] = {0x03, 0x00, 0x02, 0x00};
	struct can_frame poll_frame;
	poll_frame.can_id = 0x020;
	poll_frame.can_dlc = 4;
	std::copy(std::begin(depth_dreq_frame),
			std::end(depth_dreq_frame),
			std::begin(poll_frame.data));
	if(Mailbox::MboxCan::write_mbox(_poll_mb, &poll_frame) == -1)
	{
		RCLCPP_INFO(this->get_logger(),
			"[DresDecodeNode::_data_request] Failed to write DREQ.");
	}
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DresDecodeNode>());
	rclcpp::shutdown();
	return 0;
}
