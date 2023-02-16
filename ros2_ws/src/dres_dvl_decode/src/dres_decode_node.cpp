/* Connor Larmer
 * 1.4.23
 * Decoder package for MS5837 Depth&Temp sensor.
 * filters and decodes data from the can2ros_driver
 * node in order to output nice float values to ros2. 
 */
#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/can_frame.hpp"
#include "dres_decode_node.hpp"

using namespace std;

/* DresDecodeNode class:
 * 	This was designed to be generic to allow for scalability.
 * 	The only "specific" parts of this code are the log messages,
 * 	node name, and decoder class.
 *
 */
DresDecodeNode::DresDecodeNode() : rclcpp::Node("dvl_data")
{
	/* create subscription to dres mailbox and bind it to our callback */
	_dres_mb = this->create_subscription<scion_types::msg::CanFrame>(
		"dres_mb_pub", 10, std::bind(&DresDecodeNode::decode_cb, this, std::placeholders::_1));
	_dreq_timer = this-> create_wall_timer(
		25ms,
		std::bind(&DresDecodeNode::_data_request, this));
	);
	
	dvl_decoder = new DVL::DVLDecode( (rclcpp::Node*)this);
	_poll_mb = new Mailbox::MboxCan(&ifr, "poll_mb");
	
	RCLCPP_INFO(this->get_logger(), "[DresDecodeNode] Node Initialized.");
}

/* The purpose of this callback is to filter device IDs.
 * in theory it is possible to have multiple decoders within
 * this method, however that would mean many topics under one node...
 * which is BAD and STUPID and UGLY. Prior to filtering, we create
 * a can_frame to pass into our decoder for the sake of sanity and simplicity.
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
	if(device == dvl_decoder->device_id)
	{
		RCLCPP_INFO(this->get_logger(),
			"[DresDecodeNode::decode_cb] DVL DRES is being processed.");
		dvl_decoder->dres_handler(&frame);
	}

}

/* 
 * _data_request:
 * We poll the DVL for frequently needed data. Instead of making a ros2
 * service call, we publish data directly to the CAN bus for speed/simplicity
 */
void DresDecodeNode::_data_request()
{
	struct can_frame poll_frame {0x020, 4, {WAYFDVL_DEV_ID, 0x00, 0x07, 0x00}};
	for(int i = 0; i < 6; i++)
	{
		Mailbox::MboxCan::write_mbox(_poll_mb, poll_frame);
	}
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DresDecodeNode>());
	rclcpp::shutdown();
	return 0;
}
