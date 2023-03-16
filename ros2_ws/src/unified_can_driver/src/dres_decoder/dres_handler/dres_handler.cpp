#include "dres_handler.hpp"

using namespace Dres::Handler;

DresHandler::DresHandler(rclcpp::Node* node,
	uint16_t device,
	uint8_t topic_ct)
{
	node_context = node;
	device_id = device;
	dev_topic_ct = topic_ct;
	dev_topics = new topic_ptr_t[dev_topic_ct];
}
DresHandler::~DresHandler()
{
	delete[] dev_topics;
}	

void DresHandler::dres_handle(struct can_frame* frame)
{
	/* Dummy implementation*/
	RCLCPP_INFO(node_context->get_logger(), "[DresHandler::dres_handle] Parent Handler Called :(.");
}