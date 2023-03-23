#include "brping1.hpp"

using namespace Module;

BRPING1Module::BRPING1Module(rclcpp::Node* ctx, Mailbox::MboxCan* mb)
	: DeviceModule(ctx, mb, 50, static_cast<uint8_t>(CanDriver::Device::BRPING1::ID), true, true, 5)
{
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&BRPING1Module::dres_info);
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&BRPING1Module::brping1_nop);
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&BRPING1Module::brping1_nop);
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&BRPING1Module::brping1_nop);
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&BRPING1Module::brping1_dist_short);

	topic_dist_short = node_context->create_publisher<scion_types::msg::Datapoint> ("brping1_dist", 10);
	topic_confidence = node_context->create_publisher<scion_types::msg::Datapoint> ("brping1_confidence", 10);

	/* Send Wakeup Frame */
	struct can_frame init_frame;
	memset(&init_frame, 0, sizeof(struct can_frame));
	init_frame.can_dlc = 4;
	init_frame.can_id = static_cast<uint8_t>(CanDriver::Command::STOW);
	init_frame.data[0] = this->module_device_id;
	Mailbox::MboxCan::write_mbox(this->mailbox_ptr, &init_frame);

	RCLCPP_INFO(node_context->get_logger(), "[BRPING1Module] Initialized.");
}

void BRPING1Module::mod_init() {}
void BRPING1Module::dres_handle(struct can_frame* frame)
{
	/* We copy the 16bit topic ID from the frame (EX: aaaaBBBBccccdddd -> we get BBBB) */
	uint16_t topic;
	this->frame = frame;
	memcpy(&topic, this->frame->data + ( sizeof(char) * 2 ), sizeof(char)*2 );
	if(topic < module_topic_ct)
	{
		(this->*module_topic_ptr_array[topic])(); 		/* we jump to correct topic function */
	}
	else
	{
		RCLCPP_INFO(node_context->get_logger(),
			"[BRPING1Module::dres_handle] Topic 0x%04X does not exist, ignoring.",topic);
	}
}
void BRPING1Module::timer_callback()
{
	if(do_device_polling && this->module_enable_ti)
	{
		struct can_frame poll_frame;
		memset(&poll_frame, 0, sizeof(struct can_frame));
		poll_frame.can_id = static_cast<uint8_t>(CanDriver::Command::DREQ);
		poll_frame.can_dlc = 4;
		poll_frame.data[0] = this->module_device_id;
		poll_frame.data[2] = static_cast<uint8_t>(CanDriver::Device::BRPING1::P1D_DIST_SHORT);
		Mailbox::MboxCan::write_mbox(mailbox_ptr, &poll_frame);
	}
	
}
void BRPING1Module::dres_info()
{
	memcpy(&module_hw_info, this->frame->data+(sizeof(char)*4), sizeof(float) );
	RCLCPP_INFO(node_context->get_logger(),
		"[BRPING1Module::dres_info] Received info from hw.");
}

void BRPING1Module::brping1_nop() {}
void BRPING1Module::brping1_dist_short()
{
	float distance, confidence;
	memcpy(&distance, this->frame->data + (sizeof(char)*4), sizeof(uint16_t) );
	memcpy(&confidence, this->frame->data + (sizeof(char)*2), sizeof(uint8_t) );
	auto dist_msg = scion_types::msg::Datapoint();
	auto conf_msg = scion_types::msg::Datapoint();
	dist_msg.data = distance;
	conf_msg.data = confidence;
	topic_dist_short->publish(dist_msg);
	topic_confidence->publish(conf_msg);

	RCLCPP_INFO(node_context->get_logger(),
		"[BRPING1Module::brping1_dist_short] Published Distance : %f", distance);
	RCLCPP_INFO(node_context->get_logger(),
		"[BRPING1Module::brping1_dist_short] Published Confidence : %f", confidence);
}
