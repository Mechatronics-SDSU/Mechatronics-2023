#include "brlight.hpp"

using namespace Module;

BRLIGHTModule::BRLIGHTModule(rclcpp::Node* ctx, Mailbox::MboxCan* mb)
	: DeviceModule(ctx, mb, 0, static_cast<uint8_t>(CanDriver::Device::BRLIGHT::ID), false, true, 5)
{
	/* Member function jump table*/
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&BRLIGHTModule::dres_info);
	module_topic_ptr_array[1] = static_cast<topic_ptr_t>(&BRLIGHTModule::brlight_nop);
	module_topic_ptr_array[2] = static_cast<topic_ptr_t>(&BRLIGHTModule::brlight_nop);
	module_topic_ptr_array[3] = static_cast<topic_ptr_t>(&BRLIGHTModule::brlight_nop);
	module_topic_ptr_array[4] = static_cast<topic_ptr_t>(&BRLIGHTModule::brlight_nop);

	/* Send Wakeup/Enable frame */
	struct can_frame init_frame;
	memset(&init_frame, 0, sizeof(struct can_frame));
	init_frame.can_dlc = 5;
	init_frame.can_id = static_cast<uint8_t>(CanDriver::Command::STOW);
	init_frame.data[0] = this->module_device_id;
	init_frame.data[4] = 1;
	Mailbox::MboxCan::write_mbox(this->mailbox_ptr, &init_frame);	

	RCLCPP_INFO(node_context->get_logger(), "[BRLIGHTModule] Initialized.");
}

void BRLIGHTModule::timer_callback() {}
void BRLIGHTModule::mod_init() {}
void BRLIGHTModule::dres_handle(struct can_frame* frame)
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
			"[BRLIGHTModule::dres_handle] Topic 0x%04X does not exist, ignoring.",topic);
	}
}

void BRLIGHTModule::dres_info()
{
	memcpy(&module_hw_info, this->frame->data+(sizeof(char)*4), sizeof(float) );
	RCLCPP_INFO(node_context->get_logger(),
		"[BRLIGHTModule::dres_info] Received info from hw.");

}

void BRLIGHTModule::brlight_nop() { /* NO OPERATION */ }
