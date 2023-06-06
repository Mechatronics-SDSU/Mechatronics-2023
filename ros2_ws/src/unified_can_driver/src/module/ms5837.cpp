#include "ms5837.hpp"

using namespace Module;

MS5837Module::MS5837Module(rclcpp::Node* ctx, Mailbox::MboxCan* mb)
	: DeviceModule(ctx, mb, 40, static_cast<uint16_t>(CanDriver::Device::MS5837::ID), true, true, 4)
{
	/* pointer-to-member-function weirdness dictates that each topic function is manually added*/
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&MS5837Module::dres_info);			/* 0000 INFO */
	module_topic_ptr_array[1] = static_cast<topic_ptr_t>(&MS5837Module::ms5837_nop);			/* 0001 DATA MACRO */
	module_topic_ptr_array[2] = static_cast<topic_ptr_t>(&MS5837Module::ms5837_depth);			/* 0002 DEPTH */
	module_topic_ptr_array[3] = static_cast<topic_ptr_t>(&MS5837Module::ms5837_temp);			/* 0003 TEMPERATURE */

	sensor_depth = node_context->create_publisher<scion_types::msg::Datapoint> ("ms5837_depth", 10);
	sensor_temp  = node_context->create_publisher<scion_types::msg::Datapoint> ("ms5837_temp", 10);

	/* Send Wakeup Frame */
	struct can_frame init_frame;
	memset(&init_frame, 0, sizeof(struct can_frame));
	init_frame.can_dlc = 5;
	init_frame.can_id = static_cast<uint8_t>(CanDriver::Command::STOW);
	init_frame.data[0] = this->module_device_id;
	init_frame.data[4] = 1;
	Mailbox::MboxCan::write_mbox(this->mailbox_ptr, &init_frame);

	RCLCPP_INFO(node_context->get_logger(), "[MS5837Module] Initialized.");
}

void MS5837Module::timer_callback()
{
	if(do_device_polling && this->module_enable_ti)
	{
		struct can_frame poll_frame;
		memset(&poll_frame, 0, sizeof(struct can_frame));
		poll_frame.can_id = static_cast<uint8_t>(CanDriver::Command::DREQ);
		poll_frame.can_dlc = 4;
		poll_frame.data[0] = this->module_device_id;
		poll_frame.data[2] = static_cast<uint16_t>(CanDriver::Device::MS5837::TOPIC_DATA);
		Mailbox::MboxCan::write_mbox(mailbox_ptr, &poll_frame);
	}
}

void MS5837Module::dres_handle(struct can_frame* frame)
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
			"[MS5837Module::dres_handle] Topic 0x%04X does not exist, ignoring.",topic);
	}
}

void MS5837Module::mod_init() { /* UNUSED */ }

void MS5837Module::dres_info()
{
	memcpy(&module_hw_info, this->frame->data+(sizeof(char)*4), sizeof(float) );
	RCLCPP_INFO(node_context->get_logger(),
		"[MS5837Module::dres_info] Received info from hw.");

}

void MS5837Module::ms5837_nop() { /* NO OPERATION */}

void MS5837Module::ms5837_depth() 
{
	/* We copy the last 32 bits of the frame into a float, then publish
	   it to the correct ROS2 topic */
	float depth;
	memcpy(&depth, this->frame->data+(sizeof(char)*4), sizeof(float) );

	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = depth;
	sensor_depth->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[MS5837Module::ms5837_depth] Published data : %f.",depth);

}

void MS5837Module::ms5837_temp()
{
	/* We copy the last 32 bits of the frame into a float, then publish
	   it to the correct ROS2 topic */
	float temp;
	memcpy(&temp, this->frame->data+(sizeof(char)*4), sizeof(float) );

	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = temp;
	sensor_temp->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[MS5837Module::ms5837_temp] Published data : %f.",temp);

}
