#include "dvl.hpp"

using namespace Module;

DVLModule::DVLModule(rclcpp::Node* ctx, Mailbox::MboxCan* mb)
	: DeviceModule(ctx, mb, 10, static_cast<uint8_t>(CanDriver::Device::WAYFDVL::ID), true, true, 16)
{
	/* pointer-to-member-function weirdness dictates that each topic function is manually added*/
	module_topic_ptr_array[0x0] = static_cast<topic_ptr_t>(&DVLModule::dres_info);			/* 0000 NOP */
	module_topic_ptr_array[0x1] = static_cast<topic_ptr_t>(&DVLModule::dvl_vel_3);			/* 0001 3x Velocity */
	module_topic_ptr_array[0x2] = static_cast<topic_ptr_t>(&DVLModule::dvl_vel_x);			/* 0002 X velocity */
	module_topic_ptr_array[0x3] = static_cast<topic_ptr_t>(&DVLModule::dvl_vel_y);			/* 0003 Y velocity */
	module_topic_ptr_array[0x4] = static_cast<topic_ptr_t>(&DVLModule::dvl_vel_z);			/* 0004 Z Velocity */
	module_topic_ptr_array[0x5] = static_cast<topic_ptr_t>(&DVLModule::dvl_vel_e);			/* 0005 E Velocity */
	module_topic_ptr_array[0x6] = static_cast<topic_ptr_t>(&DVLModule::dvl_nop);			/* 0006 NOP */
	module_topic_ptr_array[0x7] = static_cast<topic_ptr_t>(&DVLModule::dvl_nop);			/* 0007 NOP */
	module_topic_ptr_array[0x8] = static_cast<topic_ptr_t>(&DVLModule::dvl_nop);			/* 0008 NOP */
	module_topic_ptr_array[0x9] = static_cast<topic_ptr_t>(&DVLModule::dvl_nop);			/* 0009 NOP */
	module_topic_ptr_array[0xA] = static_cast<topic_ptr_t>(&DVLModule::dvl_dist_bottom);	/* 000A Distance to bottom */
	module_topic_ptr_array[0xB] = static_cast<topic_ptr_t>(&DVLModule::dvl_dist_1);			/* 000B Beam 1 Distance */
	module_topic_ptr_array[0xC] = static_cast<topic_ptr_t>(&DVLModule::dvl_dist_2);			/* 000C Beam 2 Distance */
	module_topic_ptr_array[0xD] = static_cast<topic_ptr_t>(&DVLModule::dvl_dist_3);			/* 000D Beam 3 Distance */
	module_topic_ptr_array[0xE] = static_cast<topic_ptr_t>(&DVLModule::dvl_dist_4);			/* 000E Beam 4 Distance */
	module_topic_ptr_array[0xF] = static_cast<topic_ptr_t>(&DVLModule::dvl_nop);			/* 000F NOP */

	/* initialize topics*/
	dvl_vel_x_topic = node_context->create_publisher<scion_types::msg::Datapoint>("dvl_vel_x", 10);
	dvl_vel_y_topic = node_context->create_publisher<scion_types::msg::Datapoint> ("dvl_vel_y", 10);
	dvl_vel_z_topic = node_context->create_publisher<scion_types::msg::Datapoint> ("dvl_vel_z", 10);
	dvl_vel_e_topic = node_context->create_publisher<scion_types::msg::Datapoint> ("dvl_vel_e", 10);
	dvl_dist_bottom_topic = node_context->create_publisher<scion_types::msg::Datapoint> ("dvl_dist_bottom", 10);
	dvl_dist_1_topic = node_context->create_publisher<scion_types::msg::Datapoint> ("dvl_dist_1", 10);
	dvl_dist_2_topic = node_context->create_publisher<scion_types::msg::Datapoint> ("dvl_dist_2", 10);
	dvl_dist_3_topic = node_context->create_publisher<scion_types::msg::Datapoint> ("dvl_dist_3", 10);
	dvl_dist_4_topic = node_context->create_publisher<scion_types::msg::Datapoint> ("dvl_dist_4", 10);

	/* Send Wakeup Frame */
	struct can_frame init_frame;
	memset(&init_frame, 0, sizeof(struct can_frame));
	init_frame.can_dlc = 4;
	init_frame.can_id = static_cast<uint8_t>(CanDriver::Command::STOW);
	init_frame.data[0] = this->module_device_id;
	Mailbox::MboxCan::write_mbox(this->mailbox_ptr, &init_frame);

	RCLCPP_INFO(node_context->get_logger(), "[DVLModule] Initialized.");
}

void DVLModule::mod_init() {/*UNUSED*/}
void DVLModule::dres_handle(struct can_frame* frame)
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
			"[DVLModule::dres_handle] Topic 0x%04X does not exist, ignoring.",topic);
	}
}

void DVLModule::timer_callback()
{
	if(do_device_polling && this->module_enable_ti)
	{
		struct can_frame poll_frame;
		memset(&poll_frame, 0, sizeof(struct can_frame));
		poll_frame.can_id = static_cast<uint8_t>(CanDriver::Command::DREQ);
		poll_frame.can_dlc = 4;
		poll_frame.data[0] = this->module_device_id;
		poll_frame.data[2] = static_cast<uint8_t>(CanDriver::Device::WAYFDVL::TOPIC_VEL_AND_DIST);
		Mailbox::MboxCan::write_mbox(mailbox_ptr, &poll_frame);
	}
}

void DVLModule::dres_info()
{
	memcpy(&module_hw_info, this->frame->data+(sizeof(char)*4), sizeof(float) );
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dres_info] Received info from hw.");

}

/********* TOPIC METHODS *********/
void DVLModule::dvl_nop() { /* NO OPERATION */}
void DVLModule::dvl_vel_3()	{ /* UNUSED */}

void DVLModule::dvl_vel_x() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_vel_x_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dvl_vel_x] Published data : %f.",value);
}

void DVLModule::dvl_vel_y() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_vel_y_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"DVLModule::dvl_vel_y] Published data : %f.",value);
}

void DVLModule::dvl_vel_z() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_vel_z_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dvl_vel_z] Published data : %f.",value);
}

void DVLModule::dvl_vel_e() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_vel_e_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dvl_vel_e] Published data : %f.",value);
}

void DVLModule::dvl_dist_bottom() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_bottom_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dvl_dist_bottom] Published data : %f.",value);
}

void DVLModule::dvl_dist_1() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_1_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dvl_dist_1] Published data : %f.",value);
}

void DVLModule::dvl_dist_2() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_2_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dvl_dist_2] Published data : %f.",value);
}

void DVLModule::dvl_dist_3() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_3_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dvl_dist_3] Published data : %f.",value);
}

void DVLModule::dvl_dist_4() 
{
	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_4_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLModule::dvl_dist_4] Published data : %f.",value);
}
