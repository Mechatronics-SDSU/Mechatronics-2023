#include "dvl_handler.hpp"
using namespace Dres::Handler;

/* Constructor method for DVL DRES decoder class;
 * Must pass in parent node upon initialization for topic generation.
 */
DVLHandler::DVLHandler(rclcpp::Node* context)
	: DresHandler(context, WAYFDVL_DEV_ID, 16)
{
	/* pointer-to-member-function weirdness dictates that each topic function is manually added*/
	dev_topics[0x0] = static_cast<topic_ptr_t>(&DVLHandler::dvl_nop);			/* 0000 NOP */
	dev_topics[0x1] = static_cast<topic_ptr_t>(&DVLHandler::dvl_vel_3);			/* 0001 3x Velocity */
	dev_topics[0x2] = static_cast<topic_ptr_t>(&DVLHandler::dvl_vel_x);			/* 0002 X velocity */
	dev_topics[0x3] = static_cast<topic_ptr_t>(&DVLHandler::dvl_vel_y);			/* 0003 Y velocity */
	dev_topics[0x4] = static_cast<topic_ptr_t>(&DVLHandler::dvl_vel_z);			/* 0004 Z Velocity */
	dev_topics[0x5] = static_cast<topic_ptr_t>(&DVLHandler::dvl_vel_e);			/* 0005 E Velocity */
	dev_topics[0x6] = static_cast<topic_ptr_t>(&DVLHandler::dvl_nop);			/* 0006 NOP */
	dev_topics[0x7] = static_cast<topic_ptr_t>(&DVLHandler::dvl_nop);			/* 0007 NOP */
	dev_topics[0x8] = static_cast<topic_ptr_t>(&DVLHandler::dvl_nop);			/* 0008 NOP */
	dev_topics[0x9] = static_cast<topic_ptr_t>(&DVLHandler::dvl_nop);			/* 0009 NOP */
	dev_topics[0xA] = static_cast<topic_ptr_t>(&DVLHandler::dvl_dist_bottom);	/* 000A Distance to bottom */
	dev_topics[0xB] = static_cast<topic_ptr_t>(&DVLHandler::dvl_dist_1);		/* 000B Beam 1 Distance */
	dev_topics[0xC] = static_cast<topic_ptr_t>(&DVLHandler::dvl_dist_2);		/* 000C Beam 2 Distance */
	dev_topics[0xD] = static_cast<topic_ptr_t>(&DVLHandler::dvl_dist_3);		/* 000D Beam 3 Distance */
	dev_topics[0xE] = static_cast<topic_ptr_t>(&DVLHandler::dvl_dist_4);		/* 000E Beam 4 Distance */
	dev_topics[0xF] = static_cast<topic_ptr_t>(&DVLHandler::dvl_nop);			/* 000F NOP */

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

	RCLCPP_INFO(node_context->get_logger(), "[DVLHandler] Handler Initialized.");
}

void DVLHandler::dres_handle(struct can_frame* frame)
{
	/* We copy the 16bit topic ID from the frame (EX: aaaaBBBBccccdddd -> we get BBBB) */
	uint16_t topic;
	dres_frame = frame;
	memcpy(&topic, dres_frame->data + ( sizeof(char) * 2 ), sizeof(char)*2 );
	if(topic < dev_topic_ct)
	{
		(this->*dev_topics[topic])(); 		/* we jump to correct topic function */
	}
	else
	{
		RCLCPP_INFO(node_context->get_logger(),
			"[DVLHandler::dres_handler] Topic 0x%04X does not exist, ignoring.",topic);
	}
}

void DVLHandler::dvl_nop() { /* NO OPERATION */}

void DVLHandler::dvl_vel_3()	{ /* Just a macro :: Delete Me! */}

void DVLHandler::dvl_vel_x() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_vel_x_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLHandler::dvl_vel_x] Published data : %f.",value);
}

void DVLHandler::dvl_vel_y() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_vel_y_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"DVLHandler::dvl_vel_y] Published data : %f.",value);
}

void DVLHandler::dvl_vel_z() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_vel_z_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLHandler::dvl_vel_z] Published data : %f.",value);
}

void DVLHandler::dvl_vel_e() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_vel_e_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLHandler::dvl_vel_e] Published data : %f.",value);
}

void DVLHandler::dvl_dist_bottom() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_bottom_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLHandler::dvl_dist_bottom] Published data : %f.",value);
}

void DVLHandler::dvl_dist_1() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_1_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLHandler::dvl_dist_1] Published data : %f.",value);
}

void DVLHandler::dvl_dist_2() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_2_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLHandler::dvl_dist_2] Published data : %f.",value);
}

void DVLHandler::dvl_dist_3() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_3_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLHandler::dvl_dist_3] Published data : %f.",value);
}

void DVLHandler::dvl_dist_4() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	dvl_dist_4_topic->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLHandler::dvl_dist_4] Published data : %f.",value);
}


