#include "dvl_handler.hpp"

using namespace DVL;

/* Constructor method for MS5837 DRES decoder class;
 * Must pass in parent node upon initialization for topic generation.
 */
DVLDecode:: DVLDecode(rclcpp::Node* context)
{
	node_context = context;

	/* pointer-to-member-function weirdness dictates that each topic function is manually added*/
	dvl_topics[0x0] = &DVLDecode::dvl_nop;			/* 0000 NOP */
	dvl_topics[0x1] = &DVLDecode::dvl_vel_3;		/* 0001 3x Velocity */
	dvl_topics[0x2] = &DVLDecode::dvl_vel_x;		/* 0002 X velocity */
	dvl_topics[0x3] = &DVLDecode::dvl_vel_y;		/* 0003 Y velocity */
	dvl_topics[0x4] = &DVLDecode::dvl_vel_z;		/* 0004 Z Velocity */
	dvl_topics[0x5] = &DVLDecode::dvl_vel_e;		/* 0005 E Velocity */
	dvl_topics[0x6] = &DVLDecode::dvl_nop;			/* 0006 NOP */
	dvl_topics[0x7] = &DVLDecode::dvl_nop;			/* 0007 NOP */
	dvl_topics[0x8] = &DVLDecode::dvl_nop;			/* 0008 NOP */
	dvl_topics[0x9] = &DVLDecode::dvl_nop;			/* 0009 NOP */
	dvl_topics[0xA] = &DVLDecode::dvl_dist_bottom;	/* 000A Distance to bottom */
	dvl_topics[0xB] = &DVLDecode::dvl_dist_1;		/* 000B Beam 1 Distance */
	dvl_topics[0xC] = &DVLDecode::dvl_dist_2;		/* 000C Beam 2 Distance */
	dvl_topics[0xD] = &DVLDecode::dvl_dist_3;		/* 000D Beam 3 Distance */
	dvl_topics[0xE] = &DVLDecode::dvl_dist_4;		/* 000E Beam 4 Distance */
	dvl_topics[0xF] = &DVLDecode::dvl_nop;			/* 000F NOP */

	device_id = WAYFDVL_DEV_ID; 		/* public device ID (used by dres_decoder_node) */

	/* initialize topics*/
	dvl_vel_x = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_vel_x", 10);
	dvl_vel_y = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_vel_y", 10);
	dvl_vel_z = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_vel_z", 10);
	dvl_vel_e = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_vel_e", 10);
	dvl_dist_bottom = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_dist_bottom", 10);
	dvl_dist_1 = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_dist_1", 10);
	dvl_dist_2 = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_dist_2", 10);
	dvl_dist_3 = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_dist_3", 10);
	dvl_dist_4 = node_context->create_publisher<std_msgs::msg::Float32> ("dvl_dist_4", 10);

	RCLCPP_INFO(node_context->get_logger(), "[DVLDecode] Decoder Initialized.");
}

void DVLDecode::dres_handler(struct can_frame* frame) 
{
	/* We copy the 16bit topic ID from the frame (EX: aaaaBBBBccccdddd -> we get BBBB) */
	uint16_t topic;
	dres_frame = frame;
	memcpy(&topic, dres_frame->data + ( sizeof(char) * 2 ), sizeof(char)*2 );
	if(topic < dvl_topic_ct)
	{
		(this->*dvl_topics[topic])(); 		/* we jump to correct topic function */
	}
	else
	{
		RCLCPP_INFO(node_context->get_logger(),
			"[DVLDecode::dres_handler] Topic 0x%04X does not exist, ignoring.",topic);
	}
}

void DVLDecode::dvl_nop() { /* NO OPERATION */}

void DVLDecode::dvl_vel_3()	{ /* Just a macro */}

void DVLDecode::dvl_vel_x() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_vel_x->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_vel_x] Published data : %f.",value);
}

void DVLDecode::dvl_vel_y() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_vel_y->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_vel_y] Published data : %f.",value);
}

void DVLDecode::dvl_vel_z() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_vel_z->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_vel_z] Published data : %f.",value);
}

void DVLDecode::dvl_vel_e() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_vel_e->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_vel_e] Published data : %f.",value);
}

void DVLDecode::dvl_dist_bottom() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_dist_bottom->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_dist_bottom] Published data : %f.",value);
}

void DVLDecode::dvl_dist_1() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_dist_1->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_dist_1] Published data : %f.",value);
}

void DVLDecode::dvl_dist_2() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_dist_2->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_dist_2] Published data : %f.",value);
}

void DVLDecode::dvl_dist_3() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_dist_3->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_dist_3] Published data : %f.",value);
}

void DVLDecode::dvl_dist_4() 
{
	float value;
	memcpy(&value, dres_frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = value;
	dvl_dist_4->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[DVLDecode::dvl_dist_4] Published data : %f.",value);
}


