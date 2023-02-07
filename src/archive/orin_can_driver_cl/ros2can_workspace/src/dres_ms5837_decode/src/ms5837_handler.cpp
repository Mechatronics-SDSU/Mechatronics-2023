#include "ms5837_handler.hpp"

using namespace MS5837;

/* Constructor method for MS5837 DRES decoder class;
 * Must pass in parent node upon initialization for topic generation.
 */
MS5837Decode:: MS5837Decode(rclcpp::Node* context)
{
	node_context = context;

	/* pointer-to-member-function weirdness dictates that each topic function is manually added*/
	ms5837_topics[0] = &MS5837Decode::ms5837_nop;			/* 0000 NOP */
	ms5837_topics[1] = &MS5837Decode::ms5837_nop;			/* 0001 DATA MACRO */
	ms5837_topics[2] = &MS5837Decode::ms5837_depth;			/* 0002 DEPTH */
	ms5837_topics[3] = &MS5837Decode::ms5837_temp;			/* 0003 TEMPERATURE */

	device_id = 0x0002; 		/* public device ID (used by dres_decoder_node) */

	sensor_depth = node_context->create_publisher<std_msgs::msg::Float32> ("ms5837_depth", 10);
	sensor_temp  = node_context->create_publisher<std_msgs::msg::Float32> ("ms5837_temp", 10);

	RCLCPP_INFO(node_context->get_logger(), "[MS5837Decode] Decoder Initialized.");
}

void MS5837Decode::dres_handler(struct can_frame* frame) 
{
	/* We copy the 16bit topic ID from the frame (EX: aaaaBBBBccccdddd -> we get BBBB) */
	uint16_t topic;
	dres_frame = frame;
	memcpy(&topic, dres_frame->data + ( sizeof(char) * 2 ), sizeof(char)*2 );
	if(topic < ms5837_topic_ct)
	{
		(this->*ms5837_topics[topic])(); 		/* we jump to correct topic function */
	}
	else
	{
		RCLCPP_INFO(node_context->get_logger(),
			"[MS5837Decode::dres_handler] Topic 0x%04X does not exist, ignoring.",topic);
	}
}

void MS5837Decode::ms5837_nop() { /* NO OPERATION */}

void MS5837Decode::ms5837_depth() 
{
	/* We copy the last 32 bits of the frame into a float, then publish
	   it to the correct ROS2 topic */
	float depth;
	memcpy(&depth, dres_frame->data+(sizeof(char)*4), sizeof(float) );

	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = depth;
	sensor_depth->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[MS5837Decode::ms5837_depth] Published data : %f.",depth);

}

void MS5837Decode::ms5837_temp()
{
	/* We copy the last 32 bits of the frame into a float, then publish
	   it to the correct ROS2 topic */
	float temp;
	memcpy(&temp, dres_frame->data+(sizeof(char)*4), sizeof(float) );

	auto topic_msg = std_msgs::msg::Float32();
	topic_msg.data = temp;
	sensor_temp->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[MS5837Decode::ms5837_temp] Published data : %f.",temp);

}

