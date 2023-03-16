#include "ms5837_handler.hpp"

using namespace Dres::Handler;

/* Constructor method for MS5837 DRES decoder class;
 * Must pass in parent node upon initialization for topic generation.
 */
MS5837Handler::MS5837Handler(rclcpp::Node* context)
	: DresHandler(context, MS5837_DEV_ID, 4)
{

	/* pointer-to-member-function weirdness dictates that each topic function is manually added*/
	dev_topics[0] = static_cast<topic_ptr_t>(&MS5837Handler::ms5837_nop);			/* 0000 NOP */
	dev_topics[1] = static_cast<topic_ptr_t>(&MS5837Handler::ms5837_nop);			/* 0001 DATA MACRO */
	dev_topics[2] = static_cast<topic_ptr_t>(&MS5837Handler::ms5837_depth);		/* 0002 DEPTH */
	dev_topics[3] = static_cast<topic_ptr_t>(&MS5837Handler::ms5837_temp);		/* 0003 TEMPERATURE */

	sensor_depth = node_context->create_publisher<scion_types::msg::Datapoint> ("ms5837_depth", 10);
	sensor_temp  = node_context->create_publisher<scion_types::msg::Datapoint> ("ms5837_temp", 10);

	RCLCPP_INFO(node_context->get_logger(), "[MS5837Handler] Handler Initialized.");
}

void MS5837Handler::dres_handle(struct can_frame* frame) 
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
			"[MS5837Handler::dres_handler] Topic 0x%04X does not exist, ignoring.",topic);
	}
}

void MS5837Handler::ms5837_nop() { /* NO OPERATION */}

void MS5837Handler::ms5837_depth() 
{
	/* We copy the last 32 bits of the frame into a float, then publish
	   it to the correct ROS2 topic */
	float depth;
	memcpy(&depth, dres_frame->data+(sizeof(char)*4), sizeof(float) );

	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = depth;
	sensor_depth->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[MS5837Handler::ms5837_depth] Published data : %f.",depth);

}

void MS5837Handler::ms5837_temp()
{
	/* We copy the last 32 bits of the frame into a float, then publish
	   it to the correct ROS2 topic */
	float temp;
	memcpy(&temp, dres_frame->data+(sizeof(char)*4), sizeof(float) );

	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = temp;
	sensor_temp->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[MS5837Handler::ms5837_temp] Published data : %f.",temp);

}

