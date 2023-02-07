#ifndef MS5837_DECODE_H
#define MS5837_DECODE_H

#include <linux/can.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


namespace MS5837
{

	class MS5837Decode
	{
		public:
			MS5837Decode(rclcpp::Node*);
			void dres_handler(struct can_frame*);
			uint16_t device_id;
		private:
			typedef void (MS5837Decode::*topic_ptr_t) ();
			rclcpp::Node* node_context;
			struct can_frame* dres_frame;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sensor_depth;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sensor_temp;
			
			void ms5837_nop();
			void ms5837_depth();
			void ms5837_temp();

			static const int ms5837_topic_ct = 0x0004;
			topic_ptr_t ms5837_topics[ms5837_topic_ct];
	};

	
}
#endif
