#ifndef DRES_HANDLER_H
#define DRES_HANDLER_H

#include <linux/can.h>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/float32.hpp"
#include "scion_types/msg/datapoint.hpp"
#include "device_macros.hpp"

namespace Dres
{
	namespace Handler
	{
		class DresHandler
		{
			public:
				DresHandler(rclcpp::Node*, uint16_t, uint8_t);
				~DresHandler();
				virtual void dres_handle(struct can_frame*);
				uint16_t device_id;
			protected:
				typedef void (DresHandler::*topic_ptr_t) ();
				topic_ptr_t* dev_topics;
				uint8_t dev_topic_ct;
				rclcpp::Node* node_context;
				struct can_frame* dres_frame;
		};
	}
}
#endif
