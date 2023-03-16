#ifndef MS5837_DECODE_H
#define MS5837_DECODE_H

#include <linux/can.h>
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/float32.hpp"
#include "dres_handler.hpp"
namespace Dres
{
	namespace Handler
	{
		class MS5837Handler : public DresHandler
		{
			public:
				MS5837Handler(rclcpp::Node*);
				void dres_handle(struct can_frame*);
			private:

				/* Sensor specific datafields */
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr sensor_depth;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr sensor_temp;
				void ms5837_nop();
				void ms5837_depth();
				void ms5837_temp();
		};
	}
}
#endif
