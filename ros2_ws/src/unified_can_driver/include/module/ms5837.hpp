#ifndef MOD_MS5837_H
#define MOD_MS5837_H

#include "module.hpp"

namespace Module
{
	class MS5837Module : public DeviceModule
	{
		public:
			MS5837Module(rclcpp::Node*, Mailbox::MboxCan*);
			void dres_handle(struct can_frame*);
			void mod_init();

		private:
			void timer_callback();
			void dres_info();
			
			/* Sensor specific variables */
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr sensor_depth;
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr sensor_temp;

			void ms5837_nop();
			void ms5837_depth();
			void ms5837_temp();
	};
}

#endif
