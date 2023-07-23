#ifndef MOD_BRPING1_H
#define MOD_BRPING1_H
#include "module.hpp"

namespace Module
{
	class BRPING1Module : public DeviceModule
	{
		public:
			BRPING1Module(rclcpp::Node*, Mailbox::MboxCan*);
			void dres_handle(struct can_frame*);
		private:	
			void mod_init();
			void timer_callback();
			void dres_info();

			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_dist_short;
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_confidence;

			void brping1_nop();
			void brping1_dist_short();

			/* Sensor specific variables */
	};
}
#endif
