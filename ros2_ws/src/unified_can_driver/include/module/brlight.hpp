#ifndef MOD_BRLIGHT_H
#define MOD_BRLIGHT_H
#include "module.hpp"

namespace Module
{
	class BRLIGHTModule : public DeviceModule
	{
		public:
			BRLIGHTModule(rclcpp::Node*, Mailbox::MboxCan*);
			void dres_handle(struct can_frame*);
		private:	
			void mod_init();
			void timer_callback();
			void dres_info();

			void brlight_nop();

			/* Sensor specific variables */
	};
}
#endif
