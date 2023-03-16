#ifndef MOD_LOADER_H
#define MOD_LOADER_H

#include <linux/can.h>
#include <net/if.h>

#include "ms5837.hpp"
#include "dvl.hpp"
#include "mbox_can.hpp"
#include "rclcpp/rclcpp.hpp"
#include "config.hpp"

namespace Module
{
	class ModuleLoader
	{
		public:	
			ModuleLoader(rclcpp::Node*, struct ifreq*);
			~ModuleLoader();
			void module_decode_dres(struct can_frame*);
		private:
			rclcpp::Node* node_context;

			int module_ct;
			std::vector<DeviceModule*> module_list;
			Mailbox::MboxCan* module_mb;

			/* Modules */
			MS5837Module* ms5837;
			DVLModule* dvl;
	};
}

#endif