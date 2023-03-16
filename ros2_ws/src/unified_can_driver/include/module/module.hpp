/* 3.7.23
 * Connor Larmer
 * module template for embedded communication
 */

#ifndef MOD_H
#define MOD_H

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <linux/can.h>
#include "mbox_can.hpp"
#include "scion_types/msg/datapoint.hpp"
#include "config.hpp"
#include "rclcpp/rclcpp.hpp"
 
namespace Module
{
	class DeviceModule
	{
		public:
			DeviceModule(rclcpp::Node*,
				Mailbox::MboxCan*,
				uint8_t, 			/*timer_len*/
				uint16_t, 			/*device_id*/
				bool, 				/*enable_timer*/
				bool, 				/*enable_dres*/
				uint8_t); 			/*topic_count*/
			~DeviceModule();
			virtual void mod_init();
			virtual void dres_handle(struct can_frame*);

			uint8_t module_timer_len; 		/* interval for timer cb */
			uint16_t module_device_id; 		/* module device id (matches hw) */
			bool module_enable_ti; 			/* flag to enable timer_cb */
			bool module_enable_dres; 		/* flag to enable dres_cb */
			uint8_t module_topic_ct; 		/* number of topic functions */
			uint32_t module_hw_info;	 	/* Info received from 0x000 read */
		protected:
			Mailbox::MboxCan* mailbox_ptr;
			rclcpp::Node* node_context;
			rclcpp::TimerBase::SharedPtr poll_timer;	
			struct can_frame* frame;
			typedef void (DeviceModule::*topic_ptr_t) ();
			topic_ptr_t* module_topic_ptr_array;

			virtual void timer_callback();
			virtual void dres_info();
	};
}

#endif
