#ifndef MOD_DVL_H
#define MOD_DVL_H

#include "module.hpp"

namespace Module
{
	class DVLModule : public DeviceModule
	{
		public:
			DVLModule(rclcpp::Node*, Mailbox::MboxCan*);
			void mod_init();
			void dres_handle(struct can_frame*);

		private:
				void timer_callback();
				void dres_info();
				/* Ros2 Topics */
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_vel_x_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_vel_y_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_vel_z_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_vel_e_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_dist_bottom_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_dist_1_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_dist_2_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_dist_3_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_dist_4_topic;
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_pwr_input_topic;		/* Currently unused */
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_pwr_volt_topic;		/* Currently unused */
				rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr dvl_pwr_curr_topic;		/* Currently unused */
				
				void dvl_nop();
				/* Velocity */
				void dvl_vel_3();
				void dvl_vel_x();
				void dvl_vel_y();
				void dvl_vel_z();
				void dvl_vel_e();
				/* Distance */
				void dvl_dist_bottom();
				void dvl_dist_1();
				void dvl_dist_2();
				void dvl_dist_3();
				void dvl_dist_4();
				
				/* Below functions have not been implemented */

				/* Power */
				void dvl_pwr_input();
				void dvl_pwr_volt();
				void dvl_pwr_curr();
				/* System */
				void dvl_sys_set_setup();
				void dvl_sys_get_setup();
				void dvl_sys_set_sys();
				void dvl_sys_get_sys();
				void dvl_sys_set_auto();	
	};
}

#endif
