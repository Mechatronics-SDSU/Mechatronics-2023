#ifndef DVL_HANDLER_H
#define DVL_HANDLER_H

#include <linux/can.h>
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/float32.hpp"
#include "dres_handler.hpp"
namespace Dres
{
	namespace Handler
	{
		class DVLHandler : public DresHandler
		{
			public:
				DVLHandler(rclcpp::Node*);
				void dres_handle(struct can_frame*);
			private:
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
	
}
#endif
