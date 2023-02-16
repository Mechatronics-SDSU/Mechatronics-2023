#ifndef DVL_DECODE_H
#define DVL_DECODE_H

#include <linux/can.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


namespace DVL
{

	class DVLDecode
	{
		public:
			DVLDecode(rclcpp::Node*);
			void dres_handler(struct can_frame*);
			uint16_t device_id;
		private:
			typedef void (DVLDecode::*topic_ptr_t) ();
			rclcpp::Node* node_context;
			struct can_frame* dres_frame;

			/* Ros2 Topics */
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_vel_x;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_vel_y;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_vel_z;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_vel_e;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_dist_bottom;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_dist_1;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_dist_2;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_dist_3;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_dist_4;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_pwr_input;		/* Currently unused */
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_pwr_volt;		/* Currently unused */
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dvl_pwr_curr;		/* Currently unused */
			
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

			static const int dvl_topic_ct = 0x000F;
			topic_ptr_t dvl_topics[dvl_topic_ct];
	};

	
}
#endif
