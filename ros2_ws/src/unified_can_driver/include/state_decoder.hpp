#ifndef STATE_DECODER_H
#define STATE_DECODER_H

#include "mbox_can.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/sub_state.hpp"

enum class HOST_STATE
{
	MANUAL,
	AUTO_BTN,
	AUTO_EXT
};



class StateDecoder
{
	public:
		StateDecoder(rclcpp::Node* ctx, struct ifreq* ifr);
		~StateDecoder();
		void check_state();
	private:
		HOST_STATE curr_state;
		bool leak;
		bool estop;
		Mailbox::MboxCan* mbox;
		rclcpp::Node* node_context;
		struct can_frame latest;
		struct can_filter filter[4];

		rclcpp::Publisher<scion_types::msg::SubState>::SharedPtr pub_topic;
};


#endif