#ifndef DRES_DECODE_NODE_H
#define DRES_DECODE_NODE_H

#include <linux/can.h>
#include <net/if.h>

#include "can_msg_interfaces/msg/can_frame.hpp"
#include "ms5837_handler.hpp"

class DresDecodeNode : public rclcpp::Node
{
	public:
		DresDecodeNode();
	private:
		rclcpp::Subscription<can_msg_interfaces::msg::CanFrame>::SharedPtr _dres_mb;
		MS5837::MS5837Decode* ms5837_decoder;
		void decode_cb(const can_msg_interfaces::msg::CanFrame::SharedPtr) const;
		
};

#endif
