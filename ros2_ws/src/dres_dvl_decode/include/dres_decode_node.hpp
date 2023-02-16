#ifndef DRES_DECODE_NODE_DVL_H
#define DRES_DECODE_NODE_DVL_H

#include <linux/can.h>
#include <net/if.h>

#include "scion_types/msg/can_frame.hpp"
#include "dvl_handler.hpp"
#include "device_macros.hpp"
#include "mbox_can.hpp"
class DresDecodeNode : public rclcpp::Node
{
	public:
		DresDecodeNode();
	private:
		void _data_request();
		rclcpp::Subscription<scion_types::msg::CanFrame>::SharedPtr _dres_mb;
		DVL::DVLDecode* dvl_decoder;
		void decode_cb(const scion_types::msg::CanFrame::SharedPtr) const;
		rclcpp::TimerBase::SharedPtr _dreq_timer;
		Mailbox::MboxCan* _poll_mb;
};

#endif
