#ifndef DRES_DECODER_H
#define DRES_DECODER_H

#include <linux/can.h>
#include <net/if.h>

#include "ms5837_handler.hpp"
#include "dvl_handler.hpp"
#include "mbox_can.hpp"
#include <chrono>

namespace Dres
{
	namespace Decoder
	{
		class DresDecoder
		{
			public:
				DresDecoder(rclcpp::Node*, struct ifreq*);
				void decode_dres(struct can_frame*);
			private:
				rclcpp::Node* context_node;

				static const int device_ct = 0x0004;
				Dres::Handler::DresHandler* dev_handlers[device_ct];

				/* polling intervals */
				rclcpp::TimerBase::SharedPtr _dreq_timer_10ms;
				rclcpp::TimerBase::SharedPtr _dreq_timer_25ms;
				rclcpp::TimerBase::SharedPtr _dreq_timer_40ms;
				
				void _data_request_10ms();
				void _data_request_25ms();
				void _data_request_40ms();
				Mailbox::MboxCan* dreq_mb;

				/* Handler objects*/
				// DresHandler::DummyHandler dummy_handler*;
				Dres::Handler::MS5837Handler* ms5837_handler;
				Dres::Handler::DVLHandler* dvl_handler;
		};
	}
}

#endif
