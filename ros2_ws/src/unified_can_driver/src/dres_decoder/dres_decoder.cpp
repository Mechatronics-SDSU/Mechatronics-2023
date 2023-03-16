/* Connor Larmer
 * 1.4.23
 * Decoder package for MS5837 Depth&Temp sensor.
 * filters and decodes data from the can2ros_driver
 * node in order to output nice float values to ros2. 
 */
#include "rclcpp/rclcpp.hpp"
#include "dres_decoder.hpp"
#include "config.hpp"
#include <net/if.h>


using namespace Dres;

Decoder::DresDecoder::DresDecoder(rclcpp::Node* node, struct ifreq* ifr)
{
	context_node = node;

	dreq_mb = new Mailbox::MboxCan(ifr, "dreq");

	/* Conditionally initialize device modules */
	dvl_handler = (MOD_ENABLED & DVL_ENABLE) ?
		new Handler::DVLHandler(context_node)
		: nullptr;
	ms5837_handler = (MOD_ENABLED & MS5837_ENABLE) ?
		new Handler::MS5837Handler(context_node)
		: nullptr;
	dev_handlers[0x0] = nullptr;
	dev_handlers[0x1] = nullptr;
	dev_handlers[0x2] = dvl_handler;
	dev_handlers[0x3] = ms5837_handler;
	dev_handlers[0x4] = nullptr;

	/* Send Enable signal on enabled devices*/
	struct can_frame init_frame;
	memset(&init_frame, 0, sizeof(struct can_frame));
	init_frame.can_dlc = 8;
	init_frame.can_id = 0x22; 	/* STOW command */
	for(int d = 0; d < 0x4; d++) 
	{
		if(dev_handlers[d] != nullptr)
		{
			/* DEBUG */ RCLCPP_INFO(context_node->get_logger(), "[DresDecoder]init sig for %04X.", d);
			init_frame.data[0] = d;
			Mailbox::MboxCan::write_mbox(dreq_mb, & init_frame);
		}
	}
	
	/* Set interval timers */
	_dreq_timer_10ms = context_node->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DresDecoder::_data_request_10ms, this));
	_dreq_timer_25ms = context_node->create_wall_timer(std::chrono::milliseconds(25), std::bind(&DresDecoder::_data_request_25ms, this));
	_dreq_timer_40ms = context_node->create_wall_timer(std::chrono::milliseconds(40), std::bind(&DresDecoder::_data_request_40ms, this));
	
	RCLCPP_INFO(context_node->get_logger(), "[DresDecoder] Initialized.");
}

/* The purpose of this callback is to filter device IDs.
 * in theory it is possible to have multiple decoders within
 * this method, however that would mean many topics under one node...
 * which is BAD and STUPID and UGLY. Prior to filtering, we create
 * a can_frame to pass into our decoder for the sake of sanity and simplicity.
 */
void Decoder::DresDecoder::decode_dres(struct can_frame* dres_frame)
{
	uint16_t device;
	memcpy(&device, dres_frame->data, sizeof(char)*2);

	if(device < Decoder::DresDecoder::device_ct && dev_handlers[device] != nullptr)
	{
		dev_handlers[device]->dres_handle(dres_frame);
	}
	else
	{
		RCLCPP_INFO(context_node->get_logger(), "[DresDecoder::decode_dres] ID does not match any enabled modules, ignoring.");
	}
}

/* 
 * _data_request:
 * We poll the DVL for frequently needed data. Instead of making a ros2
 * service call, we publish data directly to the CAN bus for speed/simplicity
 */
void Decoder::DresDecoder::_data_request_10ms()
{
	if(DO_DEVICE_POLLING)
	{
		/* DVL polling */
		struct can_frame DVL_POLL_FRAME;
		unsigned char dvl_dreq[8] {0x03,0x00,0x07,0x00,0x00,0x00,0x00,0x00};
		memset(&DVL_POLL_FRAME, 0, sizeof(struct can_frame));
		DVL_POLL_FRAME.can_id = 0x020;
		DVL_POLL_FRAME.can_dlc = 4;
		memcpy(&DVL_POLL_FRAME.data, &dvl_dreq, 8);
		Mailbox::MboxCan::write_mbox(dreq_mb, &DVL_POLL_FRAME);	
	}
}
void Decoder::DresDecoder::_data_request_25ms()
{
		if(DO_DEVICE_POLLING)
	{
		
	}
}
void Decoder::DresDecoder::_data_request_40ms()
{
	if(DO_DEVICE_POLLING)
	{
		/* MS5837 polling */
		struct can_frame MS5837_POLL_FRAME;
		unsigned char dvl_dreq[8] {0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x00};
		memset(&MS5837_POLL_FRAME, 0, sizeof(struct can_frame));
		MS5837_POLL_FRAME.can_id = 0x020;
		MS5837_POLL_FRAME.can_dlc = 4;
		memcpy(&MS5837_POLL_FRAME.data, &dvl_dreq, 8);
		Mailbox::MboxCan::write_mbox(dreq_mb, &MS5837_POLL_FRAME);	

	}
}
