#include "module_loader.hpp"

using namespace Module;

ModuleLoader::ModuleLoader(rclcpp::Node* ctx, struct ifreq* ifr)
{
	node_context = ctx;
	module_mb = new Mailbox::MboxCan(ifr, "module_mb");
	module_ct = 5;
	module_list.reserve(module_ct);
	std::fill(module_list.begin(), module_list.end(),nullptr);


//	load_module<DVLModule>(dvl, MODULE_DVL_ENABLE); 				!!! Yea we don't use this DVL anymore
	load_module<MS5837Module>(ms5837, MODULE_MS5837_ENABLE);
	load_module<BRLIGHTModule>(brlight, MODULE_BRLIGHT_ENABLE);
//  load_module<BRPING1Module>(brping1, MODULE_BRPING1_ENABLE); 	!!! DISABLED FOR SAFETY, NO TOUCH
	
	RCLCPP_INFO(node_context->get_logger(),"[ModuleLoader] Initialized.");
}

ModuleLoader::~ModuleLoader()
{
	Mailbox::MboxCan::close_mbox(module_mb);
	delete module_mb;
}

template <typename module_type>
void ModuleLoader::load_module(DeviceModule* mod, uint8_t enable_bit)
{
	if(module_enabled_field & enable_bit)
	{
		mod = new module_type(node_context, module_mb);
		module_list[mod->module_device_id] = mod;
	}
}

void ModuleLoader::module_decode_dres(struct can_frame* frame)
{
	uint16_t device;
	memcpy(&device, frame->data, sizeof(char)*2);

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*
		RCLCPP_INFO(node_context->get_logger(),"[ModuleLoader::module_decode_dres]DEBUG START");
		RCLCPP_INFO(node_context->get_logger(),"************************ DEBUG ERROR ************************");
		RCLCPP_INFO(node_context->get_logger(),"The dres decoder has detected a device that really SHOULDNT exist!");
		RCLCPP_INFO(node_context->get_logger(),"This is most likely related to some really upsetting memory bug caused");
		RCLCPP_INFO(node_context->get_logger(),"from the dynamic loading of device modules. Still working on a fix for it.");
		RCLCPP_INFO(node_context->get_logger(),"%-16s 0x%03X", "FRAME_ID", frame->can_id);
		RCLCPP_INFO(node_context->get_logger(),"%-16s %02i", "LENGTH", frame->can_dlc);
		RCLCPP_INFO(node_context->get_logger(),"%-16s 0x%04X", "DECODED_DEVICE", device);
		RCLCPP_INFO(node_context->get_logger(),"%-16s 0x%04X", "DECODED_TOPIC", frame->data[2]);
		RCLCPP_INFO(node_context->get_logger(),"FRAME DATA");
		RCLCPP_INFO(node_context->get_logger(),"\t%02X %02X %02X %02X", frame->data[0], frame->data[1], frame->data[2], frame->data[3]);
		RCLCPP_INFO(node_context->get_logger(),"\t%02X %02X %02X %02X", frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
		RCLCPP_INFO(node_context->get_logger(),"END CAN FRAME DUMP");
		RCLCPP_INFO(node_context->get_logger(),"************************ DEBUG ERROR ************************\n\n");
*/
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

	 if(device < module_ct
	 	&& module_list[device] != nullptr
	 	&& module_list[device]->module_enable_dres
	 	&& module_list[device]->module_hw_info != 0)
	 {
		module_list[device]->dres_handle(frame);
	 }
	 else
	 {
	 	RCLCPP_INFO(node_context->get_logger(),
	 		"[ModuleLoader::module_decode_dres] Device 0x%04X does not exist/disabled/no dres support. Ignoring");
	 }
}
