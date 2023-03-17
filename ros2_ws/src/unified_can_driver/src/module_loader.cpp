#include "module_loader.hpp"

using namespace Module;

ModuleLoader::ModuleLoader(rclcpp::Node* ctx, struct ifreq* ifr)
{
	node_context = ctx;
	module_mb = new Mailbox::MboxCan(ifr, "module_mb");
	module_ct = 5;
	module_list.reserve(module_ct);
	std::fill(module_list.begin(), module_list.end(),nullptr);

	/* Conditionally enable modules */
	// if(GlobalSettings::module_enabled_field & MODULE_DVL_ENABLE)
	// {
		// dvl = new DVLModule(node_context, module_mb);
		// module_list[dvl->module_device_id] = dvl;
	// }
// 
	// if(GlobalSettings::module_enabled_field & MODULE_MS5837_ENABLE)
	// {
		// ms5837 = new MS5837Module(node_context, module_mb);
		// module_list[ms5837->module_device_id] = ms5837;
	// }
	// if(GlobalSettings::module_enabled_field & MODULE_BRLIGHT_ENABLE)
	// {
		// brlight = new BRLIGHTModule(node_context, module_mb);
		// module_list[ms5837->module_device_id] = brlight;
	// }
	// if(GlobalSettings::module_enabled_field & MODULE_BRPING1_ENABLE)
	// {
		// ms5837 = new MS5837Module(node_context, module_mb);
		// module_list[ms5837->module_device_id] = ms5837;
	// }

	load_module<DVLModule>(dvl, MODULE_DVL_ENABLE);
	load_module<MS5837Module>(ms5837, MODULE_MS5837_ENABLE);
	load_module<BRLIGHTModule>(brlight, MODULE_BRLIGHT_ENABLE);
	load_module<BRPING1Module>(brping1, MODULE_BRPING1_ENABLE);
	
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
	if(GlobalSettings::module_enabled_field & enable_bit)
	{
		mod = new module_type(node_context, module_mb);
		module_list[mod->module_device_id] = mod;
	}
}

void ModuleLoader::module_decode_dres(struct can_frame* frame)
{
	uint16_t device;
	memcpy(&device, frame->data, sizeof(char)*2);

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