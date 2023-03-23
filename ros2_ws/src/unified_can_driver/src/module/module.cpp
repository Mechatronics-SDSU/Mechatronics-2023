#include "module.hpp"

using namespace Module;

DeviceModule::DeviceModule(rclcpp::Node* ctx,
	Mailbox::MboxCan* mb,
	uint8_t timer_len,
	uint16_t device_id,
	bool enable_ti,
	bool enable_dres,
	uint8_t topic_ct)
{
	node_context = ctx;
	mailbox_ptr = mb;
	module_timer_len = timer_len;
	module_device_id = device_id;
	module_enable_ti = enable_ti;
	module_enable_dres = enable_dres;
	module_topic_ct = topic_ct;

	module_topic_ptr_array = new topic_ptr_t[module_topic_ct];

	/* Timer callback creation */
	if(do_device_polling && module_enable_ti)
	{
		poll_timer = node_context->create_wall_timer(
			std::chrono::milliseconds(module_timer_len),
			std::bind(&DeviceModule::timer_callback, this));
	}
	else
	{
		poll_timer = nullptr;
	}
}

DeviceModule::~DeviceModule()
{
	delete[] module_topic_ptr_array;
}

void DeviceModule::dres_handle(struct can_frame* frame) { /* UNUSED */ }
void DeviceModule::timer_callback() { /* UNUSED */ }
void DeviceModule::dres_info() { /* UNUSED */ }
void DeviceModule::mod_init() { /* UNUSED */ }