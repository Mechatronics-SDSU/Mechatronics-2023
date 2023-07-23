//////////////// example.cpp ////////////////
#include "example.hpp"

// You MUST use the 'Module' namespace.
using namespace Module;

// Constructor implementation, ctx and mb are only used to be passed
// in to the parent constructor.
ExampleModule::ExampleModule(rclcpp::Node* ctx, Mailbox::MboxCan* mb)
	: DeviceModule(
		ctx,		// Node context, AKA the parent Ros2 node
		mb,			// Mailbox pointer, this is used to read and write CAN frames
		10, 		// timer_callback() interval, defined in milliseconds
		// Device ID, can be expressed as an integer or through a macro
		// (as in this example). Instructions for implementing a macro can
		// be found in the README.
		static_cast<uint8_t>(CanDriver::Device::EXAMPLE::ID),
		true,		// Flag to enable timer_callback()
		true,		// Flag to enable dres decoding, this might be disabled if
					// the device is not a sensor.
		6)			// Number of topics, MUST corrospond to number of
					// topic functions in your header file.
{
	// This is how you include your topic functions into the inherited
	// pointer array. This array is then used by dres_handle() to jump
	// to the correct topic function. Note that the index of each function
	// pointer should correspond to the topic ID noted in the protocol sheet.
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&ExampleModule::dres_info);
	module_topic_ptr_array[1] = static_cast<topic_ptr_t>(&ExampleModule::example_nop);
	module_topic_ptr_array[2] = static_cast<topic_ptr_t>(&ExampleModule::example_data1);
	module_topic_ptr_array[3] = static_cast<topic_ptr_t>(&ExampleModule::example_data2);
	module_topic_ptr_array[4] = static_cast<topic_ptr_t>(&ExampleModule::example_status1);
	module_topic_ptr_array[5] = static_cast<topic_ptr_t>(&ExampleModule::example_status2);

	// initialize ROS2 publishers, node_context is the inherited variable
	// that points to the driver's ROS2 Node object
	topic_data1		= node_context->create_publisher<scion_types::msg::Datapoint> ("example_data1");
	topic_data2		= node_context->create_publisher<scion_types::msg::Datapoint> ("example_data2");
	topic_status1	= node_context->create_publisher<scion_types::msg::Datapoint> ("example_status1");
	topic_status2	= node_context->create_publisher<scion_types::msg::Datapoint> ("example_status2");

	// If needed, send a 'wakeup' frame. This may not be required
	// depending on the configuration of the device on the embedded system.
	struct can_frame init_frame;
	memset(&init_frame, 0, sizeof(struct can_frame));
	init_frame.can_dlc = 8;
	// Macro representing the STOW command on the embedded system
	init_frame.can_id = static_cast<uint8_t>(CanDriver::Command::STOW);
	init_frame.data[0] = this->module_device_id;
	// write generated CAN frame to the bus
	Mailbox::MboxCan::write_mbox(this->mailbox_ptr, &init_frame);

	// Log the output
	RCLCPP_INFO(node_context->get_logger(), "[ExampleModule] Initialized.");
}

// Whatever code should be executed on a set interval (i.e. 40ms data request polling)
void ExampleModule::timer_callback()
{
	//do_device_polling : Global configuration variable
	//module_enable_ti	: inherited flag; set in constructor
	if(do_device_polling && this->module_enable_ti)
	{
		// Create temporary can_frame & populate with data
		struct can_frame poll_frame;
		memset(&poll_frame, 0, sizeof(struct can_frame));
		// Set ID to data request command
		poll_frame.can_id = static_cast<uint8_t>(CanDriver::Command::DREQ);
		poll_frame.can_dlc = 8;
		poll_frame.data[0] = this->module_device_id;
		// Device topic to retrieve newest data, for more information
		// see the "Commands & Macros" sections of the documentation
		poll_frame.data[2] = static_cast<uint8_t>(CanDriver::Device::EXAMPLE::TOPIC_DATA_ALL);
		Mailbox::MboxCan::write_mbox(mailbox_ptr, &poll_frame);
	}
}

// dres_handle() : This method is called by the Module Loader on the event
// that a CAN frame with a device ID matching that of your module is received.
// From there, this method decodes the topic ID and jumps to the appropriate
// topic method. In a module that supports dres_decoding, the following code
// will pretty much always be the same.
void ExampleModule::dres_handle(struct can_frame*)
{
	/* We copy the 16bit topic ID from the frame (EX: aaaaBBBBccccdddd -> we get BBBB) */
	uint16_t topic;
	this->frame = frame;
	memcpy(&topic, this->frame->data + ( sizeof(char) * 2 ), sizeof(char)*2 );
	if(topic < module_topic_ct)
	{
		(this->*module_topic_ptr_array[topic])(); 		/* we jump to correct topic function */
	}
	else
	{
		RCLCPP_INFO(node_context->get_logger(),
			"[ExampleModule::dres_handle] Topic 0x%04X does not exist, ignoring.",topic);
	}
}

// This method is unused, and will be removed from the parent
// class in the future. For now, it simply exists as a dummy.
void ExampleModule::mod_init() {}	// Unuse

// This method exists only to handle topic 0x0000 (AKA the info topic).
// At the time of writing this example, the module_hw_info variable is
// unused, however will likely be needed for future development.
void ExampleModule::dres_info()
{
	memcpy(&module_hw_info, this->frame->data+(sizeof(char)*4), sizeof(float) );
	RCLCPP_INFO(node_context->get_logger(),
		"[ExampleModule::dres_info] Received info from hw.");
}

//////////////// Module specific topic methods ////////////////

void ExampleModue::example_nop() { }	// Do Nothing

// In this example, all datapoints are expressed as 32-bit floats.
// The actual data payload is the last 32-bits of the CAN frame,
// and can thus be extracted through memory manipulation. The majority
// of the following code is copy-pasted with names changed.
void ExampleModue::example_data1()
{
	/* We copy the last 32 bits of the frame into a float, then publish
	it to the correct ROS2 topic */

	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	topic_data1->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[ExampleModule::example_data1] Published data : %f.",data1);
}

void ExampleModue::example_data2()
{
	/* We copy the last 32 bits of the frame into a float, then publish
	it to the correct ROS2 topic */

	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	topic_data2->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[ExampleModule::example_data2] Published data : %f.",data1);	
}

void ExampleModue::example_status1()
{
	/* We copy the last 32 bits of the frame into a float, then publish
	it to the correct ROS2 topic */

	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	topic_status1->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[ExampleModule::example_status1] Published data : %f.",data1);
}

void ExampleModue::example_status2()
{
	/* We copy the last 32 bits of the frame into a float, then publish
	it to the correct ROS2 topic */

	float value;
	memcpy(&value, this->frame->data+(sizeof(char)*4), sizeof(float) );
	auto topic_msg = scion_types::msg::Datapoint();
	topic_msg.data = value;
	topic_status2->publish(topic_msg);
	RCLCPP_INFO(node_context->get_logger(),
		"[ExampleModule::example_status2] Published data : %f.",data1);
}
