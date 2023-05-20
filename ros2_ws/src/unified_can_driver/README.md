# Scion CAN2ROS2 Driver

to document
	MboxCan
	Various included methods and variables in module.hpp
	Document Commands & Macros
	
*also known as unified_can_driver*

***

## Introduction

This ROS2 package is the driver for CAN communication between the Nvidia Jetson Orin and Teensy 4.1 Embedded node. It is built to be modular, allowing for new 'device modules' to be quickly developed and deployed as Scion's sensor suite expands. The only dependency this package requires (other than ROS2...) is the `scion_types` ROS2 package for it's custom msgs. The driver can be configured to use a different CAN interface, control embedded update polling, and enable/disable certain modules without recompiling via a launch file. More information regarding modules, published topcis, services, and user configurable fields can be found below.

***

## Table of Contents

 1. [Introduction](#Introduction)
 2. [Building](#Building)
 3. [Configuration](#Configuration)
 4. [ROS2 Information](#ROS2%20Information)
 5. [Modules](#Modules)
 6. [Development](#Development)

***

## Building

In order to build the driver as a ROS2 package, first verify the following:

 - ROS2 Foxy is installed
 - The `scion_types` package is in the same workspace as `unified_can_driver` package
 - You have sourced your setup.sh (found in `/opt/ros/foxy/setup.sh` or your workspace)

### Build steps

 1. In your workspace, first build `scion_types` (`colcon build --packages-select scion_types`)
 2. Source your local setup.sh generated from the previous step (`source install/setup.sh`)
 3. Build `unified_can_driver` (`colcon build --packages-select unified_can_driver`)
 4. Source your local setup.sh again.
 5. The driver can now be launched with `ros2 run unified_can_driver unified_can_driver`
 
***

## Configuration

The driver's parameters can be configured either through the command-line or a launch file, see below.

### Command Line

There are three parameters that determine the way that the driver operates:

 - `can_bus_interface` A string representing the desired interface (i.e: can0, can1, etc.) the driver will connect to.
 - `do_module_polling` A boolean determining whether modules will automatically poll their embedded counterpart for new data.
 - `module_bitfield` A bitfield determining which module(s) should be enabled/disabled at startup, see below for more information.

In order to configure the driver via the command-line, simply use the following format:
 * `ros2 run unified_can_driver unified_can_driver --ros-args -p [parameter]:=[option] ... ...`
 * I.E: `ros2 run unified_can_driver unified_can_driver --ros-args -p can_bus_interface:="can0" -p do_module_polling:=false -p module_bitfield:=255`

### Launch File
(Comming soon to a theater near you!)
***

## ROS2 Information

Upon launch, the driver creates the `can_driver` node. Topics are created conditionally depending on the configuration of the driver at launch-time. Two services are created everytime *regardless of configuration*. The following table provides information on each topic.

|Topic Name | Data type (From `scion_types`) | Conditional 	|
| --------- | ----------------------------- | ----------- 	|
| brping1_dist | Datapoint.msg | Yes |
| brping1_confidence | Datapoint.msg | Yes |
| dvl_vel_x | Datapoint.msg | Yes |
| dvl_vel_y | Datapoint.msg | Yes |
| dvl_vel_z | Datapoint.msg | Yes |
| dvl_vel_e | Datapoint.msg | Yes |
| dvl_dist_bottom | Datapoint.msg | Yes |
| dvl_dist_1 | Datapoint.msg | Yes |
| dvl_dist_2 | Datapoint.msg | Yes |
| dvl_dist_3 | Datapoint.msg | Yes |
| dvl_dist_4 | Datapoint.msg | Yes |
| ms5837_depth | Datapoint.msg | Yes |
| ms5837_temp | Datapoint.msg | Yes |

| Service Name  | Service type (From `scion_types`) |
| ------------- | ---------------------------------	|
| send_can_raw | SendFrame.srv |
| send_can_command | CanCommand.srv |

***

## Modules

The following section provides brief instructions for developing and adding a device module to the driver. Prior to diving in, it is important to note the file structure you will be working with:

```
unified_can_driver/
 |
 |--include/
 |  |
 |  |--module/
 |  |  |
 |  |  |--*.hpp
 |  |  \--module.hpp      [*]
 |  |--*.hpp
 |  |--module_loader.hpp  [*]
 |  |--config.hpp         [*]
 |--src/
 |  |
 |  |--module/            [*]
 |  \--module_loader.cpp  [*]
 \--CMakeLists.txt        [*]

[*] Files/Directories that you will use.
```

### Writing a new module

Modules are classes responsible for handling information from their embedded counterparts. Some of the following tasks a typical module may perform are:
 - Polling the embedded system for sensor data updates
 - Publishing incoming data from the embedded system to ROS2
 - Creating ROS2 data Topics on startup
 - Sending device enable signals on startup

To begin, create a header file in the `unified_can_driver/include/module/` directory with the name `[module name].hpp`. Module filenames are formatted as lowercase, with spaces marked by underscores. (I.E: 'AB123 Example Module' would be`ab123_example_module.hpp`).

The header file contains the module's class declaration, it should inherit the `DeviceModule` class and *must* declare certain functions in order for your module to interact with the Module Loader. Below is a barebones example of the "Example" module:


*This header file will be included both in the module implementation and the module_loader.hpp. Any function, variable, etc. that your module requires will be declared in this header.*

#### example.hpp

```
//////////////// example.hpp ////////////////
#ifndef MOD_EXAMPLE_H	// These two lines are important!
#define MOD_EXAMPLE_H	// They prevent the compiler from including

#include "module.hpp"

// Use the 'Module' namespace, it contains all modules.
namespace Module
{
	class ExampleModule : public DeviceModule
	{
		public:
			// Your class constructor MUST contain a node pointer and mailbox
			// pointer, as these are passed in automatically by the module loader
			ExampleModule(rclcpp::Node*, Mailbox::MboxCan*);

			// These two functions are required and are inherited from
			// the DeviceModule parent class
			void dres_handle(struct can_frame*);
			void mod_init();	// This method is unused, but add it anyway

		private:
			// The timer_callback() is responsible for DREQ polling at a specified
			// interval. This is especially useful if you need automatic sensor data
			// updates from the embedded system. Both timer_callback() and dres_info()
			// are inherited from DeviceModule, and thus are required for your module
			// to compile.
			void timer_callback();
			void dres_info();

			// This is where you would normally place your module specific
			// variables. These could include ROS2 publishers, topic functions,
			// local variables, etc. Below are some typical examples.

			// Example ROS2 publisher
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_data1;
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_data2;
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_status1;
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_status2;
			
			// Example device topic function. These are typically included in
			// a jump table that is used in dres_handle() to efficiently execute
			// the correct code based on the contents of a received CAN frame.
			// The typical name scheme for these functions are
			// '[module name]_[topic/command]();' Topic functions return void.
			void example_nop();
			void example_data1();
			void example_data2();
			void example_status1();
			void example_status2();
	};
}

#endif
```
(This example is found in unified_can_driver/include/module/)

To break this down, let's first look at the namespace and class declaration:
```
namespace Module
{
	class ExampleModule : public DeviceModule
	{
```
All modules are stored within the `Module` namespace along with the Module Loader. This is purely for sanity and code organization. The next line declares a new class `ExampleModule` which inherits the parent class `DeviceModule`. DeviceModule contains methods and variables common across all modules, and has certain methods that *must* be implemented in derived classes in order for the loader to access them. For more information, see [DeviceModule](#DeviceModule).

Next are the public members of the class:
```
		public:
			ExampleModule(rclcpp::Node*, Mailbox::MboxCan*);

			void dres_handle(struct can_frame*);
			void mod_init();
```
Note that the constructor must only take two arguements: a pointer to a ROS2 Node (this should be the parent note), and a pointer to a mailbox object (the mailbox that the module will use to read/write to the CAN bus).

The `void dres_handle(struct can_frame*)` method is inherited from DeviceModule and is required by the module loader. It's implementation is more-or-less universal across modules.

The `void mod_init()` method is currently unused and will be removed in a later update. However it is still required to compile.

Then come the private members of the class:
```
		private:
			void timer_callback();
			void dres_info();

			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_data1;
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_data2;
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_status1;
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_status2;
			void example_nop();
			void example_data1();
			void example_data2();
			void example_status1();
			void example_status2();
```
The methods `timer_callback()` and `dres_info()` are both required by the module loader. `timer_callback()` contains code (such as a sensor data request) that is executed on a set interval determined by the implementation of the module. The `rclcpp::Publisher` declarations create multiple ROS2 publishers that will be updated with data from the embedded system. Note that each publisher has a corresponding topic method (`example_nop()`, `example_data1()`, etc.) that determine *what* and *how* the data will be formatted & published. 

This class is implemented in `example.cpp`:

#### example.cpp

```
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
```
(This example is found in unified_can_driver/src/module/)

Module constructors should accept a ROS2 node pointer and a mailbox pointer, which are then passed to the inherited constructor. The inherited constructor also takes some hard-coded values that determine how the module should be initialized. These values (in order) are: the `parent ROS2 node`, a `mailbox pointer`, `timer_callback() interval (ms)`, `embedded device ID`, `timer_callback() enable`, `dres_handle() enable`, and the `topic method count` (More information can be found in the [DeviceModule](#DeviceModule) section). These *must* be set for every module, and the device ID should not conflict with any other existing module:

```
#include "example.hpp"

using namespace Module;

ExampleModule::ExampleModule(rclcpp::Node* ctx, Mailbox::MboxCan* mb)
	: DeviceModule(
		ctx, mb, 10, static_cast<uint8_t>(CanDriver::Device::Example::ID),
		true, true, 6)
```

The first section of the constructor is responsible for initializing the topic jump table and ROS2 publishers. Note that each member function pointer must be cast to the custom type `<topic_ptr_t>` priot to being added to the jump table. The index of each pointer should match it's counterpart in the embedded node:

```
	// Create member function jump table
	module_topic_ptr_array[0] = static_cast<topic_ptr_t>(&ExampleModule::dres_info);
	module_topic_ptr_array[1] = static_cast<topic_ptr_t>(&ExampleModule::example_nop);
	module_topic_ptr_array[2] = static_cast<topic_ptr_t>(&ExampleModule::example_data1);
	module_topic_ptr_array[3] = static_cast<topic_ptr_t>(&ExampleModule::example_data2);
	module_topic_ptr_array[4] = static_cast<topic_ptr_t>(&ExampleModule::example_status1);
	module_topic_ptr_array[5] = static_cast<topic_ptr_t>(&ExampleModule::example_status2);

	// Initialize ROS2 data publishers
	topic_data1		= node_context->create_publisher<scion_types::msg::Datapoint> ("example_data1");
	topic_data2		= node_context->create_publisher<scion_types::msg::Datapoint> ("example_data2");
	topic_status1	= node_context->create_publisher<scion_types::msg::Datapoint> ("example_status1");
	topic_status2	= node_context->create_publisher<scion_types::msg::Datapoint> ("example_status2");
```

The next section depends on the configuration of the embedded device. If a device (including all motors and moving parts) defaults to a disabled state on the embedded node, a wakeup frame should be sent to notify the embedded system that that device is enabled. To achieve this, it is possible to hard-code a wakeup routine in the module constructor as follows:

```
	struct can_frame init_frame;
	memset(&init_frame, 0, sizeof(struct can_frame));	// Fill the structure with zeros
	init_frame.can_dlc = 8;
	// Macro representing the STOW command on the embedded system
	init_frame.can_id = static_cast<uint8_t>(CanDriver::Command::STOW);
	init_frame.data[0] = this->module_device_id;		// ID specified in constructor
	// write generated CAN frame to the bus
	Mailbox::MboxCan::write_mbox(this->mailbox_ptr, &init_frame);
```

The `timer_callback()` method function is inherited and initialized from the parent DeviceModule, and allows for custom behavior (i.e. sensor polling, watchdog, etc.) to execute at a specified interval. The example provides an implementation that polls the `EXAMPLE::TOPIC_DATA` topic at a set interval (10ms):

```
void ExampleModule::timer_callback()
{
	if(do_device_polling && this->module_enable_ti)
	{
		struct can_frame poll_frame;
		memset(&poll_frame, 0, sizeof(struct can_frame));
		// Set ID to data request command
		poll_frame.can_id = static_cast<uint8_t>(CanDriver::Command::DREQ);
		poll_frame.can_dlc = 8;
		poll_frame.data[0] = this->module_device_id;
		// Device topic to retrieve newest data, for more information
		// see the "Commands & Macros" sections of the documentation
		poll_frame.data[2] = static_cast<uint8_t>(CanDriver::Device::EXAMPLE::TOPIC_DATA);
		Mailbox::MboxCan::write_mbox(mailbox_ptr, &poll_frame);
	}
}
```

All modules that retrieve and decode data from the embedded node *must* have some implementation of the `dres_handle()` method. If the module loader receives a DRES frame, it passes control to the correct module's `dres_handle()` method. From there, the previously mentioned jump table is used to access the correct topic member function. An implementation of this behaviors is:

```
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
```

The next two methods, `mod_init()` and `dres_info()` are also implementations of methods inherited from the DeviceModule parent class. `dres_info()` is used to handle a status response from the embedded system after enabling a device. Currently, there is no implemented use for the resulting status code.  `mod_init()` is unused and will be removed in a later update.

```
void ExampleModule::mod_init() {}	// Unused

void ExampleModule::dres_info()
{
	memcpy(&module_hw_info, this->frame->data+(sizeof(char)*4), sizeof(float) );
	RCLCPP_INFO(node_context->get_logger(),
		"[ExampleModule::dres_info] Received info from hw.");
}

```

The rest of the example implementation consists of topic member functions- the various destinations within the module's jump table. Since most of these member functions only need to push data to ROS2, they can be reduced to boilerplate code that:
 1. Extracts the final 32 bits of the CAN frame
 2. casts the extracted data into a float
 3. Creates and populates a `Datapoint` message (from the `scion_types` package)
 4. Pushes the new message to a ROS2 topic.

```
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
```


### Implementing a new nodule into the driver

Once the module itself is written, it must be added to the module loader, configuration bitfield, Makefile, and (optional, but recommended) the macro header file. Information for each step can be found below.

#### Adding a module to module_loader.cpp

The `ModuleLoader` class is responsible for loading modules on launch based on the driver's configuration. By declaring a module as a pointer, it is possible to initialize it to either a new instance of itself or `nullptr`. This allows for the user to disable certain modules through a bitfield parameter instead of editing and recompiling the package.

To add a new module to the loader class, first include the module's header file in `module_loader.hpp`:

```
/******** Modules ********/
#include "ms5837.hpp"
#include "dvl.hpp"
#include "brlight.hpp"
#include "brping1.hpp"
#include "example.hpp" // new module
/*************************/
```

Still in the header file, declare a pointer to the module as a private member:

```
/* Modules */
MS5837Module* ms5837;
DVLModule* dvl;
BRLIGHTModule* brlight;
BRPING1Module* brping1;
ExampleModule* example;		// new module
```

With the new module pointer declared in the header, it needs to be loaded in `module_loader.cpp`. To conditionally load a module in `module_loader.cpp`, the `load_module(Device Module*, uint8_t)` template method is utilized. The syntax for this method is `load_module<[ModuleType]>([Module pointer], [Module Enable bit])` Modules are loaded during the initialization of the ModuleLoader class (in the constructor):

*See next section for implementing [Module Enable bit]*

```
/* Conditionally load modules*/
load_module<DVLModule>(dvl, MODULE_DVL_ENABLE);
load_module<MS5837Module>(ms5837, MODULE_MS5837_ENABLE);
load_module<BRLIGHTModule>(brlight, MODULE_BRLIGHT_ENABLE);
load_module<BRPING1Module>(brping1, MODULE_BRPING1_ENABLE);
load_module<ExampleModule>(example, MODULE_EXAMPLE_ENABLE);		// new module
```

#### Adding your module to the configuration bitfield

Currently, a 32-bit unsigned integer is used to determine which modules should be enabled/disabled on launch. By this design, the driver is capable of having up to 32 unique modules in any combination of enabled/disabled states. By default, the driver launches with all modules enabled (`0xFFFFFFFF`). See [Configuration](#Configuration) for more information.

In order to add a new module to the configuration field, edit `include/config.hpp` and create a preprocessor definition with the name: '`MODULE_[MODULE NAME]_ENABLE`'. It is important to ensure that your module's enable bit does not conflict with any existing definition in the bitfield:

```
/* Enable words for HW modules */
#define MODULE_MS5837_ENABLE        0b00000000000000000000000000000001
#define MODULE_DVL_ENABLE           0b00000000000000000000000000000010
#define MODULE_EMBEDSYS_ENABLE      0b00000000000000000000000000000100
#define MODULE_PWRSYS_ENABLE        0b00000000000000000000000000001000
#define MODULE_BRLIGHT_ENABLE       0b00000000000000000000000000010000
#define MODULE_BRPING1_ENABLE       0b00000000000000000000000000100000
#define MODULE_EXAMPLE_ENABLE       0b00000000000000000000000001000000	// new module; note the unique value.
```

#### Adding your module to CMakeLists.txt

With your module implemented, it needs to be added to the build system. The header is automatically included, so the only needed change is within the `add_executable()` section of the `CMakeLists.txt` file (located in the package root directory). Simply add the path to your module implementation below the existing modules:

```
# target executable
add_executable(
	unified_can_driver
	src/base_driver/unified_can_driver.cpp
	src/base_driver/can_send_service.cpp
	src/base_driver/can_mailbox_topic.cpp
	src/base_driver/send_command_service.cpp
	src/mbox_can.cpp
	src/module_loader.cpp
# Modules:
	src/module/dvl.cpp
	src/module/ms5837.cpp
	src/module/brlight.cpp
	src/module/brping1.cpp
	src/module/module.cpp
	src/module/example.cpp		// new module
)
```

#### Adding your module to the macro header

The macro header, `include/can_commands.hpp`, is a header file with constant definitions of commands, device IDs, and topic IDs, present on the embedded node. The purpose of this file is to simplify driver updates in the event of changes on the embedded system, as it abstracts all device specific information into a single file. This file is also intended to be copied to any other ROS2 packages that required interaction with the CAN driver, as the macros defined in the header can be used to populate a `SendCanCommand` service call. The namespace structure is as follows:

 * CanDriver (namesapce):
  * Command (enum)
  * Device (namespace):
    * EMBDSYS (enum)
    * PWRSYS (enum)
    * WAYFDVL (enum)
    * ...

All commands, device IDs, and device topics **MUST** exactly match those on the protocol sheet & embedded system. In order to implement a new device module into the header, these values must be known. Begin by implementing the following format in the Device namespace (bottom of file):

```
/**************************************************/

		enum class EXAMPLE
		{
			ID              = [UNIQUE DEVICE ID],
			INFO            = 0x0000,
			NOP             = 0x0001,
			TOPIC_DATA1     = 0x0002,
			TOPIC_DATA2     = 0x0003,
			TOPIC_STATUS1   = 0x0004,
			TOPIC_STATUS2   = 0x0005,
			TOPIC_DATA_ALL  = 0x0010
		};
```

Note that the date at the top of the file marks when it was last updated to match the protocol sheet. If you make changes to this file, updated this date accordingly.
### DeviceModule

The `DeviceModule` class is the parent of all modules, it defines certain behaviors that child classes must implement and adhere to in order for the module loader to interact with them properly. Below is a brief description of all members of this class.

#### The Constructor

The constructor should be called from all children constructor, and requires certain arguements in order to properly initialize the module. These arguements are:

```
DeviceModule(
	rclcpp::Node*,      /* node_context */
	Mailbox::MboxCan*,  /* mailbox_ptr */
	uint8_t,            /* module_timer_len */
	uint16_t,           /* module_device_id */
	bool,               /* module_enable_ti */
	bool,               /* module_enable_dres*/
	uint8_t             /* module_topic_ct */
);
```
| Arguement name | Type | Description |
| -------------- | ---- | ----------- |
| node_context | rclcpp:Node* | Pointer to the driver's ROS2 Node object, used for initialization and data publishing. |
| mailbox_ptr | Mailbox::MboxCan* | Pointer to a shared CAN mailbox, used for reading and writing CAN frames to the embedded system. |
| module_timer_len | uint8_t | Timer interval in milliseconds, this is the frequency of which timer_callback() will be executed |
| module_device_id | uint_16_t | Embedded device ID, should be retrieved from `can_commands.hpp` |
| module_enable_ti | bool | Flag designating whether timer_callback() will be enabled. |
| module_enable_dres | bool | Flag designating whether the module supports dres_handle() |
| module_topic_ct | uint8_t | Number of topic member functions, used to initialize the jump table. |

***There are several member variables both public and private that all modules inherit. They are described below:***

| Variable name | Type | Access specifier | Description |
| --- | --- | --- | --- |
| module_timer_len | uint8_t | public | timer_callback() interval |
| module_device_id | uint16_t | public | Module-specific device ID, hardcoded in module implementation |
| module_enable_ti | bool | public | Flag designating whether timer_callback() is supported | 
| module_enable_dres| bool | public | Flag designating whether dres_handle() is supported | 
| module_topic_ct | uint8_t | public | Length of the module's jump table, initialized in inherited constructor. | 
| module_hw_info| uint32_t | public | Status of devicee from embedded system, set by DRES from embedded system on topic 0x0000 |
| mailbox_ptr | Mailbox::MboxCan* | private | Pointer to shared CAN mailbox, used by modules to read and write to the bus. |
| node_context | rclcpp::Node* | private | Pointer to the driver's ROS2 Node object, allows modules to interact with ROS2. |
| poll_timer | rclcpp::TimerBase::SharedPtr | private | Timer object used to initialize timer_callback() polling |
| frame | struct can_frame* | private | Pointer to frame received by `ModuleLoader`. Used for decoding topic and data |
| module_topic_ptr_array | topic_ptr_t* | private | Jump table of member functions (topic_ptr_t AKA member function of DeviceModule/children) |

***The DeviceModule class provide several virtual methods that must be implemented by derived modules, the following is a description of those methods*** 

| Method name | Return type | Arguements | Acces Specifier | Description |
| --- | --- | --- | --- | --- |
| mod_init() | void | | public | Unused, will be removed in a later update. | 
| dres_handle()| void | struct can_frame* | public | DRES handler, called by the module loader which passes in the correct `can_frame` as a pointer. This method should handle topic decoding of any received CAN frames. | 
| timer_callback() | void |  | private | Timer callback for `poll_timer`. Called on a custom interval to execute repeating code (I.E: sensor polling) |
| dres_info() | void |  | private | Topic handler for topic 0x000 (INFO), should store the embedded device status in `module_hw_info` |
***

## Development
