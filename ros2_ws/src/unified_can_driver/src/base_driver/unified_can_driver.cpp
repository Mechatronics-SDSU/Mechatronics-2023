/* Connor Larmer
 * Jan 24 2023
 * ####CAN data publisher node####
 * This program uses the mbox_can library to create multiple filtered mailboxes, then directs any frames that arrive
 * in those mailboxes to corresponding Ros2 topics (i.e. "emergency_mb_pub", "motor_mb_pub", etc.). Frames are published
 * as custom messages found in the "scion_types" Ros2 package. Currently, frames are published based on a
 * callback timer-- this is subject to change. If a process needs to send a CAN frame to the embedded system, the /send_frame
 * ROS2 service exists to do precisely that.
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string.h>
#include <algorithm>
#include <net/if.h>
#include <vector>

#include "config.hpp"
#include "unified_can_driver.hpp"

using namespace std;

/*
	The following code creates multiple filtered CAN mailboxes to divide
	sort and separate incoming traffic from the teensy. This filtering
	is done with a bitmask-- for simplicity we are dedicating 16 addresses
	to each catagory (i.e. 0x000 -> 0x00f where the bitmask would look like {0x000,0xff0}).
	A ros2 topic is created for each mailbox with a naming scheme of
	"[mailbox name]_mb_pub" and a message type with int32_t for frame ID
	and an array of 8-bit values to store the CAN data.

	The Send Service is initialized here as well. This should only ever be done once! Both
	The CanSendService object and the mailbox objects require a pointer to this class in order
	to properly integrate with Ros2.
 */
UnifiedCanDriver::UnifiedCanDriver()
: Node("can_driver")
{

	/* User Configurable Parameters */
	this->declare_parameter("do_module_polling", true);
	this->declare_parameter("can_bus_interface", "vcan0");
	this->declare_parameter("module_bitfield", 0xFFFF);

	GlobalSettings::module_enabled_field = this->get_parameter(
		"module_bitfield").get_parameter_value().get<uint16_t>();
	GlobalSettings::can_bus_interface = this->get_parameter(
		"can_bus_interface").get_parameter_value().get<std::string>();
	GlobalSettings::do_device_polling = this->get_parameter(
		"do_module_polling").get_parameter_value().get<bool>();

	RCLCPP_INFO(this->get_logger(), "################################################");
	RCLCPP_INFO(this->get_logger(), "[UnifiedCanDriver] Unified CAN Driver ver. 0.0.4");
	RCLCPP_INFO(this->get_logger(), "[UnifiedCanDriver] [Config] Connected Interface: %s", GlobalSettings::can_bus_interface);
	RCLCPP_INFO(this->get_logger(), "[UnifiedCanDriver] [Config] Module Enable Field: %d", GlobalSettings::module_enabled_field);
	RCLCPP_INFO(this->get_logger(), "[UnifiedCanDriver] [Config] POLLING ENABLED: %s", GlobalSettings::do_device_polling ? "true":"false" );
	RCLCPP_INFO(this->get_logger(), "[UnifiedCanDriver] Starting can_driver node.");

	char interface = GlobalSettings::can_bus_interface.c_str();
	strncpy(ifr.ifr_name, interface,
		sizeof(&interface));
	_can_timer = this->create_wall_timer(
		10ms, std::bind(&UnifiedCanDriver::can_timer_callback, this));	
	rclcpp::on_shutdown(std::bind(&UnifiedCanDriver::shutdown_node, this));

	RCLCPP_INFO(this->get_logger(), "[UnifiedCanDriver] Starting module loader. . .");
	module_loader = new Module::ModuleLoader((rclcpp::Node*)this, &ifr);
	
	RCLCPP_INFO(this->get_logger(), "[UnifiedCanDriver] Starting CAN raw service. . .");
	can_service = new CanSendService((rclcpp::Node*)this, &ifr);

	RCLCPP_INFO(this->get_logger(), "[UnifiedCanDriver] Starting CAN command service. . .");
	command_service = new SendCommandService((rclcpp::Node*)this, &ifr);

	_dres_mb = new Mailbox::MboxCan(&ifr, "dres");
	Mailbox::MboxCan::set_filter(_dres_mb, {0x021, 0xfff});
	RCLCPP_INFO(this->get_logger(), "################################################");

	//////// Leftover from an earlier part of the driver. ////////
	// emergency_mb = new MailboxTopic((rclcpp::Node*)this, &ifr, "emergency", {0x000,0xff0});
	// heartbeat_mb = new MailboxTopic((rclcpp::Node*)this, &ifr, "heartbeat", {0x010,0xff0});
	// dres_mb = new MailboxTopic((rclcpp::Node*)this, &ifr, "dres", {0x021,0xff1});
}

/*
 * timer_callback()
 * 		This is called (currently) every 10ms. In order for a mailbox to publish
 * 		data to ros2, it's mailbox_cb() function must be run here.
 */		
void UnifiedCanDriver::can_timer_callback()
{
	struct can_frame dres_frame;
	if(Mailbox::MboxCan::read_mbox(_dres_mb, &dres_frame) == 0)
	{
		module_loader->module_decode_dres(&dres_frame);
	}
	// emergency_mb->mailbox_cb();
	// heartbeat_mb->mailbox_cb();
	// dres_mb->mailbox_cb();
}


/*
 * shutdown_node()
 * 		This is called upon the destruction of the Ros2 Node (end of process). In order for a mailbox to
 * 		properly close it's socket, the shutdown_cb() of each mailbox must be run here.
 */	
void UnifiedCanDriver::shutdown_node()
{
	Mailbox::MboxCan::close_mbox(_dres_mb);
	delete _dres_mb;
	//delete dres_decoder;
	delete module_loader;
	// emergency_mb->shutdown_cb();
	// heartbeat_mb->shutdown_cb();
	// dres_mb->shutdown_cb();

}



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UnifiedCanDriver>());
	rclcpp::shutdown();
	return 0;
}