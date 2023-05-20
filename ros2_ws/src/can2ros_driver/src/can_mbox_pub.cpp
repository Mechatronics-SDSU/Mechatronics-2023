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

#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/can_frame.hpp"
#include "scion_types/srv/send_frame.hpp"
#include "mbox_can.hpp"
#include "can_mbox_pub.hpp"


#define MBOX_INTERFACE "vcan0"

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
CanMailboxPublisher::CanMailboxPublisher()
: Node("can_mbox_publisher")
{
	strncpy(ifr.ifr_name, MBOX_INTERFACE, sizeof(&MBOX_INTERFACE));
	timer_ = this->create_wall_timer(
		10ms, std::bind(&CanMailboxPublisher::timer_callback, this));	
	rclcpp::on_shutdown(std::bind(&CanMailboxPublisher::shutdown_node, this));
	RCLCPP_INFO(this->get_logger(), "[MBOX INFO] Starting MBOX publisher node.");

	can_service = new CanSendService((rclcpp::Node*)this, &ifr);
	emergency_mb = new MailboxTopic((rclcpp::Node*)this, &ifr, "emergency", {0x000,0xff0});
	heartbeat_mb = new MailboxTopic((rclcpp::Node*)this, &ifr, "heartbeat", {0x010,0xff0});
	dres_mb = new MailboxTopic((rclcpp::Node*)this, &ifr, "dres", {0x021,0xff1});

}

/*
 * timer_callback()
 * 		This is called (currently) every 10ms. In order for a mailbox to publish
 * 		data to ros2, it's mailbox_cb() function must be run here.
 */		
void CanMailboxPublisher::timer_callback()
{
	emergency_mb->mailbox_cb();
	heartbeat_mb->mailbox_cb();
	dres_mb->mailbox_cb();
}


/*
 * shutdown_node()
 * 		This is called upon the destruction of the Ros2 Node (end of process). In order for a mailbox to
 * 		properly close it's socket, the shutdown_cb() of each mailbox must be run here.
 */	
void CanMailboxPublisher::shutdown_node()
{
	emergency_mb->shutdown_cb();
	heartbeat_mb->shutdown_cb();
	dres_mb->shutdown_cb();

}



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CanMailboxPublisher>());
	rclcpp::shutdown();
	return 0;
}
