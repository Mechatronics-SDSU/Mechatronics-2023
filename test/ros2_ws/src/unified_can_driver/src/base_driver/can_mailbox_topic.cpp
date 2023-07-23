/* Connor Larmer
 * Jan 5 2023
 */
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include <net/if.h>
#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/can_frame.hpp"
#include "mbox_can.hpp"
#include "unified_can_driver.hpp"

using namespace std;
/*
 * MailboxTopic class Implementation
 */

/* Constructor for MailboxTopic class. This class is used
 * to create mailboxes wrapped with their own Ros2 topics- 
 * allowing for the number of mailbox topics being published
 * to be scaled depending on the program's needs. A context
   needs to be passed into the constructor, which will then
   be used for topic creation, publishing, and logging.
 */
MailboxTopic::MailboxTopic(
	rclcpp::Node* context,
	struct ifreq* ifr,
	std::string name,
	struct can_filter filter)
{
	mailbox = new Mailbox::MboxCan(ifr,name);
	node_context = context;
	topic_name = name+"_mb_pub";
	Mailbox::MboxCan::set_filter(mailbox, filter);

	mailbox_pub = node_context->create_publisher<scion_types::msg::CanFrame>(
		topic_name,
		10);
	
	RCLCPP_INFO(
		node_context->get_logger(),
		"[MailboxTopic] Topic <%s> Created.",
		mailbox->mbox_name->c_str());
}
/* MailboxTopic update callback. This function is intended to be called
 * whenever the mailbox should be updated, whether by timer or otherwise.
 * On update, the mailbox is read. Any data recieved from the CAN bus is
 * then piped to the corresponding ros2 topic.
 */
void MailboxTopic::mailbox_cb()
{
	auto mailbox_msg = scion_types::msg::CanFrame();
	if(Mailbox::MboxCan::read_mbox(mailbox, &in_frame) == 0 )
	{
		mailbox_msg.can_id = in_frame.can_id;
		std::copy(std::begin(in_frame.data),
			std::end(in_frame.data),
			std::begin(mailbox_msg.can_data));
		mailbox_pub->publish(mailbox_msg);
		RCLCPP_INFO(
			node_context->get_logger(),
			"[MailboxTopic::mailbox_cb] <%s>: Published CAN [%03x#]",
			mailbox->mbox_name->c_str(),
			in_frame.can_id);
			
	}
}
/* MailboxTopic shutdown callback. This function is intended to be called
 * on the destruction of the Ros2 node it resides in. This is so the Mailbox
 * is able to properly close it's socket connection before the program terminates.
 */
void MailboxTopic::shutdown_cb()
{
	Mailbox::MboxCan::close_mbox(mailbox);
}
