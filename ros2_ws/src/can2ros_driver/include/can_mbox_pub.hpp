/* Connor Larmer
 * 1.24.2023
 * Header file for CAN -> ROS2 2 way communication node.
 *
 */
#ifndef CAN_MAILBOX_PUB_H
#define CAN_MAILBOX_PUB_H
#include "scion_types/srv/send_frame.hpp"
#include "scion_types/msg/can_frame.hpp"


/*
 * MailboxTopic Class:
 * 		This class exists to simplify the process of adding
 * 		Mailboxes to ROS2. An object created from this class will
 * 		automatically connect to the CAN interface and funnel data
 * 		that passes through it's filter to a unique ROS2 topic. Each
 * 		instance of this class has it's own timer and shutdown callback
 * 		functions that MUST be linked into the parent node's corresponding
 * 		callbacks to publish data and properly shutdown the CAN socket
 * 		(mailbox_cb() and shutdown_cb()).
 */
class MailboxTopic
{
	public:
		MailboxTopic(
			rclcpp::Node*,
			struct ifreq*,
			std::string,
			struct can_filter
		);
		void mailbox_cb();
		void shutdown_cb();
	private:
		rclcpp::Node* node_context;
		Mailbox::MboxCan* mailbox;
		struct can_frame in_frame;
		std::string topic_name;
		rclcpp::Publisher<can_msg_interfaces::msg::CanFrame>::SharedPtr mailbox_pub;
};

/*
 * CanSendService Class:
 * 		This defines the structure of the service responsible for sending CAN
 * 		data from ROS2 to the embedded system. Only one instance of this class
 * 		should exist at the same time. The service uses the SendFrame service
 * 		defined in can_msg_interfaces/srv/SendFrame.srv
 */
class CanSendService
{
	public:
		CanSendService(rclcpp::Node*, struct ifreq*);
	private:
		void service_handler(
			const std::shared_ptr<can_msg_interfaces::srv::SendFrame::Request>,
			std::shared_ptr<can_msg_interfaces::srv::SendFrame::Response>);

		rclcpp::Node* node_context;
		Mailbox::MboxCan* out_box;
		rclcpp::Service<can_msg_interfaces::srv::SendFrame>::SharedPtr send_service;	
};

/*
 * CanMailboxPublisher Class:
 * 		This class is where mailboxes and services are handled. It inherits rclcpp::Node,
 * 		which is then passed by pointer into subclasses to allow each mailbox object to
 * 		handle it's own integration with ros.
 */
class CanMailboxPublisher : public rclcpp::Node
{
	public:
		CanMailboxPublisher();
	private:
		void timer_callback();
		void shutdown_node();

		struct ifreq ifr;
		rclcpp::TimerBase::SharedPtr timer_;
		MailboxTopic* emergency_mb;
		MailboxTopic* heartbeat_mb;
		MailboxTopic* sensors_mb;
		MailboxTopic* dres_mb;
		CanSendService* can_service;
};

#endif
