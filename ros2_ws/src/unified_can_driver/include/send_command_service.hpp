#ifndef CAN_COMMAND_H
#define CAN_COMMAND_H

#include "rclcpp/rclcpp.hpp"
#include "scion_types.hpp"
#include "mbox_can.hpp"
class SendCommandService
{
	public:
		SendCommandService(rclcpp::Node*, struct ifreq*);
	private:
		void service_handler(
			const std::shared_ptr<scion_types::srv::CanCommand::Request>,
			std::shared_ptr<scion_types::srv::CanCommand::Response>);

		rclcpp::Node* node_context;
		Mailbox::MboxCan* out_mb;
		rclcpp::Service<scion_types::srv::CanCommand>::SharedPtr service;
};

#endif
