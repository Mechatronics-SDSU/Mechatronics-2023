
#include <memory>
#include <string>
#include <algorithm>

#include "send_command_service.hpp"

SendCommandService::SendCommandService(
	rclcpp::Node* ctx,
	struct ifreq* ifr)
{
	node_context = ctx;
	out_mb = new Mailbox::MboxCan(ifr,"command_out");
	service = node_context->create_service<scion_types::srv::CanCommand>(
		"send_can_command",
		std::bind(&SendCommandService::service_handler,
			this,
			std::placeholders::_1,
			std::placeholders::_2));
	RCLCPP_INFO(node_context->get_logger(), "[SendCommandService] CAN command service started.");
}


void SendCommandService::service_handler(
	const std::shared_ptr<scion_types::srv::CanCommand::Request> request,
	std::shared_ptr<scion_types::srv::CanCommand::Response> response)
{
	int status = 0;
	if(out_mb->can_sock)
	{
		struct can_frame send_frame;
		auto device = request->device;
		auto topic = request->topic;
		auto command = request->command;
		memset(&send_frame, 0, sizeof(struct can_frame));
		send_frame.can_id = command;
		send_frame.can_dlc = 8;

		memcpy(&send_frame.data[0], &device, sizeof(char)*2 );
		memcpy(&send_frame.data[2], &topic, sizeof(char)*2 );
//		memcpy(&send_frame.data[4], (char)request->data, sizeof(char)*4 );

		std::copy(std::begin(request->data),
						std::end(request->data),
						std::begin(send_frame.data)+4);

		if(Mailbox::MboxCan::write_mbox(out_mb, &send_frame) < 0 )
		{
			status = -1;
		}
		else
		{
			RCLCPP_INFO(node_context->get_logger(), "[SendCommandService::service_handler] Service request fulfilled.");
		}
	}
	else
	{
		status = -1;
	}
	response->status = status;	
}
