/* Connor Larmer
 * Jan 5 2023
 * Implementation of Ros2 -> Embedded System CAN Service
 * 		This service is responsible for sending CAN requests
 * 		from within Ros2 to hardware.
 */
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "mbox_can.hpp"
#include "unified_can_driver.hpp"


/*
 * CanSendService(rclcpp::Node*, struct ifreq*)
 * 		When initializing the service, a private Mailbox with no filter
 * 		is created to handle outgoing CAN frames. This is used by the 
 * 		service_handler() function that is called whenever another Ros2
 * 		process requests a frame to be sent. The service is then created
 * 		off of the parent "context" node passed by pointer into the constructor.
 * 		This is the service that will trigger the service_handler() callback on request.
 */
CanSendService::CanSendService(
	rclcpp::Node* context,
	struct ifreq* ifr)
{
	node_context = context;
	out_box = new Mailbox::MboxCan(ifr, "orin_output");
	send_service = node_context->create_service<scion_types::srv::SendFrame>(
	"send_can",
	std::bind(&CanSendService::service_handler,
		this,
		std::placeholders::_1,
		std::placeholders::_2));
	RCLCPP_INFO(node_context->get_logger(), "[CanSendService] CAN Output service started.");
}

/*
 * service_handler(request, response)
 * 		This is the service_handler callback responsible for pushing new frames
 * 		onto the CAN bus. As a private function, this should NEVER be called explicitly
 * 		in code- only by Ros2 through a callback.
 */
void CanSendService::service_handler(
	const std::shared_ptr<scion_types::srv::SendFrame::Request> request,
	std::shared_ptr<scion_types::srv::SendFrame::Response> response)
{
	int status = 0;
	if(out_box->can_sock)
	{
		struct can_frame send_frame;
		send_frame.can_id = request->can_id;
		send_frame.can_dlc = request->can_dlc;
		std::copy(std::begin(request->can_data),
				std::end(request->can_data),
				std::begin(send_frame.data));
		if (Mailbox::MboxCan::write_mbox(out_box,&send_frame) < 0)
		{
			status = -1;
		}
		else
		{
			RCLCPP_INFO(node_context->get_logger(), "[CanSendService::service_handler] Service Request to send %03x",request->can_id);
		}
	}
	else
	{
		status = -1;
	}
	response->status = status;
}
