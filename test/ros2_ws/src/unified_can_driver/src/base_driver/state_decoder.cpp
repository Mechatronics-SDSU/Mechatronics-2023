#include "state_decoder.hpp"


StateDecoder::StateDecoder(rclcpp::Node* ctx, struct ifreq* ifr)
{
	node_context = ctx;
	
	memset(&filter, 0, sizeof(filter));
	memset(&latest, 0, sizeof(latest));
	
	filter[0].can_id = 0x000;
	filter[1].can_id = 0x001;
	filter[2].can_id = 0x007;
	filter[3].can_id = 0x00A;
	
	filter[0].can_mask = 0xfff;
	filter[1].can_mask = 0xfff;
	filter[2].can_mask = 0xfff;
	filter[3].can_mask = 0xfff;

	mbox = new Mailbox::MboxCan(ifr, "sub_state");
	//Mailbox::MboxCan::set_filter(mbox, &filter); 
	// ^ This doesn't work because array shenanigans :(
	// So we cheat like this:
	setsockopt(mbox->can_sock, SOL_CAN_RAW,
		CAN_RAW_FILTER, &filter, sizeof(filter));	

	pub_topic = node_context->create_publisher<scion_types::msg::SubState>
		("submarine_state", 10);
	
	leak = estop = false;
	curr_state = HOST_STATE::MANUAL;

	RCLCPP_INFO(node_context->get_logger(),
		"[StateDecoder] Sub State Decoder started.");
}

StateDecoder::~StateDecoder()
{
	Mailbox::MboxCan::close_mbox(mbox);
	delete mbox;
}

void StateDecoder::check_state()
{
	if(Mailbox::MboxCan::read_mbox(mbox, &latest) == 0)
	{
		switch(latest.can_id)
		{
			case 0x000:
				estop = true;
				RCLCPP_INFO(node_context->get_logger(),
					"[StateDecoder::check_state] !!! KILL SIGNAL RECEIVED.");
				break;
			case 0x001:
				leak = true;
				RCLCPP_INFO(node_context->get_logger(),
					"[StateDecoder::check_state] !!! LEAK SIGNAL RECEIVED.");
				break;
			case 0x007:
				if(latest.can_dlc > 0)
					curr_state = (HOST_STATE)latest.data[0];
				RCLCPP_INFO(node_context->get_logger(),
					"[StateDecoder::check_state] Host mode changed to %02i",
					(int)curr_state);
				break;
			case 0x00A:
				leak = false; 
				estop = false;
				RCLCPP_INFO(node_context->get_logger(),
					"[StateDecoder::check_state] All Clear received.");				
				break;
		}
		auto state_msg = scion_types::msg::SubState();
		state_msg.shutdown 		= (char)estop;
		state_msg.leak_detect 	= (char)leak;
		state_msg.host_mode 	= (char)curr_state;
		pub_topic->publish(state_msg);
	}
}