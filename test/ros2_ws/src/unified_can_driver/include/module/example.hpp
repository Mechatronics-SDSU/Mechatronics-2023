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
			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_status1;			rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr topic_status2;

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
