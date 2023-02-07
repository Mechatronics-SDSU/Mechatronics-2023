/* Connor Larmer
 * 1.19.23 
 * Filtered CAN mailbox library for Junebug (C++ port)
 * 
 * This header file defines the MboxCan class, an attempt at
 * implementating a scalable filtered mailbox system for recieving
 * CAN frames from Junebug's Teensy microcrontoller.
 *
 * 	MboxCan(struct ifreq*, std::string):
 * 			This Constructor is used to create and configure the socketCAN
 *			connection.
 *				struct ifreq* 		# ifreq with inferface information
 *				std::string			# const with human-readale mailbox name
 *
 * 	set_filter(MboxCan*, struct can_filter):
 *			A mailbox can be configured without filtering in order to
 *			accept all data on the CAN bus, however in a scenario where
 *			only a certain range of frame IDs are desired, this function
 *			enables filtering on the targeted mailbox. This function has
 * 			no return.
 *				MboxCan* 			# Mailbox to be configured
 *				struct can_filter 	# can_filter structure. an array like {can_id,can_mask} [more info in <can.h> docs]
 *
 * 	read_mbox(MboxCan*, struct can_frame*):
 *			This function is used to read data from a mailbox. If there
 *			is that matches the filter, it is copied to the can_frame struct
 *			pointer. If this function completes successfully, it will return 0,
 *			-1 otherwise (mailboxes are NON-BLOCKING. If there is no data to be
 *			read, the function will return -1 with a "recource temporarily unavailible"
 *			error).
 *				MboxCan* 			# Mailbox to be read
 *				struct can_frame* 	# can_frame pointer to load recv data into [details on can_frame found in <can.h> docs]
 *
 * write_mbox(MboxCan*, struct can_frame*):
 *			This function is used to write data to a mailbox/CAN bus. If successful,
 *			the function will return 0. If the write fails, then the function will
 *			return -1.
 *				MboxCan*			# Mailbox to write to
 *				struct can_frame*	# can_frame pointer holding the data to be written.
 *
 *	close_mbox(MboxCan*):
 *			This function attempts to close the socket connection in order
 *			to free resources. If successful the function returns 0, -1 otherwise.
 *				MboxCan* 			# Mailbox to be closed
 *
 * [https://docs.huihoo.com/doxygen/linux/kernel/3.7/can_8h.html] <can.h> Reference
 */


#ifndef MBOX_CAN_H
#define MBOX_CAN_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <string>

namespace Mailbox
{
	class MboxCan
	{
		public:
			MboxCan(
				struct ifreq*,
				std::string
			);

			static void set_filter(
				MboxCan*,
				struct can_filter);

			static int read_mbox(
				MboxCan*,
				struct can_frame*);
				
			static int write_mbox(
				MboxCan*,
				struct can_frame*
			);
			static int close_mbox(MboxCan*);
			std::string* mbox_name;
			int can_sock;
		private:
			struct sockaddr_can can_addr;
			struct can_filter mbox_filter;

	};
}
#endif
