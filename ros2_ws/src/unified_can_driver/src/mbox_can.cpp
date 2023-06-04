 /* Connor Larmer
 * 1.24.2023
 * mbox_can class implementation
 * See [include/mbox_can.hpp] for a more in-depth documentation
 * of each method's use and purpose.
 */

#include "mbox_can.hpp"
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
using namespace Mailbox;


/*	Initializes a mailbox by creating & binding a socket to the specified
 *	interface. By default, mailboxes are non-blocking. A name is stored with
 *  the mailbox for identification.
 *	Ex:	
 * 		Mailbox::MboxCan* mailbox;
 * 		mailbox = new Mailbox::MboxCan::MboxCan(&ifr, "motors");
 */
MboxCan::MboxCan(struct ifreq* ifr, const std::string name)
{
	mbox_name = new std::string(name);
	can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	int flags = fcntl(can_sock, F_GETFL);
	fcntl(can_sock, F_SETFL, flags | O_NONBLOCK);
	ioctl(can_sock, SIOCGIFINDEX, ifr);
	can_addr.can_family = AF_CAN;
	can_addr.can_ifindex = ifr->ifr_ifindex;
	bind(can_sock, (struct sockaddr*)&can_addr, sizeof(&can_addr));
}

/* applies a filter on the Mailbox socket to limit acceptable IDs. Filtering
 * is done ate the kernel level through socketCAN.
 * Ex:
 *		Mailbox::MboxCan::set_filter(&mailbox_obj, {0x000,0xff0});
 */
void MboxCan::set_filter(MboxCan* mbox, struct can_filter filter)
{
	mbox->mbox_filter = filter;
	setsockopt(mbox->can_sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(&filter));
}

/* Reads a mailbox and outputs any data into a can_frame struct. If there
 * is an issue with the read OR there is no data to be recieved, the function
 * returns -1. 0 is returned if successful.
 * 	Ex:
 *		struct can_frame in_frame;								//New frame to be read into
 *		Mailbox::MboxCan::read_mbox(&mailbox_obj, &in_frame);	//Copy available data to frame
 */
int MboxCan::read_mbox(MboxCan* mbox, struct can_frame* frame)
{
	int nbytes = read(mbox->can_sock, frame, sizeof(struct can_frame));
	if(nbytes > 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

/* Writes a CAN frame to a mailbox. In the event that the write fails,
 * the function will return -1. Otherwise, 0 is returned if successful.
 * 	Ex:
 *		struct can_frame out_frame;								//New frame to be read into
 *		memset(&out_frame, 0, sizeof(struct can_frame));		//initializes out_frame.
 *		out_frame.can_id  = 0x000;
 *		out_frame.can_dlc = 0;
 *		Mailbox::MboxCan::read_mbox(&mailbox_obj, &out_frame);	//Copy available data to frame
 */
int MboxCan::write_mbox(MboxCan* mbox, struct can_frame* frame)
{
	int nbytes = write(mbox->can_sock, frame, sizeof(struct can_frame));
	if(nbytes > 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

/* Closes the mailbox socket to free recourses. If unsuccessful,
 * it returns -1, otherwise it returns 0.
 * Ex:
 *		Mailbox::MboxCan::close_mbox(&mailbox_obj);
 */
int MboxCan::close_mbox(MboxCan* mbox)
{
	if(!close(mbox->can_sock))
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
