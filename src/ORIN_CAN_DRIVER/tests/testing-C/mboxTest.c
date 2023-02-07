#include "../mbox_can/mbox_can.h"

#include <string.h>
#include <net/if.h>
#include <stdio.h>
#include <time.h>

#define NUM_BOXES 4
#define INTERFACE "vcan0"

struct mbox_can mailboxes[NUM_BOXES];
struct can_frame in_frame;
char* mb_names[] = {"emergency", "motor", "sensor", "comms"};

struct ifreq ifr;
struct timespec rem, req = {0,1*(1000*1000)};
int main()
{
	strncpy(ifr.ifr_name, INTERFACE, sizeof(&INTERFACE));

	//initialize mailboxes
	for(int i = 0; i < NUM_BOXES; i++)
	{
		init_mbox(&mailboxes[i], &ifr, mb_names[i]);	
	}
	set_f(&mailboxes[0], (struct can_filter){0x000, 0xff0});
	set_f(&mailboxes[1], (struct can_filter){0x010, 0xff0});
	set_f(&mailboxes[2], (struct can_filter){0x020, 0xff0});
	set_f(&mailboxes[3], (struct can_filter){0x030, 0xff0});

	while(1)
	{

		for(int m = 0; m < NUM_BOXES; m++)
		{
			if(read_mbox(&mailboxes[m], &in_frame) > 0)
			{
				printf("[ MAILBOX \"%s\" ]New frame: %03x#", mailboxes[m].mbox_name, in_frame.can_id);
				for(int i = 0; i < in_frame.can_dlc; i++)
				{
					printf("%02X",in_frame.data[i]);
				}
				printf("\n");
			}
		}
		nanosleep(&req, &rem);
	}

	for(int i = 0; i < NUM_BOXES; i++)
	{
		close_mbox(&mailboxes[i]);
	}
}
