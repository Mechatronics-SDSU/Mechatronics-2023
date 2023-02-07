/*
 * 12.1.22 Connor Larmer
 * Non-blocking, multi-socket CAN driver prototype for Junebug
 * Categories and their address space have been hardcoded based on
 * the current protocol sheet, sockets in the array are assigned filters
 * based on the current order of said sheet -->
 * 		[0] Emergency Messaging
 * 		[1] Status/Control
 * 		[2] Motor Controls
 * 		[3] Sensors
 * 		[X] Comms (Not integrated)
 * This is primarily for testing, and has no integration with ROS2 (YET)
 * much inspiration from https://github.com/craigpeacock/CAN-Examples
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>

int init_can(int*, struct ifreq*, struct sockaddr_can*);

int main()
{
	int socket_count = 4;
	int can_sockets[socket_count];
	struct sockaddr_can can_addrs[socket_count];
	struct can_frame fin_frames[socket_count];
	struct ifreq ifr; 				//CAN interface (ex: "can0", "vcan0", etc.)
	// struct can_frame inFrame; 		// CAN Frame to be read (Teensy --> Orin)
	// struct can_frame outFrame;		// CAN Frame to be sent (Orin --> Teensy)

	//Declaring and setting address space for filters
	struct can_filter 	emer_filter[2],
						stat_filter[1],
						moto_filter[2],
						sens_filter[2],
						comm_filter[1];
	//set emergency ID space
	emer_filter[0] = (struct can_filter){0x000, 0xff8};
	emer_filter[1] = (struct can_filter){0x008, 0xffc};
	//set status ID space
	stat_filter[0] = (struct can_filter){0x00c, 0xffc};
	//set motor ID space
	moto_filter[0] = (struct can_filter){0x010, 0xff8};
	moto_filter[1] = (struct can_filter){0x018, 0xff8};
	//set sensor ID space
	sens_filter[0] = (struct can_filter){0x020, 0xff8};
	sens_filter[1] = (struct can_filter){0x028, 0xffc};	// Current protocol sheet calls for 11 ID spaces, but 12 is easier 

	struct timespec remaining, request = {0, 50000000};

	//set up interface
	strcpy(ifr.ifr_name, "vcan0");

	//initialize multiple sockets (each with own filters)
	memset(can_sockets, 0, socket_count*sizeof(int));
	for(int i = 0; i < socket_count; i++)
	{
		init_can(&can_sockets[i], &ifr, &can_addrs[i]);		//initializes non-blocking sockets
		memset(&fin_frames[i], 0, sizeof(fin_frames[i]));
	}
	setsockopt(can_sockets[0],SOL_CAN_RAW, CAN_RAW_FILTER, &emer_filter, sizeof(emer_filter));	
	setsockopt(can_sockets[1],SOL_CAN_RAW, CAN_RAW_FILTER, &stat_filter, sizeof(stat_filter));	
	setsockopt(can_sockets[2],SOL_CAN_RAW, CAN_RAW_FILTER, &moto_filter, sizeof(moto_filter));	
	setsockopt(can_sockets[3],SOL_CAN_RAW, CAN_RAW_FILTER, &sens_filter, sizeof(sens_filter));	


	while(1)
	{
		//poll filtered sockets
		for(int i = 0; i < socket_count; i++)
		{
			int nbytes = read(can_sockets[i], &fin_frames[i], sizeof(struct can_frame));
			if(nbytes > 0)
			{
				printf("Catagory: %i || ID: %03x || DATA: ", i, fin_frames[i].can_id);
				for(int f = 0; f < fin_frames[i].can_dlc; f++)
				{
					printf("%02X ",fin_frames[i].data[f]);
				}
				printf("\n");
			}
		}
		nanosleep(&request, &remaining);
	}
	
	//closes CAN sockets
	for(int i = 0; i < socket_count; i++)
	{
		if(close(can_sockets[i]) < 0)
		{
			perror("Failed to close socket");
			return 1;
		}	
	}
}

int init_can(int* sock, struct ifreq* ifr, struct sockaddr_can* addr)
{
	memset(addr, 0, sizeof(&addr));
	if((*sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		return -1;
	}
	int flags = fcntl(*sock, F_GETFL);
	fcntl(*sock, F_SETFL, flags | O_NONBLOCK);
	ioctl(*sock, SIOCGIFINDEX, ifr);
	addr->can_family = AF_CAN;
	addr->can_ifindex = ifr->ifr_ifindex;
	if(bind(*sock, (struct sockaddr*)addr, sizeof(&addr) ) < 0 )
	{
		return -1;
	}
	return 0;
}
