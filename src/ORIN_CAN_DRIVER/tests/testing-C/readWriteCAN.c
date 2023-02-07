/*
 * 11.29.22 Connor Larmer
 * socket-can read/write/filtering for Junebug
 * much inspiration from https://github.com/craigpeacock/CAN-Examples
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>

int setupCAN(char*, int*, struct ifreq*, struct sockaddr_can*);

int main()
{
	
	int canSock; 					// CAN Socket FD
	struct sockaddr_can canAddr; 	// Socket Address
	struct ifreq ifr; 				//CAN interface (ex: "can0", "vcan0", etc.)
	struct can_frame inFrame; 		// CAN Frame to be read (Teensy --> Orin)
	struct can_frame outFrame;		// CAN Frame to be sent (Orin --> Teensy)

	//move the uggy code into a function
	setupCAN("vcan0", &canSock, &ifr, &canAddr);

	//Reads & prints CAN frame
	read(canSock, &inFrame, sizeof(struct can_frame));
	printf("ID: %04X || LENGTH: %i || DATA: ",inFrame.can_id,inFrame.can_dlc);
	for(int i = 0; i < inFrame.can_dlc; i++)
	{
		printf("%X",inFrame.data[i]);
	}
	printf("\n");

	//write to CAN socket
	memset(&outFrame, 0, sizeof(outFrame));
	outFrame.can_id = 0x010;
	outFrame.can_dlc = 4;
	memcpy(outFrame.data,(char[]){0xDE,0xAD,0xBE,0xBE},outFrame.can_dlc);
	write(canSock, &outFrame, sizeof(struct can_frame));
	
	//closes CAN socket
	if(close(canSock) < 0)
	{
		perror("Failed to close socket");
		return 1;
	}
}

int setupCAN(char* devname, int* sock, struct ifreq* ifr, struct sockaddr_can* addr)
{
	strncpy(ifr->ifr_name,devname,6);
	memset(addr, 0 ,sizeof(&addr));
	if((*sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		return -1;
	}
	ioctl(*sock, SIOCGIFINDEX, ifr);
	addr->can_family = AF_CAN;
	addr->can_ifindex = ifr->ifr_ifindex;
	if(bind(*sock, (struct sockaddr*)addr, sizeof(&addr) ) < 0 )
	{
		return -1;
	}
	return 0;
}
