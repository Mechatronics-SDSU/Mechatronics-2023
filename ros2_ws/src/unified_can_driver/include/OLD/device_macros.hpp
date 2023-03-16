/* 2.7.23 
 * Connor Larmer
 * Macro definitions for various embedded systems.
 * Last updated to protocol sheet on date above.
 *
 * The name scheme follows the basic format below:
 * 	->	Device ID: [DEVICE NAME]_DEV_ID
 * 	->	Topic ID : [DEVICE_NAME]_TOPIC_[MNEMONIC] 
 *
 */


/* DRES MESSAGE ID */
#define DRES_MSG_ID		0x020
#define DREQ_MSG_ID		0x021
/* EMBEDDED SUBSYSTEM ***************************/
#define EMBEDSYS_DEV_ID 				0x0000

#define EMBEDSYS_TOPIC_NOP 				0x0000
#define EMBEDSYS_TOPIC_SUBSYSCT 		0x0010
#define EMBEDSYS_TOPIC_CONNDEVCT 		0x0011

/* POWER SYSTEM *********************************/
#define PWRSYS_DEV_ID 					0x0001

#define PWRSYS_TOPIC_VBATT 				0x0001
#define PWRSYS_TOPIC_VRAIL_0			0x0002
#define PWRSYS_TOPIC_VRAIL_1 			0x0003
#define PWRSYS_TOPIC_VRAIL_2			0x0004
#define PWRSYS_TOPIC_VRAIL_3 			0x0005
#define PWRSYS_TOPIC_VRAIL_4 			0x0006
#define PWRSYS_TOPIC_VRAIL_5 			0x0007
#define PWRSYS_TOPIC_VRAIL_6 			0x0008
#define PWRSYS_TOPIC_VRAIL_7 			0x0009

#define PWRSYS_TOPIC_IBATT 				0x000B
#define PWRSYS_TOPIC_IRAIL_0			0x000C
#define PWRSYS_TOPIC_IRAIL_1 			0x000D
#define PWRSYS_TOPIC_IRAIL_2			0x000E
#define PWRSYS_TOPIC_IRAIL_3 			0x000F
#define PWRSYS_TOPIC_IRAIL_4 			0x0010
#define PWRSYS_TOPIC_IRAIL_5 			0x0011
#define PWRSYS_TOPIC_IRAIL_6 			0x0012
#define PWRSYS_TOPIC_IRAIL_7 			0x0013

#define PWRSYS_TOPIC_TBATT 				0x001A
#define PWRSYS_TOPIC_TRAIL_0			0x001B
#define PWRSYS_TOPIC_TRAIL_1 			0x001C
#define PWRSYS_TOPIC_TRAIL_2 			0x001D

/* DVL SENSOR DATA ******************************/
#define WAYFDVL_DEV_ID 					0x0002

#define WAYFDVL_TOPIC_NOP 				0x0000

#define WAYFDVL_TOPIC_VELOCITY_3 		0x0001
#define WAYFDVL_TOPIC_VELOCITY_X 		0x0002
#define WAYFDVL_TOPIC_VELOCITY_Y 		0x0003
#define WAYFDVL_TOPIC_VELOCITY_Z 		0x0004
#define WAYFDVL_TOPIC_VELOCITY_E 		0x0005

#define WAYFDVL_TOPIC_DIST_TO_BOTT 		0x000A
#define WAYFDVL_TOPIC_DIST_1 			0x000B
#define WAYFDVL_TOPIC_DIST_2 			0x000C
#define WAYFDVL_TOPIC_DIST_3 			0x000D
#define WAYFDVL_TOPIC_DIST_4 			0x000E

#define WAYFDVL_TOPIC_INPUT_V 			0x0010		/* UNUSED */
#define WAYFDVL_TOPIC_TX_VOLT 			0x0011		/* UNUSED */
#define WAYFDVL_TOPIC_TX_CURR 			0x0012		/* UNUSED */

#define WAYFDVL_TOPIC_SET_SETUP 		0x0016		/* UNUSED */
#define WAYFDVL_TOPIC_GET_SETUP 		0x0000		/* UNUSED */
#define WAYFDVL_TOPIC_SET_SYS 			0x0000		/* UNUSED */
#define WAYFDVL_TOPIC_GET_SYS 			0x0000		/* UNUSED */
#define WAYFDVL_TOPIC_SET_AUTO 			0x0000		/* UNUSED */

/* MS5837 Sensor Data ***************************/
#define MS5837_DEV_ID 					0x0003

#define MS5837_TOPIC_NOP				0x0000

#define MS5837_TOPIC_DATA 				0x0001
#define MS5837_TOPIC_DEPTH 				0x0002
#define MS5837_TOPIC_TEMP 				0x0003
