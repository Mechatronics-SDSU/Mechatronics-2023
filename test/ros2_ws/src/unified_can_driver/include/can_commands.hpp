/* Connor Larmer
 * 03/09/2023
 * This header file contains various Enum definitions that
 * can be used to simplify communication with the CAN Driver
 * and other CAN Related tasks.
 *
 * In order to use this header, it must be included within your project.
 * Then, simply access a command/device as follows:
 * 
 * 		CanDriver::Command::DREQ 						(Returns the address for DREQ)
 *		CanDriver::Device::WAYFDVL::TOPIC_VELOCITY_3	(Returns the topic ID for DVL_VELOCITY_3)
 * 		CanDriver::Device::MS5837::ID 					(Returns Device ID for MS5837)
 *
 * As of the date at the top of this file, this list is up-to-date
 * with the protocol sheet.
 * (https://docs.google.com/spreadsheets/d/1Z7OvEd_o8Brstv6O6mNKoWZZ86YrVBjY30yL9ZncIfY/edit?usp=sharing).
 */

#ifndef CAN_MACROS_H
#define CAN_MACROS_H

namespace CanDriver
{
/**************************************************/

	enum class Command
	{
		SOFTKILL = 0x000,
		CLEARERR = 0x00A,
		DREQ = 0x020,
		DRES = 0x021,
		STOW = 0x022
	};
/**************************************************/
	namespace Device
	{
		enum class EMBDSYS
		{
			ID					= 0x0000,
			TOPIC_STATECTL		= 0x0000,
			TOPIC_SUBSYSCT		= 0x0010,
			TOPIC_CONNDEVCT		= 0x0011
		};
/**************************************************/

		enum class PWRSYS
		{
			ID					= 0x0001,
			TOPIC_VBATT			= 0x0001,
			TOPIC_VRAIL_0		= 0x0002,
			TOPIC_VRAIL_1		= 0x0003,
			TOPIC_VRAIL_2		= 0x0004,
			TOPIC_VRAIL_3		= 0x0005,
			TOPIC_VRAIL_4		= 0x0006,
			TOPIC_VRAIL_5		= 0x0007,
			TOPIC_VRAIL_6		= 0x0008,
			TOPIC_VRAIL_7		= 0x0009,
			TOPIC_IBATT			= 0x000B,
			TOPIC_IRAIL_0		= 0x000C,
			TOPIC_IRAIL_1		= 0x000D,
			TOPIC_IRAIL_2		= 0x000E,
			TOPIC_IRAIL_3		= 0x000F,
			TOPIC_IRAIL_4		= 0x0010,
			TOPIC_IRAIL_5		= 0x0011,
			TOPIC_IRAIL_6		= 0x0012,
			TOPIC_IRAIL_7		= 0x0013,
			TOPIC_TBATT			= 0x001A,
			TOPIC_TRAIL_0		= 0x001B,
			TOPIC_TRAIL_1		= 0x001C,
			TOPIC_TRAIL_2		= 0x001D
		};
/**************************************************/

		enum class WAYFDVL
		{
			ID						= 0x0002,
			TOPIC_INFO				= 0x0000,
			TOPIC_VELOCITY_3		= 0x0001,
			TOPIC_VELOCITY_X		= 0x0002,
			TOPIC_VELOCITY_Y		= 0x0003,
			TOPIC_VELOCITY_Z		= 0x0004,
			TOPIC_VELOCITY_E		= 0x0005,
			TOPIC_VEL_AND_DIST		= 0x0007,
			TOPIC_DIST_2_BOTT	= 0x000A,
			TOPIC_DIST_1		= 0x000B,
			TOPIC_DIST_2 		= 0x000C,
			TOPIC_DIST_3 		= 0x000D,
			TOPIC_DIST_4 		= 0x000E,
			TOPIC_INPUT_V 		= 0x0010,
			TOPIC_TX_VOLTAGE 	= 0x0011,
			TOPIC_TX_CURRENT 	= 0x0012,
			TOPIC_SET_SETUP 	= 0x0016,
			TOPIC_GET_SETUP 	= -1,
			TOPIC_SET_SYSTEM 	= -1,
			TOPIC_GET_SYSTEM 	= -1,
			TOPIC_SET_AUTO 		= -1
		};
/**************************************************/

		enum class MS5837
		{
			ID					= 0x0003,
			TOPIC_INFO			= 0x0000,
			TOPIC_DATA			= 0x0001,	
			TOPIC_DEPTH			= 0x0002,
			TOPIC_TEMP			= 0x0003,
		};
/**************************************************/

		enum class BRLIGHT

		{
			ID						= 0x0004,
			TOPIC_INFO				= 0x0000,
			TOPIC_FRLIGHT_BRIGHT	= 0x0004,
		};
/**************************************************/

		enum class BRPING1
		{
			ID				= 0x0012,
			INFO			= 0x0000,
			P1D_DIST_SHORT	= 0x0004,
		};
	}
}

#endif
