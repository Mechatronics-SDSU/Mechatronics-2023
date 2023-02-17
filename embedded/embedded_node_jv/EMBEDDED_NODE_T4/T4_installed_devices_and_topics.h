#ifndef T4_INSTALLED_DEVICES_AND_TOPICS_H
#define T4_INSTALLED_DEVICES_AND_TOPICS_H

#include <FlexCAN_T4.h>
 
#define INSTALLED_DEVICE_CT 4

typedef void (**device_ptr_array_t)( CAN_message_t &msg);
typedef void (*topic_ptr_array_t)( CAN_message_t &msg);

////////////////////////////////////////////////////////////////////////////////////////
// General Use DREQ Functions
 void dreq_nop( CAN_message_t &msg);
 void dreq_res( CAN_message_t &msg);


////////////////////////////////////////////////////////////////////////////////////////
// 0x0000 EMBDSYS  Embedded System Info
#define EMBSYS_TOPIC_CT     18

// EMBDSYS Topics
  // 0x0000 NOP
  // 0x0001- 0x000F RES
 void embsys_subsysct( CAN_message_t &msg);
 void embsys_conndevct( CAN_message_t &msg);

topic_ptr_array_t dreq_EMBSYS[EMBSYS_TOPIC_CT] =
  {
    dreq_nop,         // 0x0000
    dreq_res,         // 0x0001
    dreq_res,         // 0x0002
    dreq_res,         // 0x0003
    dreq_res,         // 0x0004
    dreq_res,         // 0x0005
    dreq_res,         // 0x0006
    dreq_res,         // 0x0007
    dreq_res,         // 0x0008
    dreq_res,         // 0x0009
    dreq_res,         // 0x000A
    dreq_res,         // 0x000B
    dreq_res,         // 0x000C
    dreq_res,         // 0x000D
    dreq_res,         // 0x000E
    dreq_res,         // 0x000F
    embsys_subsysct,  // 0x0010
    embsys_conndevct  // 0x0011
  };

////////////////////////////////////////////////////////////////////////////////////////
// 0x0001 PWRSYS  Power System
#define PWRSYS_TOPIC_CT     30

// PWRSYS Topics
  // 0x0000 RES
 void pwrsys_vbatt( CAN_message_t &msg);           // 0x0001
 void pwrsys_vrail0( CAN_message_t &msg);          // 0x0002
 void pwrsys_vrail1( CAN_message_t &msg);          // 0x0003
 void pwrsys_vrail2( CAN_message_t &msg);          // 0x0004
 void pwrsys_vrail3( CAN_message_t &msg);          // 0x0005
 void pwrsys_vrail4( CAN_message_t &msg);          // 0x0006
 void pwrsys_vrail5( CAN_message_t &msg);          // 0x0007
 void pwrsys_vrail6( CAN_message_t &msg);          // 0x0008
 void pwrsys_vrail7( CAN_message_t &msg);          // 0x0009
  // 0x000A RES
 void pwrsys_ibatt( CAN_message_t &msg);           // 0x000B
 void pwrsys_irail0( CAN_message_t &msg);          // 0x000C
 void pwrsys_irail1( CAN_message_t &msg);          // 0x000D
 void pwrsys_irail2( CAN_message_t &msg);          // 0x000E
 void pwrsys_irail3( CAN_message_t &msg);          // 0x000F
 void pwrsys_irail4( CAN_message_t &msg);          // 0x0010
 void pwrsys_irail5( CAN_message_t &msg);          // 0x0011
 void pwrsys_irail6( CAN_message_t &msg);          // 0x0012
 void pwrsys_irail7( CAN_message_t &msg);          // 0x0013
  // 0x0014 - 0x0019 RES
 void pwrsys_tbatt( CAN_message_t &msg);           // 0x001A
 void pwrsys_trail0( CAN_message_t &msg);          // 0x001B
 void pwrsys_trail1( CAN_message_t &msg);          // 0x001C
 void pwrsys_trail2( CAN_message_t &msg);          // 0x001D

topic_ptr_array_t dreq_PWRSYS[PWRSYS_TOPIC_CT] =
  {
    dreq_res,
    pwrsys_vbatt,
    pwrsys_vrail0,
    pwrsys_vrail1,
    pwrsys_vrail2,
    pwrsys_vrail3,
    pwrsys_vrail4,
    pwrsys_vrail5,
    pwrsys_vrail6,
    pwrsys_vrail7,
    dreq_res,
    pwrsys_ibatt,
    pwrsys_irail0,
    pwrsys_irail1,
    pwrsys_irail2,
    pwrsys_irail3,
    pwrsys_irail4,
    pwrsys_irail5,
    pwrsys_irail6,
    pwrsys_irail7,    // 0x0013
    dreq_res,
    dreq_res,
    dreq_res,
    dreq_res,
    dreq_res,
    dreq_res,
    pwrsys_tbatt,
    pwrsys_trail0,
    pwrsys_trail1,
    pwrsys_trail2
  };


////////////////////////////////////////////////////////////////////////////////////////
// 0x0002 WAYFDVL  Wayfinder DVL
#define WAYFDVL_TOPIC_CT    19

  // 0x0000 NOP
 void wayfdvl_velocity_3( CAN_message_t &msg);    // 0x0001
 void wayfdvl_velocity_x( CAN_message_t &msg);    // 0x0002
 void wayfdvl_velocity_y( CAN_message_t &msg);    // 0x0003
 void wayfdvl_velocity_z( CAN_message_t &msg);    // 0x0004
 void wayfdvl_velocity_e( CAN_message_t &msg);    // 0x0005
  // 1x res                                       // 0x0006
 void wayfdvl_velocity_and_mean_depth( CAN_message_t &msg); // 0x0007
  // 2x res 0x0008 - 0x0009
 void wayfdvl_dist_mean( CAN_message_t &msg);     // 0x000A
 void wayfdvl_dist_1( CAN_message_t &msg);        // 0x000B
 void wayfdvl_dist_2( CAN_message_t &msg);        // 0x000C
 void wayfdvl_dist_3( CAN_message_t &msg);        // 0x000D
 void wayfdvl_dist_4( CAN_message_t &msg);        // 0x000E
  // 1x res                                       // 0x000F
 void wayfdvl_input_v( CAN_message_t &msg);       // 0x0010
 void wayfdvl_tx_v( CAN_message_t &msg);          // 0x0011
 void wayfdvl_tx_i( CAN_message_t &msg);          // 0x0012


topic_ptr_array_t dreq_WAYFDVL[WAYFDVL_TOPIC_CT] =
  {
    dreq_nop,
    wayfdvl_velocity_3,
    wayfdvl_velocity_x,
    wayfdvl_velocity_y,
    wayfdvl_velocity_z,
    wayfdvl_velocity_e,
    dreq_res,
    wayfdvl_velocity_and_mean_depth,
    dreq_res,
    dreq_res,
    wayfdvl_dist_mean,
    wayfdvl_dist_1,
    wayfdvl_dist_2,
    wayfdvl_dist_3,
    wayfdvl_dist_4,
    dreq_res,
    wayfdvl_input_v,
    wayfdvl_tx_v,
    wayfdvl_tx_i
  };

////////////////////////////////////////////////////////////////////////////////////////
// 0x0003 MS5837  MS5837 Pressure and Temp Sensor
#define MS5837_TOPIC_CT     4

  // 0x0000 NOP
 void ms5837_data( CAN_message_t &msg);
 void ms5837_depth( CAN_message_t &msg);
 void ms5837_temp( CAN_message_t &msg);

topic_ptr_array_t dreq_MS5837[MS5837_TOPIC_CT] =
  {
    dreq_nop,
    ms5837_data,
    ms5837_depth,
    ms5837_temp
  };



////////////////////////////////////////////////////////////////////////////////////////
// Device Array
device_ptr_array_t dreq_device[INSTALLED_DEVICE_CT] =
  {
    dreq_EMBSYS,
    dreq_PWRSYS,
    dreq_WAYFDVL,
    dreq_MS5837
    
  };



// Super important wrapper macro for easy nested array access
void dreq_access(uint16_t device, uint16_t topic,  CAN_message_t &msg);

void fill_msg_buffer_w_float(CAN_message_t &msg, float *d_in);
void fill_msg_buffer_w_float_buffer(CAN_message_t &msg,  uint8_t *buf);
void float_2_char_array(uint8_t *arr_out, float d_in);
#endif
