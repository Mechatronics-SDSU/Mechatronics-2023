#include "T4_installed_devices_and_topics.h"

////////////////////////////////////////////////////////////////////////////////////////////
//                      READING ADDRESS SPACE
////////////////////////////////////////////////////////////////////////////////////////////

void dreq_nop( CAN_message_t &msg){
#ifdef DEBUG_DECODE
  Serial.printf("DREQ_NOP\n");
#endif
}
void dreq_res( CAN_message_t &msg){
  Serial.printf("DREQ_RES\n");
}

////////////////////////////////////////////////////////////////////////////////////////
// 0x0000 EMBDSYS  Embedded System Info

// EMBDSYS Topics
  // 0x0000 NOP
  // 0x0001- 0x000F RES
 void embsys_subsysct( CAN_message_t &msg){
  
}

 void embsys_conndevct( CAN_message_t &msg){
  
}

////////////////////////////////////////////////////////////////////////////////////////
// 0x0001 PWRSYS  Power System

// PWRSYS Topics
  // 0x0000 RES

 void pwrsys_vbatt( CAN_message_t &msg){           // 0x0001
           
}

 void pwrsys_vrail0( CAN_message_t &msg){          // 0x0002
  
}

 void pwrsys_vrail1( CAN_message_t &msg){          // 0x0003
  
}

 void pwrsys_vrail2( CAN_message_t &msg){          // 0x0004
  
}

 void pwrsys_vrail3( CAN_message_t &msg){          // 0x0005
  
}

 void pwrsys_vrail4( CAN_message_t &msg){          // 0x0006
  
}

 void pwrsys_vrail5( CAN_message_t &msg){          // 0x0007
  
}

 void pwrsys_vrail6( CAN_message_t &msg){          // 0x0008
  
}

 void pwrsys_vrail7( CAN_message_t &msg){          // 0x0009
  
}
  // 0x000A RES
 void pwrsys_ibatt( CAN_message_t &msg){           // 0x000B
  
}

 void pwrsys_irail0( CAN_message_t &msg){          // 0x000C
  
}

 void pwrsys_irail1( CAN_message_t &msg){          // 0x000D
  
}

 void pwrsys_irail2( CAN_message_t &msg){          // 0x000E
  
}

 void pwrsys_irail3( CAN_message_t &msg){          // 0x000F
  
}

 void pwrsys_irail4( CAN_message_t &msg){          // 0x0010
  
}

 void pwrsys_irail5( CAN_message_t &msg){          // 0x0011
  
}

 void pwrsys_irail6( CAN_message_t &msg){          // 0x0012
  
}

 void pwrsys_irail7( CAN_message_t &msg){          // 0x0013
  
}
  // 0x0014 - 0x0019 RES
 void pwrsys_tbatt( CAN_message_t &msg){           // 0x001A
  
}

 void pwrsys_trail0( CAN_message_t &msg){          // 0x001B
  
}

 void pwrsys_trail1( CAN_message_t &msg){          // 0x001C
  
}

 void pwrsys_trail2( CAN_message_t &msg){          // 0x001D
  
}


////////////////////////////////////////////////////////////////////////////////////////
// 0x0002 WAYFDVL  Wayfinder DVL

#define WAYFDVL_DEVICE_ID  0x0003

  // Macro support still sucks
void wayfdvl_info( CAN_message_t &msg){                                         // 0x0000
  if(msg.id == STOW_ID){
    if(msg.buf[4]) {
      #ifdef ENABLE_DVL
      init_DVL_serial();
      WAYFDVL = init_DVL_data_struct();
      #ifdef DEBUG_MODE
      Serial.printf("Serial 1\nTX Capacity:\t%d bytes\nRX Capacity:\t%d bytes\n", DVLSERIAL.availableForWrite(), DVLSERIAL.available());
      Serial.printf("DVL Struct: %08X\n", WAYFDVL);
      #endif
      #endif
    }
  } else {
    // Return Device info
  }
}

void wayfdvl_velocity_3( CAN_message_t &msg){                                   // 0x0001
  msg.buf[2] = 0x02;
  dreq_access(WAYFDVL_DEVICE_ID, 0x0002, msg);    // X
  Can0.write(msg);
  
  msg.buf[2] = 0x03;
  dreq_access(WAYFDVL_DEVICE_ID, 0x0003, msg);    // Y
  Can0.write(msg);
  
  msg.buf[2] = 0x04;
  dreq_access(WAYFDVL_DEVICE_ID, 0x0004, msg);    // Z, implicit send
}

void wayfdvl_velocity_x( CAN_message_t &msg){                                   // 0x0002
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.velocity.X);  
}

void wayfdvl_velocity_y( CAN_message_t &msg){                                   // 0x0003
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.velocity.Y);
}

void wayfdvl_velocity_z( CAN_message_t &msg){                                   // 0x0004
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.velocity.Z);
}

void wayfdvl_velocity_e( CAN_message_t &msg){                                   // 0x0005
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.velocity.error);
}
  // 1x res
void wayfdvl_velocity_and_mean_depth( CAN_message_t &msg){                      // 0x0007
  wayfdvl_velocity_3(msg);      // X, Y, Z velocity
  Can0.write(msg);              // Make up for missing implicit send
  msg.buf[2] = 0x0A;
  wayfdvl_dist_mean(msg);       // Mean dist, implicit send
}

  // 2x res
  
void wayfdvl_dist_mean( CAN_message_t &msg){                                    // 0x000A
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.range.mean);
}

void wayfdvl_dist_1( CAN_message_t &msg){
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.range.beam0);
}

void wayfdvl_dist_2( CAN_message_t &msg){
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.range.beam1);
}

void wayfdvl_dist_3( CAN_message_t &msg){
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.range.beam2);
}

void wayfdvl_dist_4( CAN_message_t &msg){
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.range.beam3);
}
  // 1x res
void wayfdvl_input_v( CAN_message_t &msg){          // 0x0010
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.power.input_voltage);
}

void wayfdvl_tx_v( CAN_message_t &msg){             // 0x0011
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.power.tx_voltage);
}

void wayfdvl_tx_i( CAN_message_t &msg){             // 0x0012
  fill_msg_buffer_w_float(msg, &WAYFDVL->responseData.binaryData.power.tx_current);
}

////////////////////////////////////////////////////////////////////////////////////////
// 0x0003 MS5837  MS5837 Pressure and Temp Sensor

#define MS5837_DEVICE_ID  0x0003

void ms5837_info( CAN_message_t &msg){                 // 0x0000
  if(msg.id == STOW_ID){
    if(msg.buf[4]){
      #ifdef ENABLE_PRES_SENS
      // Start i2C 0 at 400kHz, initiate pressure sensor
      startup_pressure_sensor( &pressure_sensor);
      #endif
    }
  }
}

void ms5837_data( CAN_message_t &msg){                // 0x00 01
#ifdef DEBUG_DREQ_PTR
  Serial.printf("Data Requested!\n");
#endif
  // Macro support def needs to be better than this idk
  msg.buf[2] = 0x02;
  dreq_access(MS5837_DEVICE_ID, 0x0002, msg);
  Can0.write(msg);
  msg.buf[2] = 0x03;
  dreq_access(MS5837_DEVICE_ID, 0x0003, msg);
}

void ms5837_depth(CAN_message_t &msg){                // 0x00 02
#ifdef DEBUG_DREQ_PTR
  Serial.println("Depth Requested!\n");
#endif
  //fill_msg_buffer_w_float_buffer(msg, pressure_sensor.depth);
  float t__ = ms5837_Read_Depth();
  fill_msg_buffer_w_float(msg, &t__);
}

void ms5837_temp(CAN_message_t &msg){                 // 0x00 03
#ifdef DEBUG_DREQ_PTR
  Serial.printf("Temp Requested!\n");
#endif
  //fill_msg_buffer_w_float_buffer(msg, pressure_sensor.temperature);
  float t__ = ms5837_Read_Temp();
  fill_msg_buffer_w_float(msg, &t__);
}

#undef MS5837_DEVICE_ID

////////////////////////////////////////////////////////////////////////////////////////
// 0x0004 BRLIGHT External LED Lights for illumination
void brlight_info( CAN_message_t &msg){
#ifdef DEBUG_DREQ_PTR
  Serial.printf("BRLIGHT Info Access ID: %02X\n", msg.id);
#endif
  if(msg.id == STOW_ID){
    startup_light_system( &lights );  
    set_num_enabled_lights( &lights , MAX_BRLIGHTS);
    
  } else {
    msg.id = DRES_ID;
    Can0.write(msg);
  }
}
 // 3x res
void brlight_front_brightness( CAN_message_t &msg){
  //set_light_levels(bright_lights_t *light_struct, uint8_t *vals)
#ifdef DEBUG_DREQ_PTR  
  Serial.printf("Set Light Levels: %3u percent\n", msg.buf[4]);
#endif
  if(msg.len > 4){
    set_light_levels( &lights , msg.buf[4]);
  } else {
    set_light_levels( &lights , 0x00);
  }
  
}

////////////////////////////////////////////////////////////////////////////////////////

// Addressing function to find correct jump table address
/*  
 * IMPORTANT APP NOTE!!! - Joseph De Vico, 01/30/2023 09:15
 * Syntax Breakdown of: (*(*(dreq_device[device])[topic]))(msg);
 * 
 * All topics (datapoints) are "returned" (CAN message filled w data)
 *  by unique functions. These functions are pointed to by a device
 *  specific array (ie. dreq_MS5837 contains function pointers for
 *  datapoint return functions pertaining to the MS5837).
 *  
 *  These device specific arrays of pointers themselves are pointed to
 *  from an array containing every installed device.
 *  (ie. dreq_device contains pointers to dreq_{SPECIFIC DEVICE})
 *  
 *  To jump to any function we rely on first addressing (finding)
 *  the array containing the correct datapoints (the device specific
 *  array of functions), then from that base address finding the
 *  correct function.
 *  
 *  First we will locate the device specific array base address with:
 *  
 *      dreq_device[device]
 *      
 *  Now that we have the addresss of the device array we will
 *  dereference this value such that we are now counting from the
 *  device specific array's base address:
 *    
 *      *(dreq_device[device])
 *      
 *  Say we selected device 0x0003, or the MS5837 depth sensor in the
 *  2023 iteration of the platform. The above line would be equivalent
 *  to the label:
 *   
 *      dreq_MS5837
 *      
 *  From this base address we once again offset to find the specific
 *  function of interest. Say we selected 0x0002, or last read Depth.
 *  This would be indicated by:
 *  
 *      dreq_MS5837[2]
 *      
 *  Or as we now know:
 *  
 *      *(dreq_device[0x0003])[0x0002]
 *      
 *  This now is the address of the function of interest.
 *  From here standard array of functions notation takes over.
 *  We will dereference this address to execute the function,
 *  passing an argument of set type (CAN_message_t &):
 *  
 *      (*(*(dreq_device[device])[topic]))(msg);
 *      
 * 
 *  This iteration should be easily read now.
 *  
 *  Quick Note: g++ does not handle pointer arithmetic the same as
 *    gcc as C++ and C have differing implicit pointer casts.
 *    This notation is proper, effective, and easy to read.
 *    Please do not modify.
 * 
 * 
 * 
 *  dreq_access will locate the correct function and fill the msg data field appropriately
 *  dreq_access will not send the message
 */
void dreq_access(uint16_t device, uint16_t topic,  CAN_message_t &msg){
#ifdef DEBUG_DREQ_PTR
  Serial.printf("DREQ Device: %4X\tTopic: %4X\n", device, topic);
  void * jump_address = (*(dreq_device[device])[topic]);
  Serial.printf("Jump Address: %8X\n", jump_address);
  Serial.printf("DREQ_D @ %8X ->  %8X   %8X   %8X   %8X\n", dreq_device, dreq_device[0], dreq_device[1], dreq_device[2], dreq_device[3]); 
#endif
//  Serial.printf("MS5837 @ %8X ->  %8X   %8X   %8X   %8X\n", dreq_MS5837, dreq_MS5837[0], dreq_MS5837[1], dreq_MS5837[2], dreq_MS5837[3]);

  // Valid in C++, pure pointer math was nuked by g++ with optimizations turned on for some reason :(
  (*(*(dreq_device[device])[topic]))(msg);
}

void fill_msg_buffer_w_float(CAN_message_t &msg, float *d_in){
  msg.buf[4] = *((uint8_t *)(d_in));
  msg.buf[5] = *((uint8_t *)(d_in) + 1u);
  msg.buf[6] = *((uint8_t *)(d_in) + 2u);
  msg.buf[7] = *((uint8_t *)(d_in) + 3u);
}

void fill_msg_buffer_w_float_buffer(CAN_message_t &msg, uint8_t *buf){
  msg.buf[4] = buf[0];
  msg.buf[5] = buf[1];
  msg.buf[6] = buf[2];
  msg.buf[7] = buf[3];
}

void float_2_char_array(uint8_t *arr_out, float d_in){
  arr_out[0] = *((uint8_t *)(&d_in));
  arr_out[1] = *((uint8_t *)(&d_in) + 1u);
  arr_out[2] = *((uint8_t *)(&d_in) + 2u);
  arr_out[3] = *((uint8_t *)(&d_in) + 3u); 
}


////////////////////////////////////////////////////////////////////////////////////////////
//                      WRITING ADDRESS SPACE
////////////////////////////////////////////////////////////////////////////////////////////
