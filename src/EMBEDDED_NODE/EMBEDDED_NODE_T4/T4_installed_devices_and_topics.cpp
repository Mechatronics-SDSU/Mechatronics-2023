#include "T4_installed_devices_and_topics.h"

////////////////////////////////////////////////////////////////////////////////////////
// 0x0000 EMBDSYS  Embedded System Info

// EMBDSYS Topics
  // 0x0000 NOP
  // 0x0001- 0x000F RES
 void embsys_subsysct(const CAN_message_t &msg){
  
}

 void embsys_conndevct(const CAN_message_t &msg){
  
}

////////////////////////////////////////////////////////////////////////////////////////
// 0x0001 PWRSYS  Power System

// PWRSYS Topics
  // 0x0000 RES

 void pwrsys_vbatt(const CAN_message_t &msg){           // 0x0001
           
}

 void pwrsys_vrail0(const CAN_message_t &msg){          // 0x0002
  
}

 void pwrsys_vrail1(const CAN_message_t &msg){          // 0x0003
  
}

 void pwrsys_vrail2(const CAN_message_t &msg){          // 0x0004
  
}

 void pwrsys_vrail3(const CAN_message_t &msg){          // 0x0005
  
}

 void pwrsys_vrail4(const CAN_message_t &msg){          // 0x0006
  
}

 void pwrsys_vrail5(const CAN_message_t &msg){          // 0x0007
  
}

 void pwrsys_vrail6(const CAN_message_t &msg){          // 0x0008
  
}

 void pwrsys_vrail7(const CAN_message_t &msg){          // 0x0009
  
}
  // 0x000A RES
 void pwrsys_ibatt(const CAN_message_t &msg){           // 0x000B
  
}

 void pwrsys_irail0(const CAN_message_t &msg){          // 0x000C
  
}

 void pwrsys_irail1(const CAN_message_t &msg){          // 0x000D
  
}

 void pwrsys_irail2(const CAN_message_t &msg){          // 0x000E
  
}

 void pwrsys_irail3(const CAN_message_t &msg){          // 0x000F
  
}

 void pwrsys_irail4(const CAN_message_t &msg){          // 0x0010
  
}

 void pwrsys_irail5(const CAN_message_t &msg){          // 0x0011
  
}

 void pwrsys_irail6(const CAN_message_t &msg){          // 0x0012
  
}

 void pwrsys_irail7(const CAN_message_t &msg){          // 0x0013
  
}
  // 0x0014 - 0x0019 RES
 void pwrsys_tbatt(const CAN_message_t &msg){           // 0x001A
  
}

 void pwrsys_trail0(const CAN_message_t &msg){          // 0x001B
  
}

 void pwrsys_trail1(const CAN_message_t &msg){          // 0x001C
  
}

 void pwrsys_trail2(const CAN_message_t &msg){          // 0x001D
  
}


////////////////////////////////////////////////////////////////////////////////////////
// 0x0002 WAYFDVL  Wayfinder DVL

  // 0x0000 NOP
 void wayfdvl_velocity_3(const CAN_message_t &msg){
  
}

 void wayfdvl_velocity_x(const CAN_message_t &msg){
  
}

 void wayfdvl_velocity_y(const CAN_message_t &msg){
  
}

 void wayfdvl_velocity_z(const CAN_message_t &msg){
  
}

 void wayfdvl_velocity_e(const CAN_message_t &msg){
  
}
  // 4x res
 void wayfdvl_dist_mean(const CAN_message_t &msg){
  
}

 void wayfdvl_dist_1(const CAN_message_t &msg){

}

 void wayfdvl_dist_2(const CAN_message_t &msg){
  
}

 void wayfdvl_dist_3(const CAN_message_t &msg){
  
}

 void wayfdvl_dist_4(const CAN_message_t &msg){
  
}
  // 1x res

////////////////////////////////////////////////////////////////////////////////////////
// 0x0003 MS5837  MS5837 Pressure and Temp Sensor

 void ms5837_data(const CAN_message_t &msg){

}

 void ms5837_depth(const CAN_message_t &msg){

}

 void ms5837_temp(const CAN_message_t &msg){
  
}


////////////////////////////////////////////////////////////////////////////////////////
// Addressing function to find correct jump table address
 void dreq_access(uint16_t *device, uint16_t *topic, const CAN_message_t &msg){
  (*((*dreq_topic + (*device) * (sizeof(void *)))[*topic]))(msg);
}
