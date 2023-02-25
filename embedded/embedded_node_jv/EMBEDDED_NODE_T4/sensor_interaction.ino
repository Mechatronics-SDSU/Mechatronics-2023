//#include "T4_installed_devices_and_topics.h"

void sensor_interaction_handler(const CAN_message_t &msg){
  uint16_t err_val = 0;
#ifdef DEBUG_DREQ_PTR
  Serial.println("\nSensor Interaction handler!");
#endif
#ifdef DEBUG_DECODE
  canDecode(msg);
#endif

  if(msg.len < 4) return;

  switch(msg.id){
    case DREQ_ID:           // Main sensor DREQ and DRES mechanism
#ifdef DEBUG_DREQ_PTR
      Serial.printf("DREQ!\n");
#endif
      handle_dreq(msg);
    break;

    case STOW_ID:
#ifdef DEBUG_DREQ_PTR
      Serial.printf("STOW!\n");
#endif
      CAN_message_t msg2 = msg;
      handle_stow(msg2);
    break;

  }

  if(err_val){
    switch(err_val){
    
    }  
  }
}




void handle_dreq(const CAN_message_t &msg){
  // Pre Structure Return Message
  CAN_message_t res_msg;
  res_msg.id = 0x021;

  uint16_t device_queried = ((msg.buf[1] << 8) | (msg.buf[0]));
  uint16_t topic_queried = ((msg.buf[3] << 8) | (msg.buf[2]));

  // Set return Device and Topic for the data being returned
  res_msg.buf[0] = msg.buf[0];
  res_msg.buf[1] = msg.buf[1];
  res_msg.buf[2] = msg.buf[2];
  res_msg.buf[3] = msg.buf[3];

  // Fill Response message with data to be returned
  dreq_access(device_queried, topic_queried, res_msg);

  // Send 021# DRES message back
  Can0.write(res_msg);
}

void handle_stow(CAN_message_t &msg){
  uint16_t device_queried = ((msg.buf[1] << 8) | (msg.buf[0]));
  uint16_t topic_queried = ((msg.buf[3] << 8) | (msg.buf[2]));
  dreq_access(device_queried, topic_queried, msg);  
}
