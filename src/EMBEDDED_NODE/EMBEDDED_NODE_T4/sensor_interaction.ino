
void sensor_interaction_handler(const CAN_message_t &msg){
  uint16_t err_val = 0;
#ifdef DEBUG_DECODE
  canDecode(msg);
#endif
  switch(msg.id){
    case 0x020:           // Main sensor DREQ and DRES mechanism
      err_val = handle_dreq(msg);
    break;



    
  }

  if(err_val){
    switch(err_val){
    
    }  
  }
}


// MAIN 0x020# DREQ FUNCTION, VERY IMPORTANT!
uint16_t handle_dreq(const CAN_message_t &msg){
  // best way to find sensor? switch -> hash? idk

  uint16_t ret_val = 0;

  if(msg.len == 4){         // If we have a message of appropriate length
    // Isolate requested device
    uint16_t device_requested = ((msg.buf[1] << 8) | msg.buf[0]);

    // Isolate requested topic
  //  uint16_t topic_requested = ((msg.buf[3] << 8) | msg.buf[2]);
  
    switch(device_requested){
    
    } 
  } else {
    ret_val = 1;            // Incorrect Length
  }

  return ret_val;
}
