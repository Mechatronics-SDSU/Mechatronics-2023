
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
  return ret_val;
}
