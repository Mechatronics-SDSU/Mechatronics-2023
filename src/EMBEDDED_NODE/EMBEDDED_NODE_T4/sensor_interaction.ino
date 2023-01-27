
void sensor_interaction_handler(const CAN_message_t &msg){
#ifdef DEBUG_DECODE
  canDecode(msg);
#endif
  switch(msg.id){
    case 0x020:           // Main sensor DREQ and DRES mechanism

    break;



    
  }
  
}
