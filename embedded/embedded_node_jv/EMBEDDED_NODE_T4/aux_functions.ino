void aux_function_handler(const CAN_message_t &msg){
  switch(msg.id){
    case 0x1F0:
      readback_message(msg);
    break;

    case 0x1F1:
      morse_debug_led(msg);
    break;
  }
}


// 2047 == 0x7FF == Max CAN2.0A ID



void readback_message(const CAN_message_t &msg){    // Readback the exact message recieved to the sender
  Can0.write(msg);
}


void morse_debug_led(const CAN_message_t &msg){     // Blink the pattern of the first 8 bits sent. LSB first.
  
  for(int b = 0; b < msg.len; b++){   // Count thru bytes sent
    for(unsigned int n = 0; n < 8; n++){       // Set LED to bit in byte
      if(msg.buf[b] & (1u << (7u - n))){
        digitalWriteFast(DEBUG_LED, 1);   
      } else {
        digitalWriteFast(DEBUG_LED, 0);
      }
      delay(250);
    }
  }
}
