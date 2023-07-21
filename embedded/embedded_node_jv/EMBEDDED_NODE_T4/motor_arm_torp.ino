// This file contains handlers for motor, arm, and thruster input/output messages
// motorGo(uint8_t motor__, uint8_t percent)
void motor_impulse_handler(const CAN_message_t &msg){
#ifdef DEBUG_DECODE
  canDecode(msg);
#endif
  if(OA_STATE > SOFT_KILL_STATE){
    
    commsTimeoutWDT.feed();     // Indicate valid motor control messaging has been sent
    
    switch(msg.id){
      case 0x010:
        thruster_driver(msg);
      break;
      case 0x011:
        thruster_slew_driver(msg);
      break;
      case 0x12:
        // Direct set motor val
      break;

      case RESET_ESC_ID:
        reset_esc_procedure();  // This takes a ton of time and stalls the system. Please don't call this unless you know what you're doing.
      break;
    }
  }
}


void thruster_driver(const CAN_message_t &msg){
#ifdef DEBUG_MODE
  Serial.println("Motor Drive");
#endif

  int top;
  if(msg.len < ACTIVE_THRUSTERS){
    top = msg.len;
  } else {
    top = ACTIVE_THRUSTERS;
  }
  for(int n = 0; n < top; n++){
#ifdef DEBUG_MODE
    Serial.printf("Motor\t%i\t%2X%\n", control.thruster[n].pin, msg.buf[n]);
#endif
    control.thruster[n].value = motorGo(control.thruster[n].pin, msg.buf[n]);
  }
}

void thruster_slew_driver(const CAN_message_t &msg){
#ifdef DEBUG_MODE
  Serial.println("Slew Motor Drive");
#endif

  int top;
  if(msg.len < ACTIVE_THRUSTERS){
    top = msg.len;
  } else {
    top = ACTIVE_THRUSTERS;
  }

  for(int q = 0; q < 100; q++){
    for(int n = 0; n < top; n++){
      control.thruster[n].value = motorGo(control.thruster[n].pin, msg.buf[n] / (100 - q));
    }
    delayMicroseconds(500);
  }
}




void reset_esc_procedure(){
  cli();                      // IMPORTANT PLEASE NOTE THIS DOES THIS IF YOU ARE UNSURE OF WHAT YOU'RE DOING
  commsTimeoutWDT.feed();
  for(int n = 0; n < ACTIVE_THRUSTERS; n++){
    motor_signal_reset(control.thruster[n].pin);
  }
  // Must utilize isr independent counters, thus using cycle counter
  //  Resetting counter has some weird issues every once in a while,
  //  falling back to standard unsigned subtraction
  //CPU_RESET_CYCLECOUNTER;
  uint32_t start_val = ARM_DWT_CYCCNT;
  while((ARM_DWT_CYCCNT - start_val) < ESC_SIGNAL_RESET_CYCLES);
  commsTimeoutWDT.feed();

  for(int n = 0; n < ACTIVE_THRUSTERS; n++){
    // motorGo(uint8_t motor__, uint8_t percent)
    control.thruster[n].value = motorGo(control.thruster[n].pin, 0);
  }
  sei();                    // SEE ABOVE ALL CAPS MESSAGE
}
