// This file contains handlers for motor, arm, and thruster input/output messages
// motorGo(uint8_t motor__, uint8_t percent)
void motor_impulse_handler(const CAN_message_t &msg){
#ifdef DEBUG_DECODE
  canDecode(msg);
#endif
  if(OA_STATE == ALL_GOOD_STATE){
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
