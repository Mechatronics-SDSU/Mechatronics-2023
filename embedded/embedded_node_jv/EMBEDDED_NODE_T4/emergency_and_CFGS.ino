// This file contains the external handlers for Emergency, Config, and Status messages
void emergency_CFGS_handler(const CAN_message_t &msg){
#ifdef DEBUG_DECODE
  canDecode(msg);
#endif
  if(!msg.id) shutdownSystem();
  else{
    switch(msg.id){
      case 0x001:
      break;
      case 0x002:
      break;
      case 0x003:
      break;
      case 0x004:
      break;
      case 0x005:
        softShutdown(msg);
      break;

      case 0x00A:
        OA_STATE = ALL_GOOD_STATE;
      break;
    }
  }
}

void shutdownSystem(){      // Immediate shutdown of motors, release claw
  cli();                    // Control ISRs
#ifdef DEBUG_MODE
  Serial.println("Shutdown!!");
#endif
  
  OA_STATE = SOFT_KILL_STATE;
  
  for(int n = 0; n < ACTIVE_THRUSTERS; n++){
    // motorGo(uint8_t motor__, uint8_t percent)
    control.thruster[n].value = motorGo(control.thruster[n].pin, 0);
  }

  sei();
}

void softShutdown(const CAN_message_t &msg){        // Slowly shutdown motors and release claw
  uint16_t soft_delay = (msg.buf[0]) ? msg.buf[0] : DEFAULT_SOFT_SHUTDOWN_TIME_MS;  // Select how many ms to shutdown over
  for(int q = 0; q < soft_delay; q++){
    for(int n = 0; n < ACTIVE_THRUSTERS; n++){
      uint16_t new_mot_val =  control.thruster[n].value - (q * (control.thruster[n].value / soft_delay));
      
      control.thruster[n].value = motorGo(control.thruster[n].pin, new_mot_val);
    }
    delayMicroseconds(1000);
  }
}
