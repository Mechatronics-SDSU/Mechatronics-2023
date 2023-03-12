/*
Definitions:
  COUNT, Count    Unsigned 16 bit value, usually used as a compare match
  US, us          Microseconds
*/
/*
#define DEADZONE  0.03f       // +/- 3% deadzone

// us equivalents provided for print debug functions,
//  they serve no integral purpose
#define MID_US      1500      //  1500us mid freq
#define MAX_US      2000
#define MIN_US      1000

#define DEAD_US     ((uint16_t)(DEADZONE * (float)MID_US))
#define DEAD_HI_US     (MID_US + DEAD_US)
#define DEAD_LO_US     (MID_US - DEAD_US)

#define US_DELTA    (MAX_US - DEAD_HI)

// Review So Far: QuadTimer is reasonably good
//                FlexTimer is also okay. May be better since the pinout lends itself to many PWM at the same f (what we need)
//  Error Measured:
//          Seems to be sub 1% for both, which should be within whatever made up tolerance band we assumed was fine before
//          The rise times are not always ultra fast (ie. inconsistent) and can cause false measurements due to fixed
//          scope trigger levels. Be careful when measuring.
#define MOTOR_0  2   // FlexPWM 4.2
#define MOTOR_1  3   // FlexPWM 4.2
*/
//#define MOTOR_0  15  //  Quadtimer3.3

// To Calculate Count Values:
//  1 / (FREQ_HZ)  *  COUNT / MAX_16 == On Time Seconds
//  FREQ_HZ * TIME_s * MAX_16 == Count for TIME_s ON time
//

void motorInit(){
  control.thruster[0].pin = MOTOR_0;
  control.thruster[1].pin = MOTOR_1;
  
  control.thruster[2].pin = MOTOR_2;
  control.thruster[3].pin = MOTOR_3;
  control.thruster[4].pin = MOTOR_4;
  control.thruster[5].pin = MOTOR_5;
  control.thruster[6].pin = MOTOR_6;
  control.thruster[7].pin = MOTOR_7;
}

uint16_t map2Motor(float input_val__){    // Scale -100% - 100% to valid uint16_t value
  // Notice the deadzone is not considered here, rather we are concerned with the valid range
  //  esc hardware has a built in deadzone to account for us having a bad 0% stability (we have good stability),
  //  however this means we must map our 0% -> MIN or 0% -> MAX to the range outside of the deadzone.
  
  // If zero immediately return, this is the fastest response as it should be
  if(input_val__ == 0.0f){
    return MID_CT;
  }

  // Piecewise assignment function due to deadzone handling. Alternatively a single line with
  //  a vertical offset of 2 * DEADZONE_CT could work. We do math so fast who really cares tho
  if(input_val__ < 0.0f){        // Sign bit set, number is negative
    return (uint16_t)(((DEAD_LO_CT - MIN_CT) * (input_val__ / 100.0f)) + DEAD_LO_CT);
  } else {
    return (uint16_t)(((MAX_CT - DEAD_HI_CT) * (input_val__ / 100.0f)) + DEAD_HI_CT);
  }

  // Catchall condition, we should never get here but if we mess up somewhere else
  //  it's just good to have.
  return MID_CT;
}

uint16_t motorGo(uint8_t motor__, uint8_t percent){            // Wrapper with auto casting for motor controls
  int8_t inter = (int8_t)percent;
  if(!(inter > 100 || inter < -100)){
    uint16_t val_2_write = map2Motor((float)((int8_t)percent));
    analogWrite(motor__, val_2_write);
    return val_2_write;                                         // Returns current PWM compare value if valid
  }
  return 0;
}



int setupMainDrives(){
  motorInit();

#ifdef DEBUG_MODE
  Serial.println("Thrusters on pins:");
  for(int n = 0; n < ACTIVE_THRUSTERS; n++){
    Serial.printf("Thruster %d on pin %d\n", n, control.thruster[n].pin);
  }
  Serial.println('\n');
#endif

  for(int n = 0; n < ACTIVE_THRUSTERS; n++){
    analogWriteFrequency(control.thruster[n].pin, THRUSTER_PWM_FREQ); // 400Hz Hardcoded. Do not change.
  }

  analogWriteResolution(PWM_RES);                       // Full hardware resolution avail at these slow speeds. Do not change.
  
  for(int n = 0; n < ACTIVE_THRUSTERS; n++){            // Initialize all motors to 0
    // motorGo(uint8_t motor__, uint8_t percent)
    control.thruster[n].value = motorGo(control.thruster[n].pin, 0);
  }
  
  return 0;
}
