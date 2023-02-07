/*
Joseph A De Vico
for questions: @sudo apt install better grades
Teensy 4.X PWM Validation for ESC Driver
Teensy 4.X Port of Parametric ESC Drive MAP function
10/20/2022mdy
*/

/*
Definitions:
  COUNT, Count    Unsigned 16 bit value, usually used as a compare match
  US, us          Microseconds
*/
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
#define PWMPIN  2   // FlexPWM 4.2
//#define PWMPIN  15  //  Quadtimer3.3

// To Calculate Count Values:
//  1 / (FREQ_HZ)  *  COUNT / MAX_16 == On Time Seconds
//  FREQ_HZ * TIME_s * MAX_16 == Count for TIME_s ON time
//
#define MAX_16      65535                 // 16 bit range
#define CT_PER_US   26.214f               // Timer counts per microsecond
#define MID_CT      39322                 // 1500 us
#define MAX_CT      52428                 // 2000 us
#define MIN_CT      26214                 // 1000 us
#define DEAD_CT     (DEADZONE * MID_CT)
#define DEAD_HI_CT  (MID_CT + DEAD_CT)
#define DEAD_LO_CT  (MID_CT - DEAD_CT)

void setup() {
  // put your setup code here, to run once:
  analogWriteFrequency(PWMPIN, 400);        // 400 Hz hardcoded. Do not change.
  analogWriteResolution(16);                // Full hardware resolution avail at these slow speeds. Do not change.
  analogWrite(PWMPIN, map2Motor(100.0f));   // Initialize to 100% throttle.

  Serial.begin(115200);

  delay(500);

  // Below is the standard calling convention to put whatever is on MOTOR_PWM_PIN to 100% throttle
  // We can also use 8 bit representations that have a range of +/- 100 with a resolution of 0.5 (1 LSB fixed point)
  //  such that within a single CAN message we can write to 8 motors at once. This is up to implementation at a later
  //  date.
  //analogWrite(MOTOR_PWM_PIN, map2Motor(100.0));
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint8_t chg = 0;
  static uint8_t adv = 0;

  uint16_t to_wr = 0;

  if(Serial.available()){
    uint8_t inc_data = Serial.read();     // Sending any byte will change the output mode.
    if(inc_data){                         // Standard arduino IDE serial monitor -> press ctrl + enter -> observe changes on scope
      chg = 1;
      adv += 1;
    }
  }

  if(chg){
    chg = 0;
    switch(adv){
      case 0:
        to_wr = map2Motor(-100.0f);
        Serial.printf("Speed:\t-100%\t%u\n", to_wr);
        analogWrite(PWMPIN, to_wr);
      break;
      case 1:
        to_wr = map2Motor(-50.0f);
        Serial.printf("Speed:\t-50%\t%u\n", to_wr);
        analogWrite(PWMPIN, to_wr);
      break;
      case 2:
        to_wr = map2Motor(0.0f);
        Serial.printf("Speed:\t0%\t%u\n", to_wr);
        analogWrite(PWMPIN, to_wr);
      break;
      case 3:
        to_wr = map2Motor(50.0f);
        Serial.printf("Speed:\t50%\t%u\n", to_wr);
        analogWrite(PWMPIN, to_wr);
      break;
      case 4:
        to_wr = map2Motor(100.0f);
        Serial.printf("Speed:\t100%\t%u\n", to_wr);
        analogWrite(PWMPIN, to_wr);
      break;
      default:
        adv = 0;
        chg = 1;
      break;    
    }
  }
  /*  // Fade thru -100 - +100 and back, notice the jump when the deadzone at 0 is crossed
  for(uint8_t n = 0; n < 100; n++){
    analogWrite(PWMPIN, map2Motor((float)n));
    delay(10);
  }
  for(uint8_t n = 100; n != 0; n--){
    analogWrite(PWMPIN, map2Motor((float)n));
    delay(10);
  }
  for(int8_t n = 0; n > -100; n--){
    analogWrite(PWMPIN, map2Motor((float)n));
    delay(10);
  }
  for(int8_t n = -100; n < 0; n++){
    analogWrite(PWMPIN, map2Motor((float)n));
    delay(10);
  }
   */
}


uint16_t map2Motor(float input_val__){    // Scale -100% - 100% to valid uint16_t value
  // Notice the deadzone is not considered here, rather we are concerned with the valid range
  //  esc hardware has a built in deadzone to account for us having a bad 0% stability (we have good stability),
  //  however this means we must map our 0% -> MIN or 0% -> MAX to the range outside of the deadzone.
  
  // If zero immediately return, this is the fastest response as it should be
  if(input_val__ == 0.0f){
    return MID_CT;
  }

  // Bounds restrict input
  if(input_val__ > 100.0f){
    input_val__ = 100.0f;
  } else
  if(input_val__ < -100.0f){
    input_val__ = -100.0f;
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