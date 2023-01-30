#ifndef T4_NODECONSTANTS_h
#define T4_NODECONSTANTS_h

// STATE CONTROL DEFINES, THESE ARE IMPORTANT!!!!


  // Define USB Serial Print Debug Messaging
#define DEBUG_MODE
#define DEBUG_DECODE
//#define DEBUG_DREQ_PTR




  // Define Enabled Devices
#define ENABLE_PRES_SENS    // Enables i2c0 and MS5837 code


// END STATE CONTROL DEFINES

#define TEENSY_BOARD_LED    13        // LED on the Teensy 4.X board itself
#define DEBUG_LED           21        // Debug LED Pin

#define DEADZONE  0.03f       // +/- 3% deadzone

// us equivalents provided for print debug functions,
//  they serve no integral purpose
#define MID_US      1500      //  1500us mid freq
#define MAX_US      2000
#define MIN_US      1000

#define PWM_RES     16                    // 16 bit range
#define MAX_16      65535                 // 16 bit range
#define CT_PER_US   26.214f               // Timer counts per microsecond
#define MID_CT      39322                 // 1500 us
#define MAX_CT      52428                 // 2000 us
#define MIN_CT      26214                 // 1000 us
#define DEAD_CT     (DEADZONE * MID_CT)
#define DEAD_HI_CT  (MID_CT + DEAD_CT)
#define DEAD_LO_CT  (MID_CT - DEAD_CT)

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

// Submarine and Junebug Motor Mappings
#define MOTOR_0  2   // FlexPWM 4.2
#define MOTOR_1  3   // FlexPWM 4.2

// Sub exclusive motor mappings
#define MOTOR_2  4   // FlexPWM 2.0
#define MOTOR_3  5   // FlexPWM 2.1
#define MOTOR_4  6   // FlexPWM 2.2
#define MOTOR_5  7   // FlexPWM 1.3
#define MOTOR_6  8   // FlexPWM 1.3
#define MOTOR_7  9   // FlexPWM 2.2

// For Junebug ONLY
//#define ACTIVE_THRUSTERS  2
// For Sub ONLY
#define ACTIVE_THRUSTERS 8



#define DEFAULT_SOFT_SHUTDOWN_TIME_MS   250

struct THRUSTER{
  uint8_t pin;
  uint16_t value;
};

struct CONTROL{
  //uint8_t thruster[ACTIVE_THRUSTERS];
  THRUSTER thruster[ACTIVE_THRUSTERS];
};

#endif // T4_NODECONSTANTS_h
