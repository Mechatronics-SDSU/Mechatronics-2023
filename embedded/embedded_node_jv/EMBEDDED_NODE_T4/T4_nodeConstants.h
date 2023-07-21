#ifndef T4_NODECONSTANTS_h
#define T4_NODECONSTANTS_h

#define CPU_RESET_CYCLECOUNTER    do { ARM_DEMCR |= ARM_DEMCR_TRCENA;          \
                                       ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; \
                                       ARM_DWT_CYCCNT = 0; } while(0)


// STATE CONTROL DEFINES, THESE ARE IMPORTANT!!!!


  // Define USB Serial Print Debug Messaging
//#define DEBUG_MODE
#define NO_DVL_DEBUG
//#define DEBUG_DECODE
//#define DEBUG_DREQ_PTR
//#define DEBUG_STOW_ACCESS


#define CAN_LOSS_OF_CONTROL_WDT_TIMEOUT   1.0


  // Define Enabled Devices
#define ENABLE_PRES_SENS    // Enables i2c0 and MS5837 code
#define ENABLE_DVL

// END STATE CONTROL DEFINES

#define TEENSY_BOARD_LED    13        // LED on the Teensy 4.X board itself
#define DEBUG_LED           21        // Debug LED Pin

#define DEADZONE  0.03f       // +/- 3% deadzone

// us equivalents provided for print debug functions,
//  they serve no integral purpose
#define MID_US      1500      //  1500us mid freq

#define BRESC

#ifdef BRESC
#define MAX_US      1900
#define MIN_US      1100
#else
#define MAX_US      2000
#define MIN_US      1000
#endif

#define PWM_RES     16                    // 16 bit range
#define MAX_16      65535                 // 16 bit range
#define CT_PER_US   26.214f               // Timer counts per microsecond
#define MID_CT      39322                 // 1500 us

#ifdef BRESC
#define MAX_CT      49807                 // 1900 us
#define MIN_CT      28835                 // 1100 us
#else
#define MAX_CT      52428                 // 2000 us
#define MIN_CT      26214                 // 1000 us
#endif

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
#define ACTIVE_THRUSTERS  8
#define THRUSTER_PWM_FREQ 400


/*  600MHz cyc period -> 1.6666 ns
 *  50ms / 1.666ns = 30E6 = 30M
 */
 // now 250ms
#define ESC_SIGNAL_RESET_CYCLES  300000000ul

#define MAX_BRLIGHTS      2

#define DEFAULT_SOFT_SHUTDOWN_TIME_MS   250

struct THRUSTER{
  uint8_t pin;
  uint16_t value;
};

struct CONTROL{
  //uint8_t thruster[ACTIVE_THRUSTERS];
  THRUSTER thruster[ACTIVE_THRUSTERS];
};

// Hard Kill Output
#define HARD_KILL_RELAY 26
#define RELAY_ON    1
#define RELAY_OFF   0

// Leak Detection pins, trigger on HI
#define LEAK_DET_DEBOUNCE_TIMEOUT 1000000ul
#define LEAK_DET_PIN_0  30
#define LEAK_DET_PIN_1  31
#define LEAK_DET_PIN_2  32
#define LEAK_DET_PIN_3  33

// Kill Button input, trigger on HI
#define KILL_BUTTON_PIN   38
#define KILL_BUTTON_DEBOUNCE_TIMEOUT  500000

// Auto Mode Button input, trigger on HI
#define AUTO_BUTTON_PIN   39
#define MODE_SET_ID       0x007
#define MANUAL_MODE       0x00
#define AUTO_MODE         0x04

// Light PWM pins
#define BRLIGHT_PWM_FREQ    400
#define BRLIGHT_MAP_OFFSET  -100
#define LIGHT_0_PIN     28
#define LIGHT_1_PIN     29

// LED Button Light Pins (Same Timer)
#define BUTTON_LED_BLINK_RATE   1
#define KILL_BUTTON_GRN_LED     37
#define KILL_BUTTON_BLU_LED     36

// No Reponse ISR and Timer
#define NO_RESPONSE_TIMEOUT_US    1000000
#define NO_RESPONSE_ISR_PRIORITY  64

#define NEW_DREQ_SCHEME
// State Machine States, OA_STATE
/*    0   Hard Kill
 *    1   RES
 *    2   Soft Kill
 *    3   Testing in Progress, same as all good but with no motor timeout
 *    4   All Good, clear to run
 *    5
 *    6
 *    7
 */
 #define HARD_KILL_STATE    0
 // State 1 Reserved
 #define SOFT_KILL_STATE    2
 #define TEST_IA_STATE      3
 #define ALL_GOOD_STATE     4

#define DEV_DATA_0  4
#define DEV_DATA_1  5
#define DEV_DATA_2  6
#define DEV_DATA_3  7

// length in bytes of a standard DRES per payload len in bytes
#define DEV_PAY_LEN_0   4
#define DEV_PAY_LEN_1   5
#define DEV_PAY_LEN_2   6
#define DEV_PAY_LEN_3   7
#define DEV_PAY_LEN_4   8

#define DREQ_ID   0x20
#define DRES_ID   0x21
#define STOW_ID   0x22

#define INVALID_ACCESS_ID 0x24

#define RESET_ESC_ID  0x18


// EMBSYS Internal Status Bitfield

#define EMBSYS_DREQ_ERRCHK  0
#define EMBSYS_DREQ_RDBK    1

#define EMBSYS_OPERATION_STAT_0   28      // For auto and manual mode setting
#define EMBSYS_OPERATION_STAT_1   29      // See above for state IDs
#define EMBSYS_OPERATION_STAT_2   30
#define EMBSYS_OPERATION_STAT_3   31


#define EMBSYS_DFL_STATE   ( (1ul << EMBSYS_DREQ_RDBK) | (1ul << EMBSYS_DREQ_ERRCHK) )

#endif // T4_NODECONSTANTS_h
