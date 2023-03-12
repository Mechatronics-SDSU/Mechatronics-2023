/*
Joseph A De Vico
for questions: @sudo apt install better grades

Please Note: Tab/file names that match CAN message types
  pertain to the handling of functions defined in the CAN protocol
  document.
*/

// CAN Interface code, non-blocking. Tonton81
#include <FlexCAN_T4.h>

// Core essential constants and struct definitions
#include "T4_nodeConstants.h"
#include "device_data_structs.h"

// Interval Timer
#include <IntervalTimer.h>

// Watchdog Timer
#include "Watchdog_t4.h"
WDT_T4<WDT2> commsTimeoutWDT;


// Richard Gemmell T4 i2C Library
#include <i2c_driver.h>
#include <i2c_driver_wire.h>


#ifdef ENABLE_PRES_SENS
// MS5837 Library, Joseph De Vico
#include <T4_MS5837_CONSTANTS.h>
#include <T4_MS5837.h>
#endif

#ifdef ENABLE_DVL
// Wayfinder DVL Library, Joseph De Vico
#include <T4_DVL.h>
DVL_ *WAYFDVL;
#endif

// Include Devices and Topics Jump Tables
//  this is done automatically, don't re include!!!!!

// Acually nvm we do it here to have shared constants globally
#include "T4_installed_devices_and_topics.h"

// Definitions
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

///////////////////////////////////////////////////////////////
//            Datastructures for Devices
///////////////////////////////////////////////////////////////
CONTROL control;

#ifdef ENABLE_PRES_SENS
pressure_sensor_t pressure_sensor;
#endif

bright_lights_t lights;

IntervalTimer killButtonISRTimeout;
IntervalTimer leakDetectionISRTimeout;

///////////////////////////////////////////////////////////////
//            State Variables and Timers
///////////////////////////////////////////////////////////////

// Timer for no response on emergency
//IntervalTimer noResponseTimer;          // Will take over control and shutdown motors if no response to a leak detect


volatile uint32_t OA_STATE = HARD_KILL_STATE;      // Overall State Variable
volatile uint32_t LAST_GOOD_STATE = ALL_GOOD_STATE; // Maintains the last good state, default ALL_GOOD_STATE

// Device Status Banks
volatile uint32_t bank0_device_status = 0;

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
///////////////////////////////////////////////////////////////
//            INITIALIZE PWM
///////////////////////////////////////////////////////////////
  if(setupMainDrives()){
    // Error!
  }

  setup_hard_kill_relay();

  // Enable leak detection
  setup_leak_detection_pins_and_isr();

  // Enable soft kill button input
  startup_kill_button();

  // Startup moved to per device messaging

  pinMode(DEBUG_LED, OUTPUT);
  pinMode(TEENSY_BOARD_LED, OUTPUT);
  analogWrite(TEENSY_BOARD_LED, 1024);  

  // Setup CAN fairly late
  //  to reduce collisions
  //  on ISRs and normal
  //  initialization steps
  if(canSetup()){
    // Error!
  }

  // Initialization ready, blinky light time!
  for(uint32_t n = 0; n < 6; n++){
    digitalToggle(DEBUG_LED);
    delay(500);
  }

#ifdef DEBUG_MODE
  Serial.printf("Initiation Complete.\n\tDebug Mode Enabled\n");
#endif

#ifdef DEBUG_DECODE
  Serial.printf("\tCAN Message Decode Enabled\n");
#endif

#ifdef DEBUG_DREQ_PTR
  Serial.printf("\tDREQ, DRES, STOW Debug Enabled\n");
#endif

  // Indicate EMBSYS has booted
  SET_STATUS_BIT(bank0_device_status, EMBSYS_BIT);

  // Initialize and begin watchdog timer for loss of
  //  CAN communications.
  //  After some time with no valid motor drive inputs the timer will
  //  shutdown the motors
  WDT_timings_t control_loss_wdt;
  control_loss_wdt.trigger = CAN_LOSS_OF_CONTROL_WDT_TIMEOUT;       // WDT Timeout trigger
  control_loss_wdt.timeout = 2.0 * CAN_LOSS_OF_CONTROL_WDT_TIMEOUT; // WDT Timeout limit
  control_loss_wdt.callback = loss_of_valid_control;                // Function Located: Emergency and Config
  commsTimeoutWDT.begin(control_loss_wdt);
//  commsTimeoutWDT.reset();

#ifdef DEBUG_MODE
  Serial.printf("\tCAN Watchdog Started, %.4f s timeout.\n", control_loss_wdt.trigger);
#endif

}


void loop() {
  // Need to have this here
  Can0.events();
  
#ifdef ENABLE_DVL
  if(CHK_STATUS_BIT(bank0_device_status, WAYFDVL_BIT)) DVL_DATA_UPDATE();    // Returns status of DVL serial data update
#endif


  if(OA_STATE < ALL_GOOD_STATE){    
    commsTimeoutWDT.feed();     // Feed watchdog once a shutdown has occured, or if in TEST_IA_STATE 
  }
}
