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

#ifdef ENABLE_PRES_SENS

// Richard Gemmell T4 i2C Library, for MS5837 Mostly
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

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
//#include "T4_installed_devices_and_topics.h"

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


volatile uint32_t OA_STATE = SOFT_KILL_STATE;      // Overall State Variable

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
  

  // Enable leak detection
  setup_leak_detection_pins_and_isr();

  // Enable soft kill button input
  startup_kill_button();

  startup_pressure_sensor( &pressure_sensor);

  #ifdef ENABLE_DVL
  init_DVL_serial();
  WAYFDVL = init_DVL_data_struct();
  DVLSERIAL.clear();
      #ifdef DEBUG_MODE
      Serial.printf("Serial 1\nTX Capacity:\t%d bytes\nRX Capacity:\t%d bytes\n", DVLSERIAL.availableForWrite(), DVLSERIAL.available());
      Serial.printf("DVL Struct: %08X\n", WAYFDVL);
      #endif
  #endif


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

}


void loop() {
  // Need to have this here
  Can0.events();
  
#ifdef ENABLE_DVL
  DVL_DATA_UPDATE();    // Returns status of DVL serial data update
#endif
  
  
#ifdef ENABLE_PRES_SENS
/*
  if(ms5837_Data_ready() && !pressure_sensor.health){
    
    float_2_char_array(pressure_sensor.depth, ms5837_Read_Depth());
    float_2_char_array(pressure_sensor.average_depth, ms5837_Avg_Depth());
    float_2_char_array(pressure_sensor.temperature, ms5837_Read_Temp());
    float_2_char_array(pressure_sensor.average_temperature, ms5837_Avg_Temp());
  }
 */
#endif

}
