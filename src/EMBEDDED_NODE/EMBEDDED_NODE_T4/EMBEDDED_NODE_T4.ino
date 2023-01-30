
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

#ifdef ENABLE_PRES_SENS

// Richard Gemmell T4 i2C Library, for MS5837 Mostly
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

// MS5837 Library, Joseph De Vico
#include <T4_MS5837_CONSTANTS.h>
#include <T4_MS5837.h>



// Wayfinder DVL Library, Joseph De Vico
//#include <DVL_CONSTANTS.h>
//#include <T4_DVL.h>

#endif

// Include Devices and Topics Jump Tables
//  this is done automatically, don't re include!!!!!
//#include "T4_installed_devices_and_topics.h"


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

CONTROL control;

// Pressure Sensor Data Struct
pressure_sensor_t installed_MS5837;

#ifdef ENABLE_PRES_SENS
pressure_sensor_t pressure_sensor;
#endif

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
///////////////////////////////////////////////////////////////
//            INITIALIZE PWM AND CAN0
///////////////////////////////////////////////////////////////
  if(setupMainDrives()){
    // Error!
  }
  if(canSetup()){
    // Error!
  }


#ifdef ENABLE_PRES_SENS
  // Start i2C 0 at 400kHz, initiate pressure sensor
  startup_pressure_sensor( &pressure_sensor);
#endif

  pinMode(DEBUG_LED, OUTPUT);
  pinMode(TEENSY_BOARD_LED, OUTPUT);
  analogWrite(TEENSY_BOARD_LED, 1024);  

  // Initialization ready, blinky light time!
  for(uint32_t n = 0; n < 5; n++){
    digitalWriteFast(DEBUG_LED, 1);
    delay(500);
    digitalWriteFast(DEBUG_LED, 0);
  }

}


void loop() {
  // Need to have this here
  Can0.events();

#ifdef ENABLE_PRES_SENS
  if(ms5873_Data_ready() && !pressure_sensor.health){
    float_2_char_array(pressure_sensor.depth, ms5873_Read_Depth());
    float_2_char_array(pressure_sensor.average_depth, ms5873_Avg_Depth());
    float_2_char_array(pressure_sensor.temperature, ms5873_Read_Temp());
    float_2_char_array(pressure_sensor.average_temperature, ms5873_Avg_Temp());
  }
 
#endif

}
