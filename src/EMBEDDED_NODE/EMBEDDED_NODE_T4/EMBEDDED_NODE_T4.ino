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

// MS5837 Library, Joseph D
#include "T4_MS5837.h"

// Richard Gemmell T4 i2C Library, for MS5837 Mostly
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

#endif

// Include Devices and Topics Jump Tables
//  this is done automatically, don't re include!!!!!
//#include "T4_installed_devices_and_topics.h"


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

CONTROL control;

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

/*
  // for at home (ie no CAN) debugging of DRES
  // 03 01 pres depth
  if(Serial.read() == 'A'){
    uint16_t _fake_dev = 3;
    uint16_t _fake_top = 1;
    CAN_message_t fake_msg;
    fake_msg.id = 0x021;

    while(!Serial.available());

    switch(Serial.read()){
      case '0':
        _fake_top = 0;
      break;
      case '1':
        _fake_top = 1;
      break;
      case '2':
        _fake_top = 2;
      break;
      case '3':
        _fake_top = 3;
      break;
    }
    
    dreq_access(_fake_dev, _fake_top,  fake_msg);  
  }
*/



#ifdef ENABLE_PRES_SENS
  update_pres_data();
#endif

}
