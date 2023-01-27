/*
Joseph A De Vico
for questions: @sudo apt install better grades
*/

#include <FlexCAN_T4.h>
#include "T4_nodeConstants.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

CONTROL control;


void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif

  if(setupMainDrives()){
    // Error!
  }
  if(canSetup()){
    // Error!
  }

  pinMode(DEBUG_LED, OUTPUT);
  pinMode(TEENSY_BOARD_LED, OUTPUT);
  analogWrite(TEENSY_BOARD_LED, 1024);  
  

}


void loop() {
  Can0.events();

}
