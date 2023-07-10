// Leak Detection Handling
void setup_leak_detection_pins_and_isr(){
  pinMode(LEAK_DET_PIN_0, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(LEAK_DET_PIN_0), leak_detected_0, RISING);

  pinMode(LEAK_DET_PIN_1, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(LEAK_DET_PIN_1), leak_detected_1, RISING);

  pinMode(LEAK_DET_PIN_2, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(LEAK_DET_PIN_2), leak_detected_2, RISING);

  pinMode(LEAK_DET_PIN_3, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(LEAK_DET_PIN_3), leak_detected_3, RISING);
  
}

void re_enable_leak_detection(){
  leakDetectionISRTimeout.end();

  attachInterrupt(digitalPinToInterrupt(LEAK_DET_PIN_0), leak_detected_0, RISING);
  attachInterrupt(digitalPinToInterrupt(LEAK_DET_PIN_1), leak_detected_1, RISING);
  attachInterrupt(digitalPinToInterrupt(LEAK_DET_PIN_2), leak_detected_2, RISING);
  attachInterrupt(digitalPinToInterrupt(LEAK_DET_PIN_3), leak_detected_3, RISING);
}

void leak_detected_0(){
  shutdownSystem();
  detachInterrupt(LEAK_DET_PIN_0);
  CAN_message_t msg;
  msg.id = 0x001;
  msg.len = 2;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x00;
  Can0.write(msg);

  leakDetectionISRTimeout.begin(re_enable_leak_detection, LEAK_DET_DEBOUNCE_TIMEOUT);
}

void leak_detected_1(){
  shutdownSystem();
  detachInterrupt(LEAK_DET_PIN_1);
  CAN_message_t msg;
  msg.id = 0x001;
  msg.len = 2;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x01;
  Can0.write(msg);

  leakDetectionISRTimeout.begin(re_enable_leak_detection, LEAK_DET_DEBOUNCE_TIMEOUT);
}

void leak_detected_2(){
  shutdownSystem();
  detachInterrupt(LEAK_DET_PIN_2);
  CAN_message_t msg;
  msg.id = 0x001;
  msg.len = 2;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x02;
  Can0.write(msg);

  leakDetectionISRTimeout.begin(re_enable_leak_detection, LEAK_DET_DEBOUNCE_TIMEOUT);
}

void leak_detected_3(){
  shutdownSystem();
  detachInterrupt(LEAK_DET_PIN_3);
  CAN_message_t msg;
  msg.id = 0x001;
  msg.len = 2;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x03;
  Can0.write(msg);

  leakDetectionISRTimeout.begin(re_enable_leak_detection, LEAK_DET_DEBOUNCE_TIMEOUT);
}

/*
void start_no_reponse_timer(){
  noResponseTimer.begin(no_reponse_to_emergency, NO_RESPONSE_TIMEOUT_US);
  noResponseTimer.priority(NO_RESPONSE_ISR_PRIORITY);  
}

void stop_no_reponse_timer(){
  noResponseTimer.end();  
}

void no_reponse_to_emergency(){
  stop_no_reponse_timer();
  shutdownSystem();
  // Start blinking light
}
*/


  ///////////////////////////////////////////////////////////////////////////////
 //                   Kill Button                                             //
///////////////////////////////////////////////////////////////////////////////
void re_enable_kill_button_irq(){ // Allow button IRQs again
  killButtonISRTimeout.end();
  CMP2_SCR |= (1 << 2);           // Clear IRQ raised flag
  digitalToggle(DEBUG_LED);
  NVIC_CLEAR_PENDING(IRQ_ACMP2);
  NVIC_ENABLE_IRQ(IRQ_ACMP2);
}



void kill_button_triggered(){
  __disable_irq();                // Maintain control over ISRs
  CMP2_SCR |= (1 << 2);           // Clear COMP2 trig flag
  //cli();
  NVIC_DISABLE_IRQ(IRQ_ACMP2);    // Disable button IRQ for debounce
  NVIC_CLEAR_PENDING(IRQ_ACMP2);  // Clear ISR Queue for button
  
  shutdownSystem();               // Kill Motors
  hardShutdown();                 // Cut power to motors

  
  digitalToggle(DEBUG_LED);       // Blink Debug LED
  soft_kill_system_message();     // Send CAN0 Message indicating shutdown button press
  //CMP2_SCR |= (1 << 2);           // Clear COMP2 trig flag



  mode_set_message(MANUAL_MODE);
  NVIC_CLEAR_PENDING(IRQ_ACMP3);
  NVIC_ENABLE_IRQ(IRQ_ACMP3);     // Re-Enable Auto Mode set on Kill Trigger
  // Return to 0 State (Manual)
  embsys_status_flags &= ~((1ul << EMBSYS_OPERATION_STAT_0) | (1ul << EMBSYS_OPERATION_STAT_1) | (1ul << EMBSYS_OPERATION_STAT_2) | (1ul << EMBSYS_OPERATION_STAT_3));

  __enable_irq();                 // Allow other IRQs
  killButtonISRTimeout.begin(re_enable_kill_button_irq, KILL_BUTTON_DEBOUNCE_TIMEOUT);
}



  ///////////////////////////////////////////////////////////////////////////////
 //                   Auto Button                                             //
///////////////////////////////////////////////////////////////////////////////
void auto_button_triggered(){
  __disable_irq();

  if(CMP3_SCR & (1 << 2)){
    embsys_status_flags |= (AUTO_MODE << EMBSYS_OPERATION_STAT_0);
  
    mode_set_message(AUTO_MODE);  

    CMP3_SCR |= (1 << 2);   // Clear rising flag
  
    
    NVIC_DISABLE_IRQ(IRQ_ACMP3);
    NVIC_CLEAR_PENDING(IRQ_ACMP3);
  }
  
  
  __enable_irq();
}

//
void mode_set_message(uint8_t mode){
  CAN_message_t msg;
  msg.len = 1;
  msg.id  = MODE_SET_ID;
  msg.buf[0] = mode;
  Can0.write(msg);          // Send Mode Set Message
}
