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
  NVIC_CLEAR_PENDING(IRQ_ACMP2);  // Clear ISR Queue for button
  NVIC_DISABLE_IRQ(IRQ_ACMP2);    // Disable button IRQ for debounce
  shutdownSystem();               // Kill Motors
  __enable_irq();                 // Allow other IRQs
  digitalToggle(DEBUG_LED);       // Blink Debug LED
  soft_kill_system_message();     // Send CAN0 Message indicating shutdown button press
  CMP2_SCR |= (1 << 2);           // Clear COMP2 trig flag

  

  killButtonISRTimeout.begin(re_enable_kill_button_irq, KILL_BUTTON_DEBOUNCE_TIMEOUT);
}


void soft_kill_system_message(){
  CAN_message_t msg;
  msg.id = 0x000;
  msg.len = 0;
  Can0.write(msg);
}
