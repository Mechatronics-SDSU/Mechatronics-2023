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

void setup_soft_kill_button(){
  pinMode(SOFT_KILL_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(SOFT_KILL_PIN), soft_kill_system, RISING);
}

void soft_kill_system(){
  CAN_message_t msg;
  msg.id = 0x000;
  msg.len = 0;
  Can0.write(msg);
  shutdownSystem();
}

void leak_detected_0(){
  CAN_message_t msg;
  msg.id = 0x001;
  msg.len = 2;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x00;
  Can0.write(msg);
  shutdownSystem();
  //start_no_reponse_timer();
}

void leak_detected_1(){
  CAN_message_t msg;
  msg.id = 0x001;
  msg.len = 2;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x01;
  Can0.write(msg);
  shutdownSystem();
  //start_no_reponse_timer();
}

void leak_detected_2(){
  CAN_message_t msg;
  msg.id = 0x001;
  msg.len = 2;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x02;
  Can0.write(msg);
  shutdownSystem();
  //start_no_reponse_timer();
}

void leak_detected_3(){
  CAN_message_t msg;
  msg.id = 0x001;
  msg.len = 2;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x03;
  Can0.write(msg);
  shutdownSystem();
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
