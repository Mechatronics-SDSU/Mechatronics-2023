
#ifdef ENABLE_PRES_SENS
void startup_pressure_sensor(pressure_sensor_t *pres_sense_struct){
///////////////////////////////////////////////////////////////
//            INITIALIZE i2c0 AND MS5837 DEPTH SENSOR
///////////////////////////////////////////////////////////////

  const char _bar30_name[8] = "MSBAR30";
  const char _bar02_name[8] = "MSBAR02";

  // Initiate 400k i2C 0, MS5837 main dependant
  FAST_I2C.begin();
  FAST_I2C.setClock(FAST_I2C_FREQ);
  
  uint16_t e_ct = 0;
  while(!ms5837_init()){
#ifdef DEBUG_MODE  
    Serial.printf("%d\tError on Init!!\n\n", e_ct++);
    delay(1000);
#endif
  }

  if(ms5837_getModel() == MS5837_02BA_ID){
    for(uint32_t n = 0; n < 7; n++) pres_sense_struct->model[n] = _bar02_name[n];
#ifdef DEBUG_MODE
    Serial.printf("Model Decoded: %s\n", _bar02_name);
#endif
  } else {
    for(uint32_t n = 0; n < 7; n++) pres_sense_struct->model[n] = _bar30_name[n];
#ifdef DEBUG_MODE
    Serial.printf("Model Decoded: %s\n", _bar30_name);
#endif
  }

  pres_sense_struct->health = 0x00;

  ms5837_loop_begin();
  
}
#endif

//////////////////////////////////////////////////////
//            Illumination Lights (BRLIGHT)
//////////////////////////////////////////////////////
void startup_light_system(bright_lights_t *light_struct){
  light_struct->pin[0] = LIGHT_0_PIN;
  light_struct->pin[1] = LIGHT_1_PIN;


  for(int n = 0; n < MAX_BRLIGHTS; n++){
    light_struct->level[n] = 0x00;
    pinMode(light_struct->pin[n], OUTPUT);
    analogWrite(light_struct->pin[n], map2Motor(BRLIGHT_MAP_OFFSET));
    analogWriteFrequency(light_struct->pin[n], BRLIGHT_PWM_FREQ);
  }
}

void set_num_enabled_lights(bright_lights_t *light_struct, uint8_t num_en){
  light_struct->num_enabled = num_en;
}


void set_light_levels(bright_lights_t *light_struct, uint8_t vals){
  for(int n = 0; n < light_struct->num_enabled; n++){
    analogWrite(light_struct->pin[n], map2Motor(BRLIGHT_MAP_OFFSET + (vals << 1)));
  }
}


//////////////////////////////////////////////////////
//          LED Button 
//////////////////////////////////////////////////////
void startup_kill_button(){               // Button on A14 with 4k7, 100n RCLP
  CCM_CCGR3 |= CCM_CCGR3_ACMP2(CCM_CCGR_ON);  // Enable ACMP2 Clock

  // CMP2 Setup for A14 Input vs. Internal DAC
  CMP2_CR0 = 0b01000011; // Set Filter count and Hysteresis controls
  CMP2_CR1 = 0b00000001; // Mode 4B
  CMP2_FPR = 64;         // Divide periph clock to COMP sample
  //CMP2_DACCR = (1 << 7) | (0 << 6) | 31; // Enable DAC, set threshold
  set_comp2_dac(2);
  CMP2_SCR = 0b00010000;  // Enable ISR
  CMP2_MUXCR = 0b00110111;  // +in = A14, -in = DAC

  pinMode(KILL_BUTTON_PIN, INPUT_PULLDOWN);

  // Setup IRQ, ACMP2 ISR -> 124
  attachInterruptVector(IRQ_ACMP2, &kill_button_triggered);
  NVIC_CLEAR_PENDING(IRQ_ACMP2);
  NVIC_ENABLE_IRQ(IRQ_ACMP2);

  killButtonISRTimeout.priority(100);

  __enable_irq();
}

void set_comp2_dac(uint8_t val){  // Set DAC, max 64
  CMP2_DACCR = (uint8_t)((1 << 7) | (val & 0b111111));
}


// Auto button
void startup_auto_button(){               // Button on A15 with 4k7, 100n RCLP
  CCM_CCGR3 |= CCM_CCGR3_ACMP3(CCM_CCGR_ON);  // Enable ACMP3 Clock

  // CMP3 Setup for A15 Input vs. Internal DAC
  CMP3_CR0 = 0b01000011; // Set Filter count and Hysteresis controls
  CMP3_CR1 = 0b00000001; // Mode 4B
  CMP3_FPR = 64;         // Divide periph clock to COMP sample
  //CMP3_DACCR = (1 << 7) | (0 << 6) | 31; // Enable DAC, set threshold
  set_comp3_dac(2);
  CMP3_SCR = 0b00010000;  // Enable ISR
  CMP3_MUXCR = 0b00110111;  // +in = A15, -in = DAC

  pinMode(AUTO_BUTTON_PIN, INPUT_PULLDOWN);

  // Setup IRQ, ACMP3 ISR -> 124
  attachInterruptVector(IRQ_ACMP3, &auto_button_triggered);
  NVIC_CLEAR_PENDING(IRQ_ACMP3);
  NVIC_ENABLE_IRQ(IRQ_ACMP3);

  __enable_irq();
}

void set_comp3_dac(uint8_t val){  // Set DAC, max 64
  CMP3_DACCR = (uint8_t)((1 << 7) | (val & 0b111111));
}


//////////////////////////////////////////////////////
//          Hard Kill Relay Output, Active High 
//////////////////////////////////////////////////////
void setup_hard_kill_relay(){
  pinMode(HARD_KILL_RELAY, OUTPUT);
  digitalWriteFast(HARD_KILL_RELAY, 0);
}

void set_hard_kill_relay_state(bool s8__){
  digitalWriteFast(HARD_KILL_RELAY, s8__);
}
