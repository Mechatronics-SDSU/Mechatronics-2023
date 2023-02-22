
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

void startup_light_system(bright_lights_t *light_struct){
  light_struct->pin[0] = LIGHT_0_PIN;
  light_struct->pin[1] = LIGHT_1_PIN;
  light_struct->pin[2] = LIGHT_2_PIN;
  light_struct->pin[3] = LIGHT_3_PIN;

  for(int n = 0; n < MAX_BRLIGHTS; n++){
    light_struct->level[n] = 0x00;
    pinMode(light_struct->pin[n], OUTPUT);
    analogWrite(light_struct->pin[n], map2Motor(-100 + 0x00));
  }
}

void set_num_enabled_lights(bright_lights_t *light_struct, uint8_t num_en){
  light_struct->num_enabled = num_en;
}

/*
void set_light_levels(bright_lights_t *light_struct){
  for(int n = 0; n < light_struct->num_enabled; n++){
    analogWrite(light_struct->[n], map2Motor(-100 + 0x00));
  }
}
*/
