
#ifdef ENABLE_PRES_SENS
void startup_pressure_sensor(pressure_sensor_t *pres_sense_struct){
///////////////////////////////////////////////////////////////
//            INITIALIZE i2c0 AND MS5837 DEPTH SENSOR
///////////////////////////////////////////////////////////////

  const char _bar30_name[7] = "MSBAR30";
  const char _bar02_name[7] = "MSBAR02";

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
    Serial.printf("Model Decoded: %s\n", B02_STR);
#endif
  } else {
    for(uint32_t n = 0; n < 7; n++) pres_sense_struct->model[n] = _bar30_name[n];
#ifdef DEBUG_MODE
    Serial.printf("Model Decoded: %s\n", B30_STR);
#endif
  }
  
}
#endif
