



#ifdef ENABLE_PRES_SENS
void update_pres_data(const pressure_sensor_t *pres_sense_struct){
  // Check if async timer says MS5837 is clear to read
  if(ms5837_Data_ready(){
    pres_sense_struct->depth = ms5837_Read_Depth();
    pres_sense_struct->average_depth = ms5837_Avg_Depth();
    pres_sense_struct->temperature = ms5837_Read_Temp();
    pres_sense_struct->average_temperature = ms5837_Avg_Temp();
  }
}
#endif
