  ///////////////////////////////////////////////////////////////////////////////
 //                   Data Conversion Functions                               //
///////////////////////////////////////////////////////////////////////////////

// Convert floats to char array, LSB first (little endian!)
void float_2_char_array(uint8_t *arr_out, float *d_in){
  arr_out[0] = *((uint8_t *)(d_in));
  arr_out[1] = *((uint8_t *)(d_in) + 1u);
  arr_out[2] = *((uint8_t *)(d_in) + 2u);
  arr_out[3] = *((uint8_t *)(d_in) + 3u); 
}


  ///////////////////////////////////////////////////////////////////////////////
 //                   MS5837 Data, Ready to Send                              //
///////////////////////////////////////////////////////////////////////////////
// MS5837 Data Struct for general use
#ifdef ENABLE_PRES_SENS

// All float types converted to char arrays for transmission
//  these transmit LSB first!!
typedef struct pressure_sensor_t{
  uint8_t depth[4];
  uint8_t average_depth[4];
  uint8_t temperature[4];
  uint8_t average_temperature[4];
  char  model[7];
  uint8_t healthy;
};

#endif


  ///////////////////////////////////////////////////////////////////////////////
 //                   Wayfinder DVL Data, Ready to Send                       //
///////////////////////////////////////////////////////////////////////////////
// Wayfinder DVL Data Struct for general use
#ifdef ENABLE_DVL




#endif
