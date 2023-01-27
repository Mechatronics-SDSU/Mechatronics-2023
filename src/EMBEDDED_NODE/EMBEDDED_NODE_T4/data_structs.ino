

// MS5837 Data Struct for general use
#ifdef ENABLE_PRES_SENS

typedef struct pressure_sensor_t{
  float depth;
  float average_depth;
  float temperature;
  float average_temperature;
  char  model[7];
  uint8_t healthy;
};

#endif


// Wayfinder DVL Data Struct for general use
#ifdef ENABLE_DVL




#endif
