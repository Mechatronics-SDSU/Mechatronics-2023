# ENABLE SENSORS BASED ON ARGS ----------------------------------------------------------->
while getopts 'dvl' OPTION; do
  case "$OPTION" in
    d)
      cansend can0 022#0300000001
      cansend can0 020#03000000
      sleep .5
      echo "Enabling Depth Sensor"
      ;;
    v)
      cansend can0 022#0200000001
      cansend can0 020#02000000
      sleep .5
      echo "Enabling DVL Sensor"
      ;;
    l)
      cansend can0 022#0400000001
      cansend can0 020#04000000
      echo "Enabling Lamp"
      ;;
    ?)
      exit 1       
      ;;
  esac
done
