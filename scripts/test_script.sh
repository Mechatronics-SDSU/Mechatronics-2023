gnome-terminal --geometry 80x25+0+0 -- bash -c "candump can0; exec bash"

sleep .5

# ENABLE SENSORS BASED ON ARGS ----------------------------------------------------------->
while getopts 'dvl:' OPTION; do
  case "$OPTION" in
    d)
      cansend can0 022#0300000001
      cansend can0 020#03000000
      sleep .5
      ;;
    v)
      cansend can0 022#0200000001
      cansend can0 020#02000000
      sleep .5
      ;;
    l)
      cansend can0 022#0400000001
      cansend can0 020#04000000
      ;;
    ?)
      exit 1       
      ;;
  esac
done
shift "$(($OPTIND -1))"
