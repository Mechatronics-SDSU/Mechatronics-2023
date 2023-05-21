# @Zix
# Convenience script so people don't have to memorize CAN commands
# Just type the script and the number of motor you want to run

$def="runs selected motor by use of command line argument (pass in 1 to activate motor 1)"

cansend can0 010#0000000000000000
cansend can0 00A#

if [ $1 -eq 0 ]
then
    cansend can0 010#1000000000000000
fi

if [ $1 -eq 1 ]
then
    cansend can0 010#0010000000000000
fi

if [ $1 -eq 2 ]
then
    cansend can0 010#0000100000000000
fi

if [ $1 -eq 3 ]
then
    cansend can0 010#0000001000000000
fi

if [ $1 -eq 4 ]
then
    cansend can0 010#0000000010000000
fi

if [ $1 -eq 5 ]
then
    cansend can0 010#0000000000100000
fi

if [ $1 -eq 6 ]
then
    cansend can0 010#0000000000001000
fi

if [ $1 -eq 7 ]
then
    cansend can0 010#0000000000000010
fi

if [ $1 -eq 8 ]
then
    cansend can0 010#1010101010101010
fi


sleep 1

cansend can0 010#0000000000000000
cansend can0 00A#
