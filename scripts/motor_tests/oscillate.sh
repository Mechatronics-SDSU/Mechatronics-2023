# @Zix
# Working with Ken to try and find stall conditions for motors
# Script oscillates motor betwen positive 30 and -30 really fast
# DOESN"T seem to break motor for some weird reason

cansend can0 010#0000000000000000
cansend can 00A# 

# motor 1

# i=0

# while [ $i -lt 1000 ]; do
#     sleep .001 
#     cansend can0 010#0030000000000000
#     sleep .001 
#     cansend can0 010#00E2000000000000
#     i=`expr $i + 1`
# done


# motor 3

i=0

while [ $i -lt 1000 ]; do
    sleep .001 
    cansend can0 010#0000003000000000
    sleep .001 
    cansend can0 010#000000E200000000
    i=`expr $i + 1`
done

# i=0
# while [ $i -lt 100 ]; do
#     sleep .01
#     cansend can0 010#0000000010000000
#     sleep .01
#     cansend can0 010#00000000EC000000
#     i=`expr $i + 1`
# done

cansend can0 010#0000000000000000
cansend can 00A# 
