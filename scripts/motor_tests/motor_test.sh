# @Zix
# Very quick motor test to spin all motors one-by-one for .6 seconds

cansend can0 010#0000000000000000
cansend can0 00A#

sleep .6
cansend can0 010#1000000000000000

sleep .6
cansend can0 010#0010000000000000

sleep .6
cansend can0 010#0000100000000000

sleep .6
cansend can0 010#0000001000000000

sleep .6
cansend can0 010#0000000010000000

sleep .6
cansend can0 010#0000000000100000

sleep .6
cansend can0 010#0000000000001000

sleep .6
cansend can0 010#0000000000000010

sleep .6
cansend can0 010#0000000000000000
cansend can0 00A#
