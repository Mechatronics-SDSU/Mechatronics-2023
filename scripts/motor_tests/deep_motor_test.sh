# @Zix
# Deep motor test spins the motors in both directions with 4 speeds in both ways
# Please use this or I did twos complement conversions for nothing
# NOTE: This script will stall motors if the sleep is less than .1 seconds
# It is a guarantee to stall the motors at .1 seconds

$def="runs motors through backwards and forwards cycles to run a thorough test"

cansend can0 010#0000000000000000
cansend can0 00A#

# 0

sleep .1
cansend can0 010#1000000000000000
sleep .1
cansend can0 010#2000000000000000
sleep .1
cansend can0 010#3000000000000000
sleep .1
cansend can0 010#4000000000000000
sleep .1
cansend can0 010#3000000000000000
sleep .1
cansend can0 010#2000000000000000
sleep .1
cansend can0 010#1000000000000000
sleep .1
cansend can0 010#0000000000000000

cansend can0 010#F600000000000000
sleep .1
cansend can0 010#EC00000000000000
sleep .1
cansend can0 010#E200000000000000
sleep .1
cansend can0 010#C8000000000000000
sleep .1
cansend can0 010#E200000000000000
sleep .1
cansend can0 010#EC00000000000000
sleep .1
cansend can0 010#F600000000000000

# 1

sleep .1
cansend can0 010#0010000000000000
sleep .1
cansend can0 010#0020000000000000
sleep .1
cansend can0 010#0030000000000000
sleep .1
cansend can0 010#0040000000000000
sleep .1
cansend can0 010#0030000000000000
sleep .1
cansend can0 010#0020000000000000
sleep .1
cansend can0 010#0010000000000000
sleep .1
cansend can0 010#0000000000000000

cansend can0 010#00F6000000000000
sleep .1
cansend can0 010#00EC000000000000
sleep .1
cansend can0 010#00E2000000000000
sleep .1
cansend can0 010#00C80000000000000
sleep .1
cansend can0 010#00E2000000000000
sleep .1
cansend can0 010#00EC000000000000
sleep .1
cansend can0 010#00F6000000000000
sleep .1
cansend can0 010#0000000000000000

# 1

sleep .1
cansend can0 010#0000100000000000
sleep .1
cansend can0 010#0000200000000000
sleep .1
cansend can0 010#0000300000000000
sleep .1
cansend can0 010#0000400000000000
sleep .1
cansend can0 010#0000300000000000
sleep .1
cansend can0 010#0000200000000000
sleep .1
cansend can0 010#0000100000000000
sleep .1
cansend can0 010#0000000000000000

cansend can0 010#0000F60000000000
sleep .1
cansend can0 010#0000EC0000000000
sleep .1
cansend can0 010#0000E20000000000
sleep .1
cansend can0 010#0000C800000000000
sleep .1
cansend can0 010#0000E20000000000
sleep .1
cansend can0 010#0000EC0000000000
sleep .1
cansend can0 010#0000F60000000000
sleep .1
cansend can0 010#0000000000000000

# 3

sleep .1
cansend can0 010#0000001000000000
sleep .1
cansend can0 010#0000002000000000
sleep .1
cansend can0 010#0000003000000000
sleep .1
cansend can0 010#0000004000000000
sleep .1
cansend can0 010#0000003000000000
sleep .1
cansend can0 010#0000002000000000
sleep .1
cansend can0 010#0000001000000000
sleep .1
cansend can0 010#0000000000000000

cansend can0 010#000000F600000000
sleep .1
cansend can0 010#000000EC00000000
sleep .1
cansend can0 010#000000E200000000
sleep .1
cansend can0 010#000000C8000000000
sleep .1
cansend can0 010#000000E200000000
sleep .1
cansend can0 010#000000EC00000000
sleep .1
cansend can0 010#000000F600000000
sleep .1
cansend can0 010#0000000000000000

# 4

sleep .1
cansend can0 010#0000000010000000
sleep .1
cansend can0 010#0000000020000000
sleep .1
cansend can0 010#0000000030000000
sleep .1
cansend can0 010#0000000040000000
sleep .1
cansend can0 010#0000000030000000
sleep .1
cansend can0 010#0000000020000000
sleep .1
cansend can0 010#0000000010000000
sleep .1
cansend can0 010#0000000000000000

cansend can0 010#00000000F6000000
sleep .1
cansend can0 010#00000000EC000000
sleep .1
cansend can0 010#00000000E2000000
sleep .1
cansend can0 010#00000000C80000000
sleep .1
cansend can0 010#00000000E2000000
sleep .1
cansend can0 010#00000000EC000000
sleep .1
cansend can0 010#00000000F6000000
sleep .1
cansend can0 010#0000000000000000


sleep .1
cansend can0 010#0000000000100000
sleep .1
cansend can0 010#0000000000200000
sleep .1
cansend can0 010#0000000000300000
sleep .1
cansend can0 010#0000000000400000
sleep .1
cansend can0 010#0000000000300000
sleep .1
cansend can0 010#0000000000200000
sleep .1
cansend can0 010#0000000000100000
sleep .1
cansend can0 010#0000000000000000

cansend can0 010#0000000000F60000
sleep .1
cansend can0 010#0000000000EC0000
sleep .1
cansend can0 010#0000000000E20000
sleep .1
cansend can0 010#0000000000C800000
sleep .1
cansend can0 010#0000000000E20000
sleep .1
cansend can0 010#0000000000EC0000
sleep .1
cansend can0 010#0000000000F60000
sleep .1
cansend can0 010#0000000000000000


sleep .1
cansend can0 010#0000000000001000
sleep .1
cansend can0 010#0000000000002000
sleep .1
cansend can0 010#0000000000003000
sleep .1
cansend can0 010#0000000000004000
sleep .1
cansend can0 010#0000000000003000
sleep .1
cansend can0 010#0000000000002000
sleep .1
cansend can0 010#0000000000001000
sleep .1
cansend can0 010#0000000000000000

cansend can0 010#000000000000F600
sleep .1
cansend can0 010#000000000000EC00
sleep .1
cansend can0 010#000000000000E200
sleep .1
cansend can0 010#000000000000C800
sleep .1
cansend can0 010#000000000000E200
sleep .1
cansend can0 010#000000000000EC00
sleep .1
cansend can0 010#000000000000F600
sleep .1
cansend can0 010#0000000000000000

sleep .1
cansend can0 010#0000000000000010
sleep .1
cansend can0 010#0000000000000020
sleep .1
cansend can0 010#0000000000000030
sleep .1
cansend can0 010#0000000000000040
sleep .1
cansend can0 010#0000000000000030
sleep .1
cansend can0 010#0000000000000020
sleep .1
cansend can0 010#0000000000000010
sleep .1
cansend can0 010#0000000000000000

cansend can0 010#00000000000000F6
sleep .1
cansend can0 010#00000000000000EC
sleep .1
cansend can0 010#00000000000000E2
sleep .1
cansend can0 010#00000000000000C8
sleep .1
cansend can0 010#00000000000000E2
sleep .1
cansend can0 010#00000000000000EC
sleep .1
cansend can0 010#00000000000000F6
sleep .1
cansend can0 010#0000000000000000


cansend can0 010#0000000000000000
cansend can0 00A#