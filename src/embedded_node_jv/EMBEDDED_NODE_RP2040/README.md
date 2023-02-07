# EMBEDDED NODE RP2040
The RP2040 fills the role of the slow speed subsystem of the  
core embedded systems. This entails driving slow speed motion,  
lighting, collecting long term data, and interfacing with  
leak detection.  
Utilization of both cores is crucial, where one core must  
run at full speed with no blocking outside of communication  
handling, and the other must handle slow speed motion.  

## Dependencies
Only a solid install of the `pico-sdk` is required.  
Please follow a guide recommended by a lead if there  
is any confusion.  
This repository will not provide you with the  
required `pico_sdk_import.cmake`, nor any built binaries.
  
  
## System Primary Tasks
- Smooth closure of actuators
- Torpedo firing
- Status Lighting
- Analog Data Collection
- Leak Detection

## Core 0
Running at full speed Core 0 will handle incoming FAST I2C (400kHz)  
communications with the T4 node.  
Additional parallel interrupt lines will link the 2 systems as well.  
Any bulk processing of data, DMA triggering, and more will be  
handled here as well.  
  
If PIO is used, all PIO interaction will be handled here.

## Core 1
Core 1 will have 2 modes:
- Active
- Inactive
In the Active state the core will run at a slow speed such that many  
human timescale changes (close a servo, fade an led, etc) can be done in  
the same loop.
That is, if 2 things must change at once, their values can be updated at  
effectively the same interval, with equal spacing between intervals.  
This can offer benefits when parallelizing many slow movements.  
Use of the hardware interpolation may be used to map all changes to the  
same timebase (~10 us per interval, TBD).
  
Inactive mode will effectively freewheel or sleep, waiting for instructions  
to change any/many output(s).