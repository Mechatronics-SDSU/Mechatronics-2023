# EMBEDDED_NODE
This is the home of the embedded node software. 
Two components make up the current Embedded Node:
- Teensy 4.1
- RP2040
  
Balancing availablity, processing power, peripheral availability, and usability for future members  
the Teensy 4.1 was selected as the main processing hub, with primary development done with the  
Teensyduino add-on with Arduino IDE version `1.8.19`.  
  
The RP2040 was selected to handle slower speed tasks that may require `busy wait` or blocking delay conditions.  
Primary development is done through the standard RP2040 C SDK workflow.
  
  
## Requirements for Additions
All additions and modifications should be shown and approved by the current Embedded lead.  
This is the heart of the realtime processing within the sub and as such must have changes reviewed,  
and guaranteed to follow the core design requirements of the embedded subsystem.
  
  
## T4 Dependencies
- `FlexCAN_T4` Provides asynchronous access to the iMX.RT 1062 CAN Peripheral
- [T4_WAYFINDER_DVL](https://github.com/4n3m4i1/T4_WAYFINDER_DVL) Provides asynchronous handling and bindings to read from and write to the Wayfinder DVL
- [T4_MS5837](https://github.com/4n3m4i1/T4_MS5837) Provides asynchronous bindings and optimizations to the Blue Robotics MS5837 library
- `IntervalTimer` Provides ISR triggering interval timer bindings
- [WDT_T4](https://github.com/tonton81/WDT_T4) Provides Watchdog Timer interfaces
- [teensy4_i2c](https://github.com/Richard-Gemmell/teensy4_i2c) Teensy i2c library, Richard Gemmel
- [T4_WL_M64_MODEM](https://github.com/4n3m4i1/T4_WL_M64_Modem) Driver for Waterlinked Acoustic Modems

## T4 Design Requirements
- No polling or blocking. All processes should leverage the asynchronous nature of peripherals and priority driven interrupts
- Minimal reliance on unknown code. If you don't know what's in a library, it shouldn't be in here. (FlexCAN gets a slight pass here)  
- Optimized response for higher priority CAN frames, or "Reflex Responses" (ex. Shutdown 000# and Motor Drive 010#)
- Timer + ISR driven events if periodic queries are requested. (Interval Timer)
- Slow Speed tasks should be avoided entirely, instead these should be offloaded to subsystems
  
## T4 Pinout
The T4 Pinout described in the attached PNG (T4_Pinout.png) is representative  
of the current on-board pinout that is being used at any given time.  
Many pin assignments are subject to change over time and future plans for  
boards should only be done after consulting Joseph to ensure the most  
current pinout is posted.

- Note: 07/21/2023 This image is super outdated, do not cite.
  
## RP2040 Dependencies
- RP2040 C SDK
  
## RP2040 Design Requirements
- Utilize both cores, one for mostly non-blocking and one for blocking tasks
- Implement all low/slow speed tasks on chip if possible
- Drive servos, torpedo launch systems
- Actively sense power system datapoints if power system does not contain standalone processing
- Drive non-critical human interfaces (lights, indicators).
