# CAN Baseline Description
Communication between the COMP (Computer, AGX Orin for 2023) and MCU (Embedded subsystem, Teensy 4.1 for 2023)  
is done through a standard automotive CAN 2.0A protocol at 500k BAUD.  
A fully custom and comprehensive protocol has been described in the following document:  
- https://docs.google.com/spreadsheets/d/1Z7OvEd_o8Brstv6O6mNKoWZZ86YrVBjY30yL9ZncIfY/edit?usp=sharing
  
To request edit permissions please contact the current document owner.
  
CAN 2.0A frames for every message that is valid for the `EMBEDDED_NODE_T4` configuration a user has chosen  
are described in the document. Messages sent outside of the documented scope will be ignored.  
With this in mind, undefined behavior is possible if a valid message ID is sent with data outside of the  
defined valid range/format.
  
 ## Notation
 Common notation may appear as:
 -`010#FF01`
 This notation is composed of 2 main parts:
 - `010#`  Represents the 9 bit CAN ID. Can easily be identified as 3 hex values followed by a `#`
 - `FF01`  Represents the sent data, in this case 2 bytes. Representation is always in hex if a value is given
 
 Always include the `#` after an ID to clearly denote the value is a CAN ID.
 
 ### Decoding Given CAN Frames
Always first check the ID of your message, then work from there citing the sheet.  
To decode a message and debug the contents, please issue the same data payload but with the  
readback ID (`1F0#`).
  
# Using the CAN Bus
Simple transmits may be performed with the command line utility:
- `cansend {selected can bus, eg. can0) {valid data frame}`
  
Simple reads may be performed with:
- `candump {selected can bus, eg. can0)`
  
Please ensure all hardware connections are made correctly.  
Standard transciever part number: `SN65HVD230`
  
Following the aforementioned documentation will offer guaranteed results. Common test procedures can be done by  
- Ask for a readback: `cansend {selected can bus, eg. can0} 1F0#ABCD` should return `1F0#ABCD`
- Ask for a blinky response: Same as above but with `1F1#`
