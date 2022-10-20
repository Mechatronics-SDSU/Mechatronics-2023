## Teensy 4.X ESC Drive Code
#### Completed in the standard Arduino Environment
This is test drive code to validate the Teensy 4.X platform as  
a viable way to drive ESC PWM control signals. Somewhat extensive  
testing was done with somewhat interesting findings.  
  
#### Updated version, library-ization (?), and macros are coming
  
  
### Findings of Note
> The rise time of both Flex and Quad timers is sometimes not very sharp  
> Accuracy of pulse width and frequency are very good at low speeds  
> Response times are unsurprisingly lightning fast  
  
### To Use
> Choose Which Timer to Test (Quad or Flex) by uncommenting one and commenting the other
> Upload to Teensy 4.X
> Observe output on scope
> Vary the speed through sending any byte (or no byte) through your favorite serial monitor
> That's it!
  
### Notes on Deadzone Considerations
A deadzone is almost required for any PWM based system to ensure that the 0% setting maintains  
a safety margin (ie. changes in signal or processing will not turn the motor on inside a  
tolerance band). Unfortunately this means with a direct map system (1% -> 0% PWM Val + 1%)  
your throttle turns on AT the deadzone, so with a +/- 3% deadzone our usable range is  
actually between -100% - -3% and 3% - 100%, meaning over the whole range we maintain  
limited control especially at lower throttle values. A control system may ramp through lower  
throttle values to correct some movement, but the first 3% (in this scenario) do nothing.  
Provided in the tile is a function that can easily map 0% - 100% and -100% - 0% to the  
actual range of control that we maintain (ie. RANGE - 2 * (DEADZONE)). This function intakes  
a floating point value (-100.0% - 100.0%) and will output an unsigned 16 bit number to be  
used as a compare match.
  
### Notes on Parametric Settings
Many defines are broken out and can be changed independently.
