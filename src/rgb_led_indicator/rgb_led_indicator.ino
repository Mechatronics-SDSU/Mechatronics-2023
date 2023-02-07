/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */

// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
int whiteled = 3;
int greenled = 5;
int redled = 6;
int blueled = 9;


// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(blueled, OUTPUT);
    pinMode(greenled, OUTPUT);
      pinMode(redled, OUTPUT);
        pinMode(whiteled, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
 // whiteRGB();
  whiteonly();
  delay(500);
  redonly();
  delay(500);
  blueonly();
  delay(500);
  greenonly();
  delay(500);
  
}

void whiteRGB(){
  analogWrite(redled, 100);
  analogWrite(blueled, 75);
  analogWrite(greenled, 255);
  analogWrite(whiteled, 0);
}

void whiteonly(){
  analogWrite(redled, 0);
  analogWrite(blueled, 0);
  analogWrite(greenled, 0);
  analogWrite(whiteled, 255);
}

void redonly(){
  analogWrite(redled, 255);
  analogWrite(blueled, 0);
  analogWrite(greenled, 0);
  analogWrite(whiteled, 0);
}
void blueonly(){
  analogWrite(redled, 0);
  analogWrite(blueled, 255);
  analogWrite(greenled, 0);
  analogWrite(whiteled, 0);
}
void greenonly(){
  analogWrite(redled, 0);
  analogWrite(blueled, 0);
  analogWrite(greenled, 255);
  analogWrite(whiteled, 0);
}
