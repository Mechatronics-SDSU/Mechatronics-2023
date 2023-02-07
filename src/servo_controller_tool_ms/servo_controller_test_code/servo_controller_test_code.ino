#include <Wire.h>

//For Encoder
#include <SimpleRotary.h>
// Pin A, Pin B, Button Pin
SimpleRotary rotary(2,3,4);
int angle = 0;

//For screen
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)(1 disables blink)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {

  Serial.begin(9600);

  //For Screen
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay(); //Clear display
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text, required
}


void loop() {
  screen();
  encoder(); //may requires library interrupts, display may have delays?
  display.display(); //update display, delays the encoder for some reason, remove to speed it up.
}

void screen(){
// Stuff for Display
  display.clearDisplay(); //Clear display
  display.setCursor(0,0);  // Start at top-left corner
  display.println(F("Servo Controller: ")); //prints text on same line
  display.print(F("Angle: ")); //prints text on same line
  display.print(angle); //prints new line
  display.println(" Degrees"); //prints new line
//  display.display(); //prints display
}

void encoder(){
    byte i; // 0 = not turning, 1 = CW, 2 = CCW
  i = rotary.rotate();
    if ( i == 1 ) {
    Serial.println("Clock-Wise");  //prints in terminal
    display.println("Clock-Wise"); //prints new line
  }
  if ( i == 2 ) {
    Serial.println("Counter Clock-Wise"); //prints in terminal
    display.println("Counter Clock-Wise"); //prints new line
  }
}
