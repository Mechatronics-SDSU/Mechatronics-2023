#include <Servo.h>
#include <SimpleRotary.h>
SimpleRotary rotary(2,3,4);

//For screen
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)(1 disables blink)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int angle = 0;
int pos = 0;
//delay alternative setup
unsigned long previousMillis = 0UL;
unsigned long interval = 250UL;
Servo mainservo;
void setup() {
  Serial.begin(9600);
  mainservo.attach(9);

  //For Screen
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay(); //Clear display
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text, required

  
  
}

void loop() {
  // changes the servo position when rotary is rotated
    byte i; // 0 = not turning, 1 = CW, 2 = CCW
  i = rotary.rotate();
  if ( i == 1 ) {
    pos -= 5;
    Serial.println(pos);  //prints in terminal
    display.println("Clock-Wise"); //prints new line
  }
  if ( i == 2 ) {
    pos += 5;
    Serial.println(pos); //prints in terminal
    display.println(pos); //prints new line
  }
  // sets min and max positions 
  if (pos >= 180){
    pos = 180;
  }
  else if (pos <=0){
    pos = 0;
  }
  mainservo.write(pos);


  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval)
  {
  /* The Arduino executes this code once every second
  *  (interval = 1000 (ms) = 1 second).
  */
  display.clearDisplay(); //Clear display
  display.setCursor(0,0);  // Start at top-left corner
  display.println(F("Servo Controller: ")); //prints text on same line
  display.print(F("Angle: ")); //prints text on same line
  display.print(pos); //prints new line
  display.println(" Degrees"); //prints new line
  display.display(); //prints display
  // Don't forget to update the previousMillis value
  previousMillis = currentMillis;
  }
}
