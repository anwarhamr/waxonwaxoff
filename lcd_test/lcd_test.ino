/*
  LiquidCrystal Library - Hello World
 
 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.
 
 This sketch prints "Hello World!" to the LCD
 and shows the time.
 
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 modified 7/11/2014
 by Jingjing Yi, LinkSprite
 
 This example code is in the public domain.
 
http://www.arduino.cc/en/Tutorial/LiquidCrystal
 
 */
#define btnLeft ((int) 1)
#define btnUp ((int) 2)
#define btnDown ((int) 3)
#define btnRight ((int) 4)
#define btnSelect ((int) 5)

 
// include the library code:
#include <LiquidCrystal.h>
 
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
 
 
int sensorValue ;
int speed; 
int passes;
int isProgramRunning;
int currentPassCount;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 200;    // the debounce time; increase if the output flickers

unsigned char GetKey(int value)
{
  Serial.print("keydown");
  Serial.println(value);
  if (value < 620){ return 5;}
  if (value < 822){ return 4;}
  if (value < 855){ return 1;}
  if (value < 920){ return 3;}
  if (value < 940){ return 2;}
  if (value < 1050){ return 0;}
 
}
 
void setup() 
{
  int tmpInt;
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Show me... ");
  lcd.setCursor(0,1);
  lcd.print("  wax on wax off!");
  sensorValue  = 1023;
   Serial.begin(9600); 
  passes = 2;
  speed  = 1;
  isProgramRunning = 0;
  currentPassCount = 0;

  
  delay(2000);
}

void handleButtons(){
  
  unsigned char key;
  if(sensorValue != analogRead(A0))
  {
     if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
   
       lastDebounceTime = millis();
       sensorValue = analogRead(A0);
      key = GetKey(sensorValue);
      if (key == btnLeft)
       {
        passes -= 1;
        if (passes < 1){ 
          passes = 1;
        }
       }
      if (key == btnRight)
        {
        passes += 1;
        
        }

      if (key == btnUp)
        {
        speed += 1;
        }
      if (key == btnDown)
        {
        speed -= 1;
        if (speed < 1){
          speed =1;
        }
       }
      if (key == btnSelect){
        isProgramRunning = 1;
      }
     }
  }
}
int lastDisplayPassCount;
void handleDisplay(){
  if (!isProgramRunning){
    lcd.setCursor(0, 0);
    lcd.print("Passes: ");      
    lcd.print(passes);
    lcd.print("        ");   
    lcd.setCursor(0, 1);
    lcd.print("Speed: ");      
    lcd.print(speed);
    lcd.print("        "); 
  }
  else{
    //todo: don't repaint so often and move code to another library.
    lcd.setCursor(0,0);
    lcd.print("Ps:");
    lcd.print(passes);
    lcd.print("  Spd:");
    lcd.print(speed);
    lcd.setCursor(0,1);
    lcd.print("Remaining:");
    lcd.print(passes - currentPassCount);
    
  }
}
void loop() 
{
   handleButtons();
   handleDisplay(); 
}
   

