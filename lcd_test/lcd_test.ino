/*
  LiquidCrystal Library - Hello World
 

 This example code is in the public domain.
 
http://www.arduino.cc/en/Tutorial/LiquidCrystal
 
 */
 // include the library code:
#include <LiquidCrystal.h>
#include <AccelStepper.h>

#define btnLeft ((int) 1)
#define btnUp ((int) 2)
#define btnDown ((int) 3)
#define btnRight ((int) 4)
#define btnSelect ((int) 5)
#define startOfTrackSensor ((int)2)
#define endOfTrackSensor ((int)3)
#define heater ((int)11)
//#define enablePin ((int)??)
#define speedMultiplier ((int)100000)

#define Stopped ((int) 0)
#define Starting ((int) 2)
#define Initializing ((int) 3)
#define Initialized ((int) 4)
#define Running ((int) 5)
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// initialize the stepper library on pins
AccelStepper stepper(AccelStepper::FULL2WIRE,12,13);
int pos = 1;

int sensorValue ;
int speed; 
int passes;
int currentPassCount;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 200;    // the debounce time; increase if the output flickers
int motorDirection = 1;
int startOfTrackSensorValue = LOW;
int endOfTrackSensorValue = LOW;
int trackSensorOn = LOW;
int programStatus = 0; //0 stopped, 1 = initializing, 2 = running

unsigned char GetKey(int value)
{
  Serial.print("keydown");
  Serial.println(value);
  //todo: check on these values we should only have 5 buttons not 6...
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
  pinMode(startOfTrackSensor, INPUT);
  pinMode(endOfTrackSensor, INPUT);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.print("Show me... ");
  lcd.setCursor(0,1);
  lcd.print("  wax on, wax off!");
  
  //init values
  sensorValue  = 1023;
  passes = 2;
  speed  = 1;
  currentPassCount = 0;
  
  //stepper.setEnablePin(enablePin);
  
  digitalWrite(heater,LOW);
  Serial.begin(9600); 
  delay(2000);
}

void loop() 
{
  handleEndOfTrackSensors();
  if (programStatus == Starting || programStatus == Initializing){
    initialize();
  }
  else{
    handleButtons();
    handleDisplay(); 
    //displayDebug();
  }
  if (programStatus == Initialized){
    Serial.println("setting run params");
    digitalWrite(heater,HIGH);
    stepper.setAcceleration(100000);
    stepper.setMaxSpeed(speed* speedMultiplier);
    programStatus = Running;
  }
  if (programStatus == Running){
    //todo check if we've written latest run values to eeprom and load in startup.
    if (currentPassCount <= passes){
      Serial.print("running ");
      Serial.print(motorDirection);
      Serial.print(" currentPass:");
      Serial.println(currentPassCount);
      stepper.move(pos * motorDirection);
      stepper.run();
      if (motorDirection == -1){
        lcd.setCursor(0,1);
        lcd.print("Returning 4 nxt");
      }
    }
    else{
      handleEndOfProgram();
    }
  }
}

void initialize(){
  //turn off heater
  if (programStatus == Starting){
    Serial.println("setting values for initialization");
    digitalWrite(heater,LOW);
    stepper.setAcceleration(500000);
    stepper.setMaxSpeed(1000000);
    programStatus = Initializing;
  }
  
  if (!startOfTrackSensorValue){
    Serial.println("init");
    lcd.setCursor(0,1);
    lcd.print("Initializing...  ");
    stepper.move(pos * -1);
    stepper.run();
  }
  else{
    stepper.stop();
    Serial.println("done init");
    programStatus = Initialized;
  }
}

void displayDebug(){
  lcd.setCursor(0,0);
  lcd.print("s");
  lcd.print(speed);
  lcd.print("p");
  lcd.print(passes);
  lcd.print("t");
  lcd.print(trackSensorOn);
  lcd.print("st:");
  lcd.print(startOfTrackSensorValue);
  lcd.print("en:");
  lcd.print(endOfTrackSensorValue);
  lcd.print(" ");
  
  lcd.setCursor(0,1);
  lcd.print("m");
  lcd.print(stepper.isRunning());
  lcd.print(":");
  lcd.print(stepper.currentPosition());
  lcd.print("@");
  lcd.print(stepper.speed());
}

int handleButtons(){
  unsigned char key;
  if(sensorValue != analogRead(A0))
  {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      
      lastDebounceTime = millis();
      sensorValue = analogRead(A0);
      key = GetKey(sensorValue);
      if (key == btnLeft){
        passes -= 1;
        if (passes < 1){ 
          passes = 1;
        }
      }
      if (key == btnRight){
        passes += 1;
      }
      if (key == btnUp){
        speed += 1;
      }
      if (key == btnDown){
        speed -= 1;
        if (speed < 1){
          speed =1;
        }
      }
      if (key == btnSelect){
        startProgram();
      }
    }
  }
}
void startProgram(){
  if (programStatus == Stopped){
    programStatus = Starting;
    stepper.enableOutputs();
    Serial.println("starting...");
  }
}

void handleEndOfProgram(){
    Serial.println("ending...");
    programStatus = 0;
    currentPassCount = 0;
    stepper.disableOutputs();
}

void handleDisplay(){
  if (programStatus == 0){
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
    if (startOfTrackSensorValue) {
      lcd.print("**** Start of Track ****");
      return;
    }
    if (endOfTrackSensorValue) {
      lcd.print("**** End of Track ****");
      return;
    }
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Ps: ");
    lcd.print(passes);
    lcd.print("  Spd: ");
    lcd.print(speed);
    lcd.setCursor(0,1);
    lcd.print("pos ");
    
    lcd.print("Remaining: ");
    lcd.print(passes - currentPassCount);
    
  }
}

void handleEndOfTrackSensors(){
  startOfTrackSensorValue = digitalRead(startOfTrackSensor);  
  endOfTrackSensorValue = digitalRead(endOfTrackSensor);  
  
  if (endOfTrackSensorValue || startOfTrackSensorValue){
    Serial.print(trackSensorOn);
    Serial.print(" ");
    Serial.print(startOfTrackSensorValue);
    Serial.print(" ");
    Serial.print(endOfTrackSensorValue);
    Serial.println(" ");
    
    if (startOfTrackSensorValue){
      motorDirection = 1;
      Serial.println("start of track");
    }
    if (endOfTrackSensorValue){
      motorDirection = -1;
      if (!trackSensorOn && programStatus == Running){
        currentPassCount +=1;
      }
      Serial.println("end of track");
    }
  }
  trackSensorOn = endOfTrackSensorValue || startOfTrackSensorValue;
}

   

