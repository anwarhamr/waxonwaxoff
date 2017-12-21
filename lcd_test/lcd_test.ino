/*
  LiquidCrystal Library - Hello World
 

 This example code is in the public domain.
 
http://www.arduino.cc/en/Tutorial/LiquidCrystal
 
 */
 // include the library code:
#include <LiquidCrystal.h>
#include <AccelStepper.h>
#include "eepromHelper.h"

#define btnLeft ((int) 1)
#define btnUp ((int) 2)
#define btnDown ((int) 3)
#define btnRight ((int) 4)
#define btnSelect ((int) 5)
#define startOfTrackSensor ((int)2)
#define endOfTrackSensor ((int)3)
#define heater ((int)11)
#define motor1 ((int)12)
#define motor2 ((int)13)
//#define enablePin ((int)??)

/*
 * change these values for run configuration
 * //todo add these value to a configration page.
 */
#define speedMultiplier ((unsigned long)50000)
#define speedMax       ((unsigned long)4000000)
#define speedReturn ((unsigned long)3000000)
#define heaterWarmupPeriod ((int)3)

#define Stopped ((int) 0)
#define Starting ((int) 2)
#define Initializing ((int) 3)
#define Initialized ((int) 4)
#define Warmup ((int) 5)
#define Running ((int) 10)

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// initialize the stepper library on pins
AccelStepper stepper(AccelStepper::FULL2WIRE,12,13);
int pos = 1;

int sensorValue ;
int currentPassCount;
int returning =0;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 200;    // the debounce time; increase if the output flickers
int motorDirection = 1;
int startOfTrackSensorValue = LOW;
int endOfTrackSensorValue = LOW;
int trackSensorOn = LOW;
int programStatus = 0; //0 stopped, 1 = initializing, 2 = running
int heaterStatus = LOW;
unsigned long warmupStartTime = 0;

struct config_t
{
    int isConfigured; // our first write we'll have this set to 999
    long passes;
    int speed;
} configuration;

unsigned char GetKey(int value)
{
  //Serial.print("keydown");
  //Serial.println(value);
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
  pinMode(heater,OUTPUT);
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.print("Show me... ");
  lcd.setCursor(0,1);
  lcd.print(" wax on, wax off!");
  lcd.noAutoscroll();
  
  //init values
  sensorValue  = 1023;
  EEPROM_readAnything(0, configuration);
  Serial.print("config values");
  Serial.print(configuration.passes);
  Serial.println(configuration.speed);
  
  if (configuration.isConfigured != 9999){
    configuration.isConfigured = 9999;
    configuration.passes = 2;
    configuration.speed  = 1;
  }
  currentPassCount = 1;
  
  //stepper.setEnablePin(enablePin);
  
  updateHeater(LOW);
  Serial.begin(9600); 
  delay(2000);
}

void loop() 
{
  handleEndOfTrackSensors();
  if (programStatus == Stopped){
    handleDisplay(); 
  }
  else if (millis()% 600 == 1 && programStatus != Running){
    handleDisplay(); 
    //runtime update will happen only after each track sensor is touched.
  }
  //if (programStatus == Stopped){
    handleButtons();
  //}
  if (programStatus == Starting || programStatus == Initializing){
    initialize();
  }
  if (programStatus == Initialized){
    Serial.println("setting run params");
    EEPROM_writeAnything(0, configuration);
    programStatus = Warmup;
    warmupStartTime = millis();
  }
  if (programStatus == Warmup){
    if (heaterStatus == LOW){
      updateHeater(HIGH);
    }
    if (millis() - warmupStartTime > heaterWarmupPeriod * 1000){
      programStatus = Running;
      Serial.print("Running MotorDir:");
      Serial.println(motorDirection);
    }
  }
  if (programStatus == Running){
    //todo check if we've written latest run values to eeprom and load in startup.
    if (currentPassCount <= configuration.passes || motorDirection == -1){
      if (motorDirection == -1){
        if (heaterStatus != LOW){
          updateHeater(LOW);
        }
        if (!returning){
          Serial.println("returning setting accel");
          stepper.setAcceleration(speedReturn);
          returning = 1;
        }
        stepper.move(-pos);
      }
      else{
        if (heaterStatus != HIGH){
          updateHeater(HIGH);
        }
        if (returning){
          Serial.print("running setting accel ");
          Serial.println(configuration.speed * speedMultiplier);
          stepper.setAcceleration(configuration.speed * speedMultiplier);
          returning = 0;
        }
        stepper.move(pos);
      }
//      Serial.print("running ");
//      Serial.print(motorDirection);
//      Serial.print(" currentPass:");
//      Serial.println(currentPassCount);
      stepper.run();
      
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
    updateHeater(LOW);
    stepper.setAcceleration(speedReturn);
    motorDirection = -1;
    programStatus = Initializing;
    
  }
  
  if (!startOfTrackSensorValue){
    //Serial.println("init");
    stepper.move(pos * -1);
    stepper.run();
  }
  else{
    stepper.stop();
    motorDirection = 1;
    returning = 1;
    Serial.println("done init");
    programStatus = Initialized;
  }
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
        configuration.passes -= 1;
        if (configuration.passes < 1){ 
          configuration.passes = 1;
        }
      }
      if (key == btnRight){
        configuration.passes += 1;
      }
      if (key == btnUp){
        Serial.print(configuration.speed * speedMultiplier);
        Serial.println(speedMax);
        if (configuration.speed * speedMultiplier < speedMax){
          configuration.speed += 1;
        }
      }
      if (key == btnDown){
        configuration.speed -= 1;
        if (configuration.speed < 1){
          configuration.speed =1;
        }
      }
      if (key == btnSelect){
        if (programStatus == Stopped){
        startProgram();
        }
        else{
          handleEndOfProgram();
        }
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
    programStatus = Stopped;
    currentPassCount = 1;
    updateHeater(LOW);
    stepper.disableOutputs();
}

void updateHeater(int value){
  heaterStatus = value;
  digitalWrite(heater,heaterStatus);
}

void handleDisplay(){
  if (programStatus == Stopped){
    lcd.setCursor(0, 0);
    lcd.print("Passes: ");      
    lcd.print(configuration.passes);
    lcd.print("        ");   
    lcd.setCursor(0, 1);
    lcd.print("Speed: ");      
    lcd.print(configuration.speed);
    lcd.print("        "); 
  }
  else if (programStatus == Warmup){
      lcd.setCursor(0, 1);
      lcd.print("Warming... ");
      lcd.print((int) (heaterWarmupPeriod) - (millis() - warmupStartTime)/1000 );
      lcd.print("       ");
  }
  else{
    lcd.setCursor(0,0);
    lcd.print("Ps: ");
    lcd.print(configuration.passes);
    lcd.print("  Spd: ");
    lcd.print(configuration.speed);
    lcd.setCursor(0,1);
    if (motorDirection == 1){
      if (configuration.passes - currentPassCount > 0){
        lcd.print("Remaining ");
        lcd.print(configuration.passes - currentPassCount);
        lcd.print("       ");
      }
      else{
        lcd.print("Last pass, finally right?   ");
      }
    }
    else{
        if (programStatus == Running){
          lcd.print("beep beep beep       ");
        }
        else{
          lcd.print("Chill Winston...      ");
        }
    }
  }
}

void handleEndOfTrackSensors(){
  startOfTrackSensorValue = digitalRead(startOfTrackSensor);  
  endOfTrackSensorValue = digitalRead(endOfTrackSensor);  
  
  if (endOfTrackSensorValue || startOfTrackSensorValue){
//    Serial.print(trackSensorOn);
//    Serial.print(" ");
//    Serial.print(startOfTrackSensorValue);
//    Serial.print(" ");
//    Serial.print(endOfTrackSensorValue);
//    Serial.println(" ");
    
    
    if (startOfTrackSensorValue){
      if (motorDirection != 1){
        stepper.stop();
        Serial.print("Start of track Pos before setting to zero ");
        Serial.print(stepper.currentPosition());
        Serial.print(" Target Pos:");
        Serial.println(stepper.targetPosition());
        stepper.setCurrentPosition(0);
      }
      motorDirection = 1;
      //Serial.println("start of track");
    }
    if (endOfTrackSensorValue){
      
      if (motorDirection != -1){
        stepper.stop();
        Serial.print("End of track Pos:");
        Serial.print(stepper.currentPosition());
        Serial.print(" Target Pos:");
        Serial.println(stepper.targetPosition());
      }
      motorDirection = -1;
      if (!trackSensorOn && programStatus == Running){
        currentPassCount +=1;
      }
//      Serial.println("end of track");
    }
    
    handleDisplay(); 
  }
  trackSensorOn = endOfTrackSensorValue || startOfTrackSensorValue;
}


void displayDebug(){
  lcd.setCursor(0,0);
  lcd.print("s");
  lcd.print(configuration.speed);
  lcd.print("p");
  lcd.print(configuration.passes);
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

