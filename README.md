# Wax on Wax off
# Ski Waxer build for Maiden Ski's

## Hardware
* 24V15A Power supply
* Arduino Uno
* 5 Button LCD Shield for Arduino
* 2x Switch's (End of track sensors)
* Nema 24 Stepper motor
* DM542T Stepper Driver
* Switching Wall outlet Powers strip (for heat lamp)
* Heat lamp
* Modular frame material
* Breadboard, LED's some wire and coffee
* Voltage Meter
* Drill Press, Gig saw, etc

## Manual Program Execution
1. Power on
2. Select # of passes using buttons
3. Select speed using buttons
4. Press select button
5. Initialization will move motor until it hit the start of track sensor
6. Heat lamp turns on and motor progresses to end of track sensor
7. Heat lamp is turned off and returns to start of track sensor
8. Repeat 6 & 7 until all passes are compelte
9. Return Heat Lamp to start of track sensor
10. Program loops to step 2

# Open Source Licensing GPL V2 license due to library(s) used.

## Libraries
* AccelStepper - Open Source Licensing GPL V2 - http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
* LiquidCrystal

## Thank you to
* Kelvin Wu and Maiden Ski's for letting me help with this project.
* Elli for putting me up, putting up with me and the feed.
* Mark Lehmkule for giving me the time off :D
* Youtube University
