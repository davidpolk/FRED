/////////////////////// PINOUT MAPING ////////////////////////////////
//
//   This document outlines all pins used on the Arduino Mega 2560 fork the FRED robot. 
//   The document is separated by the robot's subsystems. 
//   This document does not outline power or ground pins.
//   For further detail, see Wiring Schematics
//
//   Format:
//   Arduino Pin # ------ Endpoint component pin name - Description/Usage
//
//////////////////////////////////////////////////////////////////////




// ..........  Motor Drivers - L6474 (Motors) .......... //
//
// Both motor drivers use the same Direction, PWM, Reset, and Flag pins. 
//
// Pin 46 ------ CS1 - Chip select for Feeding plate motor
// Pin 47 ------ CS2 - Chip select for Movement Motor
// Pin 3  ------ Flag
// Pin 11 ------ PWM1 - Signal frequency (motor speed)
// Pin 7  ------ DIR1 - Direction in which motors move
// Pin 8  ------ Reset
// Pin 50 - MISO
// Pin 51 - MOSI
// Pin 52 - SCK



// .....................  LCD Screen .................. //
// Pin 40 ------ RS - Register Select 
// Pin 41 ------ EN - Enable
// Pin 24 ------ D4 - Data pin 4
// Pin 25 ------ D5 - Data pin 5
// Pin 26 ------ D6 - Data pin 6
// Pin 27 ------ D7 - Data pin 7
// Pin 10 ------ A  - Backlight 



// ...........  SD Card Module (Logger) ............... //
// Pin 50 ------ MISO - Master in slave out
// Pin 51 ------ MOSI - Master out slave in                     //////   GET RID OF SD CARD MODULE
// Pin 52 ------ SCK - Clock
// Pin 53 ------ SS/CS - Chip Select



// ......  XBEE (Wireless Serial Communication) ...... //
// Pin 19 ------ DOUT (RX1) - Data Sending 
// Pin 18 ------ DIN  (TX1) - Data Receiving 


  
// ................  Perilstatic Pumps .............. //
// Pin 4 ------ Q2 - Feeding pump activate
// Pin 5 ------ Q1 - Ethanol pump activate



// .....................  Buttons .................. //
// Pin 34 ------ Feeding Button 
// Pin 32 ------ Ethanol dispensing button
// Pin 36 ------ Feeding Routine Button 
// Pin 33 ------ Move Forward Button
// Pin 35 ------ Move Backward Button




// ................  Proximity Sensor ............. //
// Pin 20 ------ Proximity sensor OUT
// Pin 21 ------ Proximity sensor OUT 

// ................  LED ............. //
// Pin 53 ------ LED +3.3V