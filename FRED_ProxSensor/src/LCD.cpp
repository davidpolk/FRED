#include <Arduino.h>
#include <avr/io.h>
#include <stdlib.h>
#include "lcd.h"
#include <LiquidCrystal.h>

// Arduino pins
const int rs = 40, en = 41, d4 = 24, d5 = 25, d6 = 26, d7 = 27;
const int backlightPin = 10;

//Define LCD object
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/*
 * Initializes all pins related to the LCD to be outputs
 */

void LCDSetup(){

    // set up the LCD's number of columns and rows:
    lcd.begin(20, 4);
    Serial.println("LCD initiated");
    pinMode(backlightPin, OUTPUT);
    LCDBacklightOn();
}

// Print a message to the LCD.
void writeLCD(const char* word, int line){
  lcd.setCursor(0, line);
  lcd.print(word);
  lcd.display();

}


//Clear LCD screen; deleta all text
void clearLCD(){
  lcd.clear();
}

void clearLine(int lineNum){
  lcd.setCursor(0, lineNum);
  lcd.print("                    ");
  lcd.display();
}

void LCDoff(){
  lcd.noDisplay();
}

void LCDon(){
  lcd.display();
}

void LCDBacklightOn() {
  digitalWrite(backlightPin, HIGH);
}

void LCDBacklightOff() {
  digitalWrite(backlightPin, LOW);
}

