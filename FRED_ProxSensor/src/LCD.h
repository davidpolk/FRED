#ifndef LCD_H
#define LCD_H4

#include <avr/io.h>

// Define LCD Functions
void LCDSetup();
void writeLCD(const char *word, int line);
void clearLCD();
void clearLine(int lineNum);
void LCDoff();
void LCDon();
void LCDBacklightOn();
void LCDBacklightOff();

#endif