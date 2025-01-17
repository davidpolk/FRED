#ifndef LOGGER_H
#define LOGGER_H
#include <Arduino.h>
#include <avr/io.h>

// Define Logger Functions
String getCurrentTime(float logTimerStartMs);
float convertTimeToMillis(float hour, float min, float sec, float millisec);
String formatTime(float hour, float min, float sec, float ms);
void SDCardSetup();
void writeToSDCard(String time, String message, String filename);

#endif