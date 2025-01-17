#include <Arduino.h>
#include <SPI.h>
#include <avr/io.h>
#include "SdFat.h"

File logFile;
SdFat SD;

// Format Time String
// Recieves time information ad floats and makes a printable time string
String formatTime(float hour, float min, float sec, float ms) {
double iMs = double(ms);
double iSeconds = double(sec);
double iMinute = double(min);
double iHour = double(hour);
char cMs[100];
char cSeconds[3];
char cMinute[3];
char cHour[3];
char csTimeStr[13];
String timeStr;
iHour = hour;
iMinute = min;
  
  //Convert milliseconds to seconds
  if (iMs >= 1000) {
    iSeconds = iSeconds + int(iMs / 1000);
    iMs = fmod(iMs, 1000);
  }

  //Convert seconds to minutes
  if (iSeconds >= 60) {
    iMinute = iMinute + int(iSeconds / 60);
    iSeconds = fmod(iSeconds, 60);
  }

  // Converts minutes to hours
  if (iMinute >= 60) {
    iHour = iHour + int(iMinute / 60);
    iMinute = fmod(iMinute, 60);
  }

  // Convert ints to string
  dtostrf(iMs, 3, 0, cMs);
  dtostrf(iSeconds, 1, 0, cSeconds);
  dtostrf(iMinute, 1, 0, cMinute);
  dtostrf(iHour, 1, 0, cHour);

  // Creates formatted time string HH:MM:SS:mS
  sprintf(csTimeStr, "%s:%s:%s:%s", cHour, cMinute, cSeconds, cMs);
  timeStr = String(csTimeStr);
  Serial.println(timeStr);

  return timeStr;
}



// getCurrentTime returns the currentTime in String
String getCurrentTime(float logTimerStartMs) {

  // Gets time elapsed since time sync
  float currentTimeMs = millis() - logTimerStartMs;

  Serial.println(currentTimeMs);
  String currentTimeStr = formatTime(0.0, 0.0, 0.0, currentTimeMs);

  return currentTimeStr;
}


// Setup for SD card reader on robot 
void SDCardSetup() {
  Serial.print("Initializing SD card...");
  SD.begin(53);

  if (!SD.begin(53)) {  //SD card module data port connected to Arduino pin 53
    Serial.println("initialization failed!");
    // while (1); 
  }
  Serial.println("initialization done.");
}


// Logs data into SD card 
void writeToSDCard(String time, String message, String filename) {
  
  
  logFile.close();
  logFile = SD.open(filename, FILE_WRITE); // Open file
  delay(10);

  if (logFile) {
    Serial.print("Writing to ");
    Serial.println(filename);
    logFile.print(time);
    logFile.print("    ");
    logFile.println(message);

    // close the file:
    logFile.close();
    Serial.println("done writing to file");
  }
  
  else {
    // if the file didn't open, print an error:
    // Serial.print("error opening ");
    // Serial.println(filename);

  }
}
