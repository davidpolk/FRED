#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Arduino pins connected to the pump
int feedPumpSignal = 4; 
int cleanPumpSignal = 5; 


// Declaration of the Pump setup
void PumpSetup(){
  
  pinMode(feedPumpSignal, OUTPUT);
  pinMode(cleanPumpSignal, OUTPUT);
  digitalWrite(feedPumpSignal, LOW);
  digitalWrite(cleanPumpSignal, LOW);
  Serial.println("Feeder Pump initiated");

}



// Handles liquid pumping for feeding and ethanol
void pumpFood(int delayTimeMs){

    // QueuePacket(&r2a, 'r', delayTimeMs, cnt_rew);
    // SendPacket(&r2a);

    digitalWrite(feedPumpSignal, HIGH);
    delay(delayTimeMs);
    digitalWrite(feedPumpSignal, LOW);


}


// Handles liquid pumping for feeding and ethanol
void pumpEtoh(){

    digitalWrite(cleanPumpSignal, HIGH);
    delay(2000);
    digitalWrite(cleanPumpSignal, LOW);


}