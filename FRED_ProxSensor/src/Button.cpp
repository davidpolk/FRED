#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

/*
 * Initializes pull-up resistor on button pins and sets it into input mode
 */
void ButtonSetup() {
  // set direction for input
  DDRC &= ~(1 << DDC3); // Feed
  DDRC &= ~(1 << DDC5); // Prime feeding pump
  DDRC &= ~(1 << DDC4); // Move Forward
  DDRC &= ~(1 << DDC2); // Move Backward
  DDRC &= ~(1 << DDC1); // Feed Routine
  DDRC &= ~(1 << DDC0); // LED On/Off

  // enable the pullup resistor for stable input
  PORTC |= (1 << PORTC3);
  PORTC |= (1 << PORTC5);
  PORTC |= (1 << PORTC4);
  PORTC |= (1 << PORTC2);
  PORTC |= (1 << PORTC1);
  PORTC |= (1 << PORTC0);

  // enable the group pin change interrupts PCINTs 0 through 7
  PCICR |= (1 << PCIE0);

  // enable the local PCINT 4
  PCMSK0 |= (1 << PCINT4);

  Serial.println("Buttons initiated");
}
