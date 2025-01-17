#include <Arduino.h> 
#include <avr/io.h>
#include <stdio.h>

// #define SERIAL_BUFFER_SIZE 128


// GET/SET BYTE BIT VALUE
bool GetSetByteBit(byte * b_set, int bit, bool do_set){
  
  // Set bit
  if (do_set)
  {
    *b_set = *b_set | 0x01 << bit;
  }

  // Return state
  return ((*b_set >> bit) & 0x01) == 1;
}






