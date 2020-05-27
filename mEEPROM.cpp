#include "Arduino.h"
#include "include/mEEPROM.h"
#include <EEPROM.h> //Some parameters stored in EEPROM, such as slave I2C address and board maximal voltage. PID paramters could be stored there as well

void MEEPROM::setup(){
  return;
}

void MEEPROM::read_string(word aAddr, byte an, char * c) //reads a string from EEPROM at address addr, and with a length of n or less (n includes the termination character \0) and store it in C
{
  if(EEPROM.read(aAddr)==255){  //If string has not yet been defined in EEPROM
    c[0]='\0';
  }
  else {
    for (byte i=0; i<an; i++) {
      c[i]=EEPROM.read(aAddr+i);
      if (c[i]=='\0'){
        break;
      }
    }
  }
}

void MEEPROM::update_string(word aAddr, byte an, const char * c) //writes string c at address addr, but not more than n bytes
{
  for (byte i=0; i<an; i++) {
    EEPROM.update(aAddr+i,c[i]);
    if(c[i]=='\0'){
      break;
    }
  }
}
