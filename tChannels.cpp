#include "Arduino.h"
#include "include/tChannels.h"

void TChannels::setup(){
  for(int i=0; i<NBREL; i++){
    pinMode(Rel_pin[i], OUTPUT);
    digitalWrite(Rel_pin[i], LOW);
    Rel_status[i] = false;
  }
}

void TChannels::run(){

}

void TChannels::allOn(){
  for(int i=0; i<NBREL; i++){
    digitalWrite(Rel_pin[i], HIGH);
    Rel_status[i] = true;
  }
  currentMode = AllOn;
}

void TChannels::allOff(){
  for(int i=0; i<NBREL; i++){
    digitalWrite(Rel_pin[i], LOW);
    Rel_status[i] = false;
  }
  currentMode = AllOff;
}

bool TChannels::set1(unsigned int channel, bool state){
  if(channel < NBREL){
    digitalWrite(Rel_pin[channel], state);
    Rel_status[channel] = state;
    return true;
  }
  else{
    // Channel out of bound
    return false;
  }

}

void TChannels::autoMode(int aAutoRestart, bool* aListChannelsUsed){
  //TODO implement
    currentMode = AutoMode;
}

void TChannels::getChannelsStatus(bool* retVal){
    for(int i=0; i<NBREL; i++){
        retVal[i] = Rel_status[i];
    }
}

void TChannels::printChannelsStatus() {
    Serial.print(currentMode);
    Serial.print(";");
    for (int i = 0; i < NBREL; i++) {
        Serial.print(Rel_status[i]);
        Serial.print(",");
    }
    Serial.println("");

}