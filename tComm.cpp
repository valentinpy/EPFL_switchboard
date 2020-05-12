#include "Arduino.h"
#include "include/tComm.h"

void TComm::setup(){
  Serial.begin(115200);
  Serial.println("Switchboard V2");
}

void TComm::run(){
  return;
  // Serial.print(".");
  // delay(100);
}
