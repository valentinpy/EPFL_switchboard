#include "Arduino.h"
#include "include/tLed.h"

void TLed::setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  lastRun = millis();
}

void TLed::run(){
  if (millis()-lastRun > PERIOD_MS){
    state = ! state;
    lastRun = millis();
    digitalWrite(HV_LED_PIN, state);
  }
}
