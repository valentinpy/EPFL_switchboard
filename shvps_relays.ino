/*Code for the switchboard boards of the Project derivated from Peta-pico-Voltron
Author: Valentin Py
E-mail: valentin.py@epfl.ch / valentin.py@gmail.com
Company: EPFL - LMTS
Date: 17.12.2019

Target platform: Arduino micro + switchboard v1.0

Copyright 2019 Valentin Py
Distributed under the terms of the GNU General Public License GNU GPLv3

This file is part of Switchboard.

shvps is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

shvps is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with shvps.  If not, see  <http://www.gnu.org/licenses/>

*/

void Rel_init(){
  // Initializes relays pin: output, off
  RelMode = RelMode_manual;
  for(int i=0; i< RelNbRel; i++){
    RelState[i] = false;
    pinMode(RelCmd_pin[i], OUTPUT);
    digitalWrite(RelCmd_pin[i], LOW);
  }
}

void Rel_allOff(){
  // all relays => off
  for(int i=0; i< RelNbRel; i++){
    RelState[i] = false;
    digitalWrite(RelCmd_pin[i], LOW);
  }
}

void Rel_allOn(){
  // all relays => on
  for(int i=0; i< RelNbRel; i++){
    RelState[i] = true;
    digitalWrite(RelCmd_pin[i], HIGH);
  }
}

void Rel_setRel(byte relay, bool status){
  // set / reset a relay.
  // relay: index of relay to set/reset: [0;5]
  // status: true -> set (ON), false -> reset (OFF)
  if((relay >=0) && (relay < RelNbRel)){
    RelState[relay] = status;
    digitalWrite(RelCmd_pin[relay], status);
  }
  else{
    Serial.print("Bad relay index: ");
    Serial.println(relay);
  }
}

void Rel_apply(){
  // apply RelState values to all relays
  for(int i=0; i< RelNbRel; i++){
    digitalWrite(RelCmd_pin[i], RelState[i]);
  }
}

void Rel_printState(){
  // Print rekays state
  Serial.print("Relays state: ");
  for(int i=0; i< RelNbRel; i++){
    Serial.print(RelState[i]);
    Serial.print(",");
  }
  Serial.println("");
}

void Rel_autodisconnect(){
  // Handle autodisconnexion

  // Short circuit detection
  if((RelMode != RelMode_manual) && (RelAuto_searching_short == false))// only if we are in auto mode
  {
    if((Vnow < (Vset/2)) && (Vset>50)){ // Low voltage
      if(RelAuto_shortcuircuit_trigger_raised_ms == 0){
        // Serial.println("Probable short circuit detected!");
        RelAuto_shortcuircuit_trigger_raised_ms = millis();
      }
      if (millis()-RelAuto_shortcuircuit_trigger_raised_ms > RELAUTO_MIN_LOW_VOLTAGE_TIME_MS)
      {
        // Serial.println("Short circuit confirmed!");
        // init conditions for searching short circuit:
        RelAuto_searching_short = true; //enable search
        RelAuto_shortcircuit_finder_last_ms = 0; // start immediately
        RelAuto_shortcircuit_finder_index = -1; // start by relay 0
      }
    }
    else { // No more low voltage
      if(RelAuto_shortcuircuit_trigger_raised_ms != 0){
        // Serial.println("No more short circuit!");
        RelAuto_shortcuircuit_trigger_raised_ms = 0;
      }
    }
  }

  // find short circuit and disable flag once finished
  if(RelAuto_searching_short){
    // start by disconnecting all relays
    // then wait for defined time before measuring voltage.
    // if voltage is too low, CRITICAL ERROR
    // else, power on relay 0
    // then wait for defined time before measuring voltage.
    // if voltage is too low, mark relay 1 as dead and disconnect it again
    // then, power on relay 1
    // then wait for defined time before measuring voltage.
    // if voltage is too low, mark relay 1 as dead and disconnect it again
    // ...
    // then, power on relay (RelNbRel-1)
    // then wait for defined time before measuring voltage.
    // if voltage is too low, mark relay (RelNbRel-1) as dead and disconnect it again

    if ((millis()-RELAUTO_TESTING_TIME_MS) > RelAuto_shortcircuit_finder_last_ms) {
      if(RelAuto_shortcircuit_finder_index == -1){ // initial value
        Rel_allOff();
        RelAuto_shortcircuit_finder_index=0;
        // Serial.print("All relays off: ");
        // Rel_printState();
      }
      else if(RelAuto_shortcircuit_finder_index==0){
        if(Vnow < (Vset/2)){
          // Serial.print("[CRITICAL] All relays disconnected, current voltage too low: ");
          // Serial.print(Vnow);
          // Serial.println("V");
          // Rel_printState();
          // Serial.println("Retry...");
          Rel_auto_reset();
          return;
        }
        else{
          // Serial.print("Testing to connect relay:");
          // Serial.println(RelAuto_shortcircuit_finder_index);
          Rel_setRel(0, true);
          // Rel_printState();
          RelAuto_shortcircuit_finder_index++;
        }
      }
      else if (RelAuto_shortcircuit_finder_index < RelNbRel){
        if(Vnow<(Vset/2)){
          // Serial.print("Relay");
          // Serial.print(RelAuto_shortcircuit_finder_index-1);
          // Serial.println(" was shorted, disconnecting again");
          Rel_setRel(RelAuto_shortcircuit_finder_index-1, false);
        }
        // else{
        //   Serial.print("Relay ");
        //   Serial.print(RelAuto_shortcircuit_finder_index-1);
        //   Serial.println(" was ok");
        // }

        // Serial.print("Testing to connect relay:");
        // Serial.println(RelAuto_shortcircuit_finder_index);
        Rel_setRel(RelAuto_shortcircuit_finder_index, true);

        // Rel_printState();
        RelAuto_shortcircuit_finder_index++;
      }
      else{
        RelAuto_searching_short = false;
      }
      // store time of last action
      RelAuto_shortcircuit_finder_last_ms = millis();
    }
  }

  if(RelMode==RelMode_auto_disconnect_retry){
    // Reset states after checking time
    if ((millis()-RelRetry_last_ms)/1000 > RelRetryPeriod_s) {
      Rel_auto_reset();
    }
  }
}

void Rel_auto_reset(){
  // Serial.println("Reseting relays");
  // Serial.print("Old state:");
  // Rel_printState();
  RelRetry_last_ms = millis();
  Rel_allOn(); //activate all relays
  RelAuto_searching_short = false; // reset flags
  RelAuto_shortcuircuit_trigger_raised_ms = 0; // reset timer
  RelAuto_shortcircuit_finder_index = -1;
  // Serial.print("New state:");
  // Rel_printState();
}
