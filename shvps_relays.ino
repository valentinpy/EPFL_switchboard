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

void Rel_apply_testing(){
  // apply RelState values to all relays
  for(int i=0; i< RelNbRel; i++){
    RelState[i] = RelState_testing[i];
    digitalWrite(RelCmd_pin[i], RelState_testing[i]);
  }
}

bool Rel_currentlyTesting(){
  return (!((RelAutoState == RelAutoStateNormal) || (RelAutoState == RelAutoStateOff)));
}


void Rel_printState(){
  // Print rekays state
  if(!(Rel_currentlyTesting())){ // Not curently solving a short-circuit problem
    Serial.print("Relays state: ");
    for(int i=0; i< RelNbRel; i++){
      Serial.print(RelState[i]);
      Serial.print(",");
    }
    Serial.println("");
  }
  else{ // We are trying to find the source of short-circuit, current relay state doesn't make sense
  Serial.print("Relays state: ");
  for(int i=0; i< RelNbRel; i++){
    Serial.print("2");
    Serial.print(",");
  }
  Serial.println("");
}
}

void Rel_long_shortcircuit_protection(){
  // detect begin of short circuit
  if((Vnow < (Vset/2)) && (Vset>50)){ // low voltage
    Rel_long_shortcircuit_protection_cancel_last_ms = 0;
    if(Rel_long_shortcircuit_protection_last_ms == 0){ // Low voltage first time
      Rel_long_shortcircuit_protection_last_ms = millis();
    }
    else if ((Rel_long_shortcircuit_protection_last_ms != 0) &&(millis() - Rel_long_shortcircuit_protection_last_ms) > REL_LONG_SHORTCIRCUIT_PROTECTION_TIME_MS){
      RelMode = RelMode_manual;
      Rel_allOff();
      Rel_long_shortcircuit_protection_last_ms = 0;
    }
  }
  else{ // not low voltage anymore for 500ms
    if (Rel_long_shortcircuit_protection_cancel_last_ms == 0){
      Rel_long_shortcircuit_protection_cancel_last_ms = millis();
    }
    if((millis()-Rel_long_shortcircuit_protection_cancel_last_ms) > 500){
      Rel_long_shortcircuit_protection_last_ms = 0;
      Rel_long_shortcircuit_protection_cancel_last_ms = 0;
    }
  }
}

void Rel_autodisconnect(){
  // Handle autodisconnexion

  if(RelAutoState == RelAutoStateOff){
    // Disabled
    RelRetry_last_ms = 0;
    RelAuto_shortcuircuit_trigger_raised_ms = 0; // reset timer
    RelAuto_shortcircuit_finder_index = 0;

    // Ensure we have correct target voltage
    if(Vset_normal>0){
      Vset = Vset_normal;
    }

    //Next state: stay here
    RelAutoState = RelAutoStateOff;
  }

  else if(RelAutoState == RelAutoReset){
    // Reset

    //reset everything that has to be reseted and activate all relays
    RelRetry_last_ms = millis(); // store current time for automatic reconnection
    RelAuto_shortcuircuit_trigger_raised_ms = 0; // reset short circuit detection timer
    RelAuto_shortcircuit_voltage_reg_last_ms = 0; // reset voltage change timer
    RelAuto_shortcircuit_finder_index = 0;
    SwMode_save_autorelay = SwMode;
    Rel_allOn();

    // Ensure we have correct target voltage
    if(Vset_normal>0){
      Vset = Vset_normal;
    }

    // Next state, normal mode
    RelAutoState = RelAutoStateNormal;
  }

  else if(RelAutoState == RelAutoStateNormal) {
    // SwMode back to normal
    SwMode = SwMode_save_autorelay;

    // Normal mode: detect short circuits and handle auto retry if activated
    Vthreshold = (word)((RELAUTO_THRESHOLD_PERCENT/100.0)*(float)Vset);

    if((Vnow < (Vthreshold)) && (Vset>50) && (RelAuto_shortcuircuit_trigger_raised_ms == 0)){
      // Low voltage: first time
      // Store time, stay in this state
      RelAuto_shortcuircuit_trigger_raised_ms = millis();
      RelAutoState = RelAutoStateNormal;
    }
    else if ((Vnow >= (Vthreshold)) && (Vset>50)) {
      // No low voltage anymore
      // Reset short circuit timer and stay in this state
      RelAuto_shortcuircuit_trigger_raised_ms = 0;
      RelAutoState = RelAutoStateNormal;
    }

    else if((Vnow < (Vthreshold)) && (Vset>50) && ((millis() - RelAuto_shortcuircuit_trigger_raised_ms) >= RELAUTO_MIN_LOW_VOLTAGE_TIME_MS)){
      // Short circuit confirmed
      // Go to confirmed short circuit init case
      RelAuto_shortcircuit_voltage_reg_last_ms = 0;
      RelAutoState = RelAutoStateConfirmedShort_init;
    }

    if ((RelMode == RelMode_auto_disconnect_retry) && ((millis() - RelRetry_last_ms) > (RelRetryPeriod_s*1000))){
      // Autoretry mode activated and time interval for checking elapsed
      // Go to reset state
      RelAutoState = RelAutoReset;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_init){
    // Save current voltage, decrease voltage, wait, and disconnect everything, and cancel Switching
    SwMode_save_autorelay = SwMode;
    SwMode = 1;

    if (RelAuto_shortcircuit_voltage_reg_last_ms == 0){
      // Serial.println("state: confirmed short init");
      // Set voltage to 0, store nominal voltage
      Vset_normal = Vset;
      Vset = 0;

      // Store current time
      RelAuto_shortcircuit_voltage_reg_last_ms = millis();

      // Stay in that state for a moment
      RelAutoState = RelAutoStateConfirmedShort_init;
    }
    else if ((millis() - RelAuto_shortcircuit_voltage_reg_last_ms) >= RELAUTO_WAITING_VOTLAGE_REG_TIME_MS){
      // Delay elapsed, voltage should have dropped, disconnect everything, start new timer and go to next state
      Rel_allOff();
      RelAutoState = RelAutoStateConfirmedShort_waiting_deco;
      RelAutoStateConfirmedShort_waiting_deco_last_ms = millis();
    }
    else{
      //stay in that case until delay elapsed
      RelAutoState = RelAutoStateConfirmedShort_init;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_waiting_deco) {
    // wait for relays to disconnect
    if ((millis() - RelAutoStateConfirmedShort_waiting_deco_last_ms) > 100){
      //relays must be disconnected yet, let's increase voltage again
      RelAuto_shortcircuit_incr_volt_last_ms = millis();
      Vset = Vset_normal;
      RelAutoState = RelAutoStateConfirmedShort_start_searching;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_start_searching) {

    //  all relays disconnected
    //increase voltage + wait
    if((millis() - RelAuto_shortcircuit_incr_volt_last_ms) > 500){
      // Voltage should have raised now (all relays disconnected)
      Vthreshold = (word)((RELAUTO_THRESHOLD_PERCENT/100.0)*(float)Vset);
      if(Vnow < Vthreshold){
        // Serial.println("Voltage still too low, aborting - settting target voltage to 0");
        Rel_allOff();
        Vset = 0;
        Vset_normal = 0;
        RelAutoState = RelAutoStateOff;
      }
      else{
        // Go to next state: trying all relays
        RelAuto_shortcircuit_finder_index = 0;

        for (int i=0; i<6; i++){
          RelState_testing[i] = false;
        }
        RelAutoState = RelAutoStateConfirmedShort_searching_1;
        RelAuto_shortcircuit_deco_last_ms = 0;
      }
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_searching_1){
      //voltage is on, relays are off
      // relay i => on
      //wait
      if (RelAuto_shortcircuit_deco_last_ms == 0){
        // Relay on, then start waiting
        Rel_setRel(RelAuto_shortcircuit_finder_index, true);
        RelAuto_shortcircuit_deco_last_ms = millis();
        RelAutoState = RelAutoStateConfirmedShort_searching_1; //stay here
      }
      else if ((millis() - RelAuto_shortcircuit_deco_last_ms) > 500){
        RelAutoState = RelAutoStateConfirmedShort_searching_2; // go to next state
      }
      else{
        //wait here
        RelAutoState = RelAutoStateConfirmedShort_searching_1;
      }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_searching_2){
    // measure voltage, store SwState
    Vthreshold = (word)((RELAUTO_THRESHOLD_PERCENT/100.0)*(float)Vset);
    if(Vnow > Vthreshold){
      RelState_testing[RelAuto_shortcircuit_finder_index] = true;
    }

    // decrease voltage
    Vset_normal = Vset;
    Vset = 0;


    // start waiting
    RelAuto_shortcircuit_deco_last_ms = millis();
    RelAutoState = RelAutoStateConfirmedShort_searching_3;

  }

  else if (RelAutoState == RelAutoStateConfirmedShort_searching_3){
    if((millis() - RelAuto_shortcircuit_deco_last_ms) > 500){
      //disconnect and wait in next state
      Rel_setRel(RelAuto_shortcircuit_finder_index, false);
      RelAuto_shortcircuit_deco_last_ms = millis();
      RelAutoState = RelAutoStateConfirmedShort_searching_4;
    }
    else{
      //wait here until V=0
      RelAutoState = RelAutoStateConfirmedShort_searching_3;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_searching_4){
    if((millis() - RelAuto_shortcircuit_deco_last_ms) > 500){
        //set voltage back on and start waiting
        Vset = Vset_normal;
        RelAuto_shortcircuit_deco_last_ms = millis();
        RelAutoState = RelAutoStateConfirmedShort_searching_5;
    }
    else{
      //wait here until relays disconnected
      RelAutoState = RelAutoStateConfirmedShort_searching_4;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_searching_5){
    if((millis() - RelAuto_shortcircuit_deco_last_ms) > 500){
      if(RelAuto_shortcircuit_finder_index>=5){
          // Serial.println("finished");
          Rel_apply_testing();
          RelRetry_last_ms = millis();
          RelAutoState = RelAutoStateNormal;
      }
      else{
          // start for next relay
          RelAuto_shortcircuit_finder_index++;
          RelAuto_shortcircuit_deco_last_ms = 0;
          RelAutoState = RelAutoStateConfirmedShort_searching_1;
          // Serial.println("Finished for a relay, testing next one");
      }
    }
    else{
      //wait here until voltage back to normal
      RelAutoState = RelAutoStateConfirmedShort_searching_5;
    }
  }
}
