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
  // unsigned long Rel_long_shortcircuit_protection_last_ms = 0; REL_LONG_SHORTCIRCUIT_PROTECTION_TIME_MS

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
      // Serial.println("all relays disconnected");
    }
    else{
      // Serial.println(millis() - Rel_long_shortcircuit_protection_last_ms);
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

  if(RelAutoState == RelAutoStateOff){ // Disabled
    RelRetry_last_ms = 0;
    RelAuto_shortcuircuit_trigger_raised_ms = 0; // reset timer
    RelAuto_shortcircuit_finder_index = -1;

    // Ensure we have correct target voltage
    if(Vset_normal>0){
      Vset = Vset_normal;
    }

    //Next state: stay here
    RelAutoState = RelAutoStateOff;
  }

  else if(RelAutoState == RelAutoReset){ // Reset
    RelRetry_last_ms = millis(); // store current time for
    // RelAuto_searching_short = false; // reset flags
    RelAuto_shortcuircuit_trigger_raised_ms = 0; // reset timer
    RelAuto_shortcircuit_finder_index = -1;
    Rel_allOn();

    // Ensure we have correct target voltage
    if(Vset_normal>0){
      Vset = Vset_normal;
    }

    // Next state
    RelAutoState = RelAutoStateNormal;
  }

  // Short circuit detection
  else if(RelAutoState == RelAutoStateNormal) // Normal mode
  {
    if((Vnow < (Vset/2)) && (Vset>50)){ // Low voltage
      // Store time
      RelAuto_shortcuircuit_trigger_raised_ms = millis();

      // Next state: confirmation of short circuit
      RelAutoState = RelAutoStateProbableShort;
    }
    else if ((RelMode == RelMode_auto_disconnect_retry) && ((millis() - RelRetry_last_ms) > (RelRetryPeriod_s*1000))){
      RelAutoState = RelAutoReset; // TODO new state for that case
    }
    else{
      // Stay there as there is no short circuit detected
      RelAutoState = RelAutoStateNormal;
    }
  }

  else if (RelAutoState == RelAutoStateProbableShort){
    if(Vnow > (Vset/2)) {// && (Vset>50)){ // Low voltage
      // Next state, as short circuit was temporary
      RelAutoState = RelAutoStateNormal;
    }
    else if ((millis()-RelAuto_shortcuircuit_trigger_raised_ms) > RELAUTO_MIN_LOW_VOLTAGE_TIME_MS){
      // Next state: short circuit confirmed
      RelAutoState = RelAutoStateConfirmedShort_init;
    }
    else{
      // Next state: wait for sufficient time
      RelAutoState = RelAutoStateProbableShort;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_init){
    // init conditions for searching short circuit:
    RelAuto_shortcircuit_finder_last_ms = 0; // start immediately
    RelAuto_shortcircuit_finder_index = -1; // start by relay 0
    RelRetry_last_ms = millis();

    // store normal Voltage
    Vset_normal = Vset;
    Vset_reduced = 0;//Vset/5;

    // Apply lower voltage
    Vset = Vset_reduced;

    // Store time
    RelAuto_shortcircuit_voltage_reg_last_ms = millis();

    // Next state: wait for voltage to decrease
    RelAutoState = RelAutoStateConfirmedShort_decreasing_voltage;
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_decreasing_voltage){
    if((millis() - RelAuto_shortcircuit_voltage_reg_last_ms) > RELAUTO_WAITING_VOTLAGE_REG_TIME_MS){
      // Next step: disconnexion of relays
      RelAutoState = RelAutoStateConfirmedShort_deco_all;
    }
    else{
      // Wait here until timer elapsed
      RelAutoState = RelAutoStateConfirmedShort_decreasing_voltage;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_deco_all){
    if (RelAuto_shortcircuit_finder_index == -1){
      Rel_allOff();
    }
    else{
      Rel_setRel(RelAuto_shortcircuit_finder_index, false);
    }
    Vset = Vset_normal;

    // Store time
    RelAuto_shortcircuit_voltage_reg_last_ms = millis();

    // Next state: waiting for voltage to increase
    RelAutoState = RelAutoStateConfirmedShort_increasing_voltage;
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_increasing_voltage){
    if((millis() - RelAuto_shortcircuit_voltage_reg_last_ms) > RELAUTO_WAITING_VOTLAGE_REG_TIME_MS){
      // Next step: disconnexion of relays
      RelAutoState = RelAutoStateConfirmedShort_measure;
    }
    else{
      // Wait here until timer elapsed
      RelAutoState = RelAutoStateConfirmedShort_increasing_voltage;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_measure){
    if ((RelAuto_shortcircuit_finder_index >= (RelNbRel-1))&& (Vnow > (Vset/2))){
      RelAutoState = RelAutoStateNormal;
    }
    else if ((RelAuto_shortcircuit_finder_index == -1) && (Vnow < (Vset/2))){
      // Even with all realys disconnected, no voltage!
      RelAutoState = RelAutoStateOff;
    }
    else if ((RelAuto_shortcircuit_finder_index == -1) && (Vnow > (Vset/2))){
      // All was disconnected, we have voltage
      RelAuto_shortcircuit_finder_index++;
      RelAutoState = RelAutoStateConfirmedShort_reco;
    }
    else if ((RelAuto_shortcircuit_finder_index >-1) && (Vnow > (Vset/2))){
      // last reconnexion of relay was not a problem, continue
      RelAuto_shortcircuit_finder_index++;
      RelAutoState = RelAutoStateConfirmedShort_reco;
    }
    else if ((RelAuto_shortcircuit_finder_index >-1) && (Vnow < (Vset/2))){
      // last reconnexion of relay was a problem. Disconnect and continue
      // Rel_setRel(RelAuto_shortcircuit_finder_index, false);
      RelAuto_shortcircuit_finder_index++;
      Vset = Vset_reduced;
      RelAuto_shortcircuit_before_deco_last_ms = millis();
      RelAutoState = RelAutoStateConfirmedShort_deco_wait;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_reco){
    Rel_setRel(RelAuto_shortcircuit_finder_index, true);
    RelAuto_shortcircuit_before_meas_last_ms = millis();
    RelAutoState = RelAutoStateConfirmedShort_reco_wait;
  }
  else if (RelAutoState == RelAutoStateConfirmedShort_reco_wait){
    if((millis() - RelAuto_shortcircuit_before_meas_last_ms) > 500){
      RelAutoState = RelAutoStateConfirmedShort_measure;
    }
    else{
      // Wait here until timer elapsed
      RelAutoState = RelAutoStateConfirmedShort_reco_wait;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_deco_wait){
    if((millis() - RelAuto_shortcircuit_before_deco_last_ms) > 500){
      Rel_setRel(RelAuto_shortcircuit_finder_index-1, false);
      Vset = Vset_normal;
      RelAuto_shortcircuit_after_deco_last_ms = millis();
      RelAutoState = RelAutoStateConfirmedShort_deco_wait_post;
    }
    else{
      // Wait here until timer elapsed
      RelAutoState = RelAutoStateConfirmedShort_deco_wait;
    }
  }

  else if (RelAutoState == RelAutoStateConfirmedShort_deco_wait_post){
    if((millis() - RelAuto_shortcircuit_after_deco_last_ms) > 500){
      if (RelAuto_shortcircuit_finder_index<(RelNbRel)){
        RelAutoState = RelAutoStateConfirmedShort_reco;
      }
      else{
        RelAutoState = RelAutoStateNormal;
      }
    }
    else{
      // Wait here until timer elapsed
      RelAutoState = RelAutoStateConfirmedShort_deco_wait_post;
    }
  }

  else{
    // Unknow case
    // Should never happen
    RelAutoState = RelAutoStateOff;
  }
}
