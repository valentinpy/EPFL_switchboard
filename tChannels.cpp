#include "Arduino.h"
#include "include/tChannels.h"
#include "include/tDCDC.h"
#include "include/tOC.h"


// external instances of others tasks
extern TDCDC gTDCDC;
extern TOC gTOC;

void TChannels::setup(){
  for(int i=0; i<NBREL; i++){
    pinMode(Rel_pin[i], OUTPUT);
    digitalWrite(Rel_pin[i], LOW);
    Rel_status[i] = false;
    RelState_testing[i] = 0;
  }
  autoModeState = AUTOSTATE_OFF;
  testingShort = 0;
  main_timer = millis();
}

void TChannels::run(){
//    if ((millis() - main_timer) > MAIN_DELAY_MS) {
//        main_timer = millis();
        if ((currentMode == AutoMode) && (gTDCDC.get_Vset() > 50) && (gTDCDC.get_enable_switch())) {
            bool currentlyTesting = relay_state_machine();
        }
//    }
}

void TChannels::voltage_drop_detected_callback() {
    if (!testingShort) {
        // unless we're already testing for shorts, test for shorts!
        voltage_drop_detected = true;
        run();
    }
}

void TChannels::setCurrentMode(autoModeEnum newCurrentMode) {
    currentMode = newCurrentMode;
}

void TChannels::allOn(){
  for(int i=0; i<NBREL; i++){
    digitalWrite(Rel_pin[i], HIGH);
    Rel_status[i] = true;
  }
  currentMode = AllOn;
  gTDCDC.restore_voltage();
  testingShort = 0;
}

void TChannels::allOff(){
  for(int i=0; i<NBREL; i++){
    digitalWrite(Rel_pin[i], LOW);
    Rel_status[i] = false;
  }
  currentMode = AllOff;
  gTDCDC.restore_voltage();
  testingShort = 0;
}

bool TChannels::set1(unsigned int channel, bool state){
  if(channel < NBREL){
    digitalWrite(Rel_pin[channel], state);
    Rel_status[channel] = state;
    gTDCDC.reset_stabilization_timer(); // reset timer so that we don't just assume voltage is stable because it was a moment ago
//    currentMode = Manual;
//    testingShort = 0;
    return true;
  }
  else{
    // Channel out of bound
    return false;
  }
}

bool TChannels::set6(bool* state) {
    for (int i = 0; i < NBREL; i++) {
        digitalWrite(Rel_pin[i], state[i]);
        Rel_status[i] = state[i];
    }
    gTDCDC.reset_stabilization_timer(); // reset timer so that we don't just assume voltage is stable because it was a moment ago
    //currentMode = Manual;
    return true;
}

void TChannels::autoMode(int aNewAutoRestartDelay_s, bool* aListChannelsUsed){

    //Serial.println("[INFO]: activating automode");

    autoRestartDelay_s = aNewAutoRestartDelay_s;

    // apply initial state to relays and store it into Rel_automode_enabled[
    for (int i = 0; i < NBREL; i++) {
        digitalWrite(Rel_pin[i], aListChannelsUsed[i]);
        Rel_status[i] = aListChannelsUsed[i];
        Rel_automode_enabled[i] = aListChannelsUsed[i];
    }

    // tell state machine to start!
    autoModeState = AUTOSTATE_RST;
}

void TChannels::getChannelsStatus(bool* retVal){
    for(int i=0; i<NBREL; i++){
        retVal[i] = Rel_status[i];
    }
}

int8_t TChannels::isTestingShort() {
    return testingShort;
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

bool TChannels::relay_state_machine() {
    // Handle autodisconnexion
    uint16_t Vset = gTDCDC.get_Vset();
    uint16_t Vnow = gTDCDC.get_last_Vnow();
    uint32_t Vthreshold = ((uint32_t)Vset) * (uint32_t)(THRESHOLD_PERCENT) / 100;

    if (autoModeState == AUTOSTATE_OFF) {
        // Disabled
        timer_relretry = 0; // reset relretry timer
        timer1 = 0; // reset generic timer
        
        shortcircuit_finder_index = 0;

        gTDCDC.restore_voltage();

        //Next state: stay here
        autoModeState = AUTOSTATE_OFF;
        testingShort = 0;
    }

    if (autoModeState == AUTOSTATE_RST) {
        // Reset
        Serial.println("[INFO]: AUTOSTATE_RST");

        //reset everything that has to be reseted and activate all relays
        timer_relretry = millis(); // store current time for automatic reconnection
        timer1 = 0; // reset generic timer
        shortcircuit_finder_index = 0;

        // enable relays selected by user
        set6(Rel_automode_enabled);

        // Ensure we have correct target voltage
        gTDCDC.restore_voltage();
        gTOC.ac_paused = false;  // make sure AC is not disabled

        // Next state, normal mode
        autoModeState = AUTOSTATE_NORMAL;
    }

    if (autoModeState == AUTOSTATE_NORMAL) {
        testingShort = 0;
        for (int i = 0; i < NBREL; i++) {
            RelState_testing[i] = 0;
        }

        // Normal mode: detect short circuits and handle auto retry if activated
        if ((Vnow < Vthreshold) && (timer1 == 0)) { // &&Vset>50
            // Low voltage: first time
            // Store time, stay in this state
            timer1 = millis();
            autoModeState = AUTOSTATE_NORMAL;
        }
        else if (Vnow >= Vthreshold) { //  && (Vset > 50)
            // No low voltage anymore
            // Reset short circuit timer and stay in this state
            timer1 = 0;
            autoModeState = AUTOSTATE_NORMAL;
        }

        if (voltage_drop_detected || ((Vnow < Vthreshold) && ((millis() - timer1) >= RELAUTO_MIN_LOW_VOLTAGE_TIME_MS))) { //&& (Vset > 50) 
            // Short circuit confirmed
            // Go to confirmed short circuit init case
            voltage_drop_detected = false; // reset flag (in case it was set)
            timer1 = 0;
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_INIT;
        }

        if ((autoRestartDelay_s > 0) && ((millis() - timer_relretry) > (autoRestartDelay_s * 1000))) {
            // Autoretry mode activated and time interval for checking elapsed
            // Go to reset state
            autoModeState = AUTOSTATE_RST;
        }
    }

    if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_INIT) {
        // decrease voltage, wait, and disconnect everything, and cancel Switching
        Serial.println("[INFO]: AUTOSTATE_CONFIRMED_SHORT_INIT");
        //Decrease voltage for testing
        gTDCDC.target_voltage_modifier = TEMP_DECREASE_MODIFIER;
        //  turn AC off
        gTOC.ac_paused = true;
        // disconnect all relays
        bool tmp[6] = { 0,0,0,0,0,0 };
        set6(tmp);
        
        testingShort = 1;
        shortcircuit_finder_index = 0; // start testing from relay 0
               
        timer1 = millis();  // start timer for next state

        // go to next state
        autoModeState = AUTOSTATE_CONFIRMED_SHORT_WAITING;
    }

    else if (autoModeState == AUTOSTATE_CONFIRMED_SHORT_WAITING) {
        // with all relays off, wait until voltage is stable
        if (gTDCDC.is_voltage_stable()) {
            // voltage is stable, let's turn on one relay and see if it remains stable
            Serial.print("[INFO]: Relays off, voltage stabilized -> switching on channel ");
            Serial.println(shortcircuit_finder_index);
            if (shortcircuit_finder_index < NBREL) {
                // next channel index is in range, so we're not done testing yet
                set1(shortcircuit_finder_index, true);
                timer1 = millis();  // reset timer for next state
                autoModeState = AUTOSTATE_CONFIRMED_SHORT_TESTING;
            }
            else {
                // all channels tested, so let's finish up and resume normal operation
                autoModeState = AUTOSTATE_CONFIRMED_SHORT_TESTING_DONE;
            }
        }
        else if ((millis() - timer1) > RELAUTO_WAITING_VOTLAGE_REG_TIME_MS) {
            // timeout expired, voltage did not stabilize -> seems to be a problem with the HVPS, not any of the samples
            Serial.println("[ERR]: Voltage did not stabilize with all relays off, aborting - settting target voltage to 0");
            gTDCDC.set_target_voltage(0);
            autoModeState= AUTOSTATE_OFF;
        }
        else {
            // not yet stable -> stay in this state to keep waiting
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_WAITING;
        }
    }

    else if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_TESTING) {
        // with one relay on, wait until voltage is stable (or timeout expired)
        bool v_stable = gTDCDC.is_voltage_stable();
        if (v_stable || (millis() - timer1) > RELAUTO_TESTING_TIME_MS) {
            // we're done with this channel -> store result and move on to the next
            
            RelState_testing[shortcircuit_finder_index] = v_stable; // store whether or not this channel was fine

            if(v_stable){
                Serial.print("[INFO]: Voltage stabilized, channel ");
                Serial.print(shortcircuit_finder_index);
                Serial.println(" is OK");
            } else {
                Serial.print("[INFO]: Voltage did not stabilize, channel ");
                Serial.print(shortcircuit_finder_index);
                Serial.println(" is faulty");
            }
            set1(shortcircuit_finder_index, false); // turn this channel off again

            shortcircuit_finder_index++; // move on to next channel
            
            timer1 = millis();  // reset timer for next state
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_WAITING; // back to waiting with relays off
        }
        else {
            // not yet stable -> stay in this state to keep waiting
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_TESTING;
        }
    }

    else if (autoModeState == AUTOSTATE_CONFIRMED_SHORT_TESTING_DONE) {
        // test is done -> reconnect good samples, restore the proper voltage and resume

        set6(RelState_testing);
        
        Serial.print("[INFO] finished: ");
        printChannelsStatus();

        // test finished so we can restore the proper voltage
        gTDCDC.restore_voltage();
        timer1 = 0;  // reset timer so we don't immediatly detect another short circuit while the voltage is restored
        
        timer_relretry = millis();
        autoModeState= AUTOSTATE_NORMAL;
        
    }
    
    if ((autoModeState == AUTOSTATE_NORMAL) || (autoModeState == AUTOSTATE_OFF)) {
        return false;
    }
    else {
        return true;
    }
}
