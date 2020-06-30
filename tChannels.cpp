#include "Arduino.h"
#include "include/tChannels.h"
#include "include/tDCDC.h"


// external instances of others tasks
extern TDCDC gTDCDC;

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
    if ((millis() - main_timer) > MAIN_DELAY_MS) {
        main_timer = millis();
        if ((currentMode == AutoMode) && (gTDCDC.get_Vset() > 50) && (gTDCDC.get_enable_switch())) {
            bool currentlyTesting = relay_state_machine();
        }
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
  testingShort = 0;
}

void TChannels::allOff(){
  for(int i=0; i<NBREL; i++){
    digitalWrite(Rel_pin[i], LOW);
    Rel_status[i] = false;
  }
  currentMode = AllOff;
  testingShort = 0;
}

bool TChannels::set1(unsigned int channel, bool state){
  if(channel < NBREL){
    digitalWrite(Rel_pin[channel], state);
    Rel_status[channel] = state;
    currentMode = Manual;
    testingShort = 0;
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

    else if (autoModeState == AUTOSTATE_RST) {
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

        // Next state, normal mode
        autoModeState = AUTOSTATE_NORMAL;
    }

    else if (autoModeState == AUTOSTATE_NORMAL) {
        gTDCDC.restore_voltage();

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

        else if ((Vnow < Vthreshold) && ((millis() - timer1) >= RELAUTO_MIN_LOW_VOLTAGE_TIME_MS)) { //&& (Vset > 50) 
            // Short circuit confirmed
            // Go to confirmed short circuit init case
            timer1 = 0;
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_INIT;
        }

        if ((autoRestartDelay_s > 0) && ((millis() - timer_relretry) > (autoRestartDelay_s * 1000))) {
            // Autoretry mode activated and time interval for checking elapsed
            // Go to reset state
            autoModeState = AUTOSTATE_RST;
        }
    }

    else if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_INIT) {
        // decrease voltage, wait, and disconnect everything, and cancel Switching
        if (timer1 == 0) {
            //Serial.println("[INFO]: AUTOSTATE_CONFIRMED_SHORT_INIT");
            //Decrease voltage
            gTDCDC.decrease_temporary_voltage(TEMP_DECREASE);

            // Store current time
            timer1 = millis();

            // Stay in that state for a moment
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_INIT;
        }
        else if ((millis() - timer1) >= RELAUTO_WAITING_VOTLAGE_REG_TIME_MS) {
            //Serial.println("[INFO]: Disconnecting all relays");
            // Delay elapsed, voltage should have dropped, disconnect everything, start new timer and go to next state
            bool tmp[6] = { 0,0,0,0,0,0 };
            set6(tmp);
            
            // store current time for next state
            timer1 = millis();

            // go to next state
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_WAITING_DECO;
        }
        else {
            //stay in that case until delay elapsed
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_INIT;
        }
    }

    else if (autoModeState == AUTOSTATE_CONFIRMED_SHORT_WAITING_DECO) {
        testingShort = 1;

        // wait for relays to disconnect
        if ((millis() - timer1) > RELAUTO_REL_TIME_MS) {
            //relays must be disconnected yet, let's increase voltage again
            //Serial.println("[INFO]: Relays disconnected, increasing voltage");

            timer1 = millis();
            

            // start searching
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_START_SEARCHING;
        }
        else {
            // stay in that state until relays switches
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_WAITING_DECO;
        }
    }

    else if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_START_SEARCHING) {
        //  all relays disconnected
        //increase voltage + wait
        if ((millis() - timer1) > RELAUTO_TESTING_TIME_MS) {
            //Serial.println("[INFO]: AUTOSTATE_CONFIRMED_SHORT_START_SEARCHING");
            // Voltage should have raised now (all relays disconnected)
            //Vthreshold = (word)((THRESHOLD_PERCENT / 100.0) * (float)Vset);
            if (Vnow < Vthreshold) {
                Serial.println("[ERR]: Voltage still too low, aborting - settting target voltage to 0");
                bool tmp[6] = { 0,0,0,0,0,0 };
                set6(tmp);
                autoModeState= AUTOSTATE_OFF;
                gTDCDC.set_target_voltage(0);
            }
            else {
                // Go to next state: trying all relays
                shortcircuit_finder_index = 0;
               
                //Go to next state
                autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_1;
                timer1 = 0;
            }
        }
        else {
            // stay in that state until voltage rises again
            autoModeState = AUTOSTATE_CONFIRMED_SHORT_START_SEARCHING;
        }
    }

    else if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_SEARCHING_1) {
        //voltage is on, relays are off
        // relay i => on
        //wait
        if (timer1 == 0) {
            // Relay on, then start waiting
            //TODO: only do if channel activated by user!!
            //Serial.println("[INFO]: AUTOSTATE_CONFIRMED_SHORT_SEARCHING_1");

            while ((!Rel_automode_enabled[shortcircuit_finder_index]) && (shortcircuit_finder_index < 6)){
                shortcircuit_finder_index++;
            }
            if (shortcircuit_finder_index == 6) {
                autoModeState = AUTOSTATE_CONFIRMED_SHORT_SEARCHING_5;
                timer1 = millis();
            }
            else {
                Serial.print("[INFO]: Activating channel: ");
                Serial.println(shortcircuit_finder_index);
                digitalWrite(Rel_pin[shortcircuit_finder_index], true);
                timer1 = millis();
                autoModeState = AUTOSTATE_CONFIRMED_SHORT_SEARCHING_1; //stay here
            }
        }
        else if ((millis() - timer1) > RELAUTO_REL_TIME_MS) {
            //Serial.println("[INFO]: Delay elapsed, going to AUTOSTATE_CONFIRMED_SHORT_SEARCHING_2");
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_2; // go to next state
        }
        else {
            //wait here
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_1;
        }
    }

    else if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_SEARCHING_2) {
        // measure voltage
        if (Vnow > Vthreshold) {
            //Serial.print("[INFO]: Voltage above threshold, saved state: ON for channel ");
            //Serial.println(shortcircuit_finder_index);
            RelState_testing[shortcircuit_finder_index] = true;
            supplementary_delay_ms = 0;
        }
        else {
            //Serial.print("[INFO]: Voltage below threshold, saved state: OFF for channel ");
            //Serial.println(shortcircuit_finder_index);
            RelState_testing[shortcircuit_finder_index] = false;
            supplementary_delay_ms = 2*RELAUTO_WAITING_VOTLAGE_REG_TIME_MS;
        }

        // start waiting
        timer1 = millis();
        autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_3;

    }

    else if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_SEARCHING_3) {
        if ((millis() - timer1) > (RELAUTO_WAITING_VOTLAGE_REG_TIME_MS + supplementary_delay_ms)) {
            //disconnect and wait in next state
            digitalWrite(Rel_pin[shortcircuit_finder_index], false);
            timer1 = millis();
            //Serial.println("[INFO]: going to state: AUTOSTATE_CONFIRMED_SHORT_SEARCHING_4");
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_4;
        }
        else {
            //wait here until V=0
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_3;
        }
    }

    else if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_SEARCHING_4) {
        if ((millis() - timer1) > RELAUTO_REL_TIME_MS) {
            //start waiting
            timer1 = millis();
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_5;
        }
        else {
            //wait here until relays disconnected
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_4;
        }
    }

    else if (autoModeState== AUTOSTATE_CONFIRMED_SHORT_SEARCHING_5) {
        if ((millis() - timer1) > RELAUTO_WAITING_VOTLAGE_REG_TIME_MS) {
            if (shortcircuit_finder_index >= 5) {
                Serial.print("[INFO] finished ");
                //Rel_apply_testing();
                for (int i = 0; i < 6; i++) {
                    if (RelState_testing[i]) {
                        digitalWrite(Rel_pin[i], true);
                        Rel_status[i] = true;
                        Serial.print("1,");
                    }
                    else {
                        digitalWrite(Rel_pin[i], false);
                        Rel_status[i] = false;
                        Serial.print("0,");
                    }
                }
                Serial.println("");
                timer_relretry = millis();
                autoModeState= AUTOSTATE_NORMAL;
            }
            else {
                // start for next relay
                //Serial.print("[INFO]: Finished for relay");
                //Serial.print(shortcircuit_finder_index);
                //Serial.println(", testing next one");

                shortcircuit_finder_index++;
                timer1 = 0;
                autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_1;
                
            }
        }
        else {
            //wait here until voltage back to normal
            autoModeState= AUTOSTATE_CONFIRMED_SHORT_SEARCHING_5;
        }
    }
    
    if ((autoModeState == AUTOSTATE_NORMAL) || (autoModeState == AUTOSTATE_OFF)) {
        return false;
    }
    else {
        return true;
    }
}