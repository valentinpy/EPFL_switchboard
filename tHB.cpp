#include "Arduino.h"
#include "include/tHB.h"
#include "include/tDCDC.h"


// external instances of others tasks
extern TDCDC gTDCDC;

// TODO: handle PWM / high speed

void THB::setup(){
	// 4 control pins as output
	pinMode(HB_LINA_PIN, OUTPUT);
	pinMode(HB_LINB_PIN, OUTPUT);
	pinMode(HB_HINA_PIN, OUTPUT);
	pinMode(HB_HINB_PIN, OUTPUT);

	// At start, H-Bridge shorts output to ground as a protection
	digitalWrite(HB_HINA_PIN, LOW);
	digitalWrite(HB_HINB_PIN, LOW);
	delay(THB::REL_DELAY_MS); // delay for short circuit protection
	digitalWrite(HB_LINA_PIN, HIGH);
	digitalWrite(HB_LINB_PIN, HIGH);

	// stay in standby mode until there is a change requested by user
	stateMachine = standby;

	//Operation mode
	//TODO: could be stored in EEPROM, as well as frequency
	operationMode = OPMANUAL;
}

void THB::run() {
	// If operation mode is manual, change state only if gTcomm told us to do it
	if (operationMode == OPMANUAL) {
		if (newStateS.stateChanged) {
			newStateS.stateChanged = false; // reset flag
			internalRun(true, newStateS.state);
		}
		else {
			internalRun(false, DONTCARE);
		}
	}
	// Else, we handle that according to frequency
	// TODO: This is "soft PWM", should use hardware counter compare
	else {
	//Serial.println(period_us);
		if (micros() - timer_freq_us > period_us && !ac_paused) {  // don't switch if AC is paused
			timer_freq_us = micros();
			frequency_toggler = ((frequency_toggler + 1) % 2); //0,1,0,1,...
			internalRun(true, (frequency_toggler+1)); //1,2,1,2,...
		}
		else {
			internalRun(false, DONTCARE);
		}
	}
}

void THB::stateChange(uint8_t newState) {
	newStateS.stateChanged = true;
	newStateS.state = newState;
}

void THB::forceState(uint8_t newState) {
	// Immediately start switching to the specified state
	newStateS.state = newState;
	internalRun(true, newState);
}

uint8_t THB::getState(){
	return currentState;
}

uint8_t THB::getOperationMode() {
	return operationMode;
}

uint16_t THB::getMaxFrequencyHz() {
	return MAXFREQUENCY_HZ;
}

void THB::setOperationMode(operationModeEnum newOpMode, double newFrequency) {
	operationMode = newOpMode; // save new operation mode
	if (newFrequency != 0) {
		period_us = (uint32_t)(1000000 / (2 * newFrequency)); // if we are in frequency mode, save frequency (not used in manual mode)
		timer_freq_us = micros();// start timer //TODO: check for overflow after about 70 minutes
	}
	else {
		// if invalid frequency, switch back to manual mode
		operationMode = OPMANUAL;
		newStateS.state = GND;
		newStateS.stateChanged = true;
	}
	frequency_toggler = 0; // init "toggler", variable used to switch from low to high,... in frequency mode
}

void THB::internalRun(bool stateChange, stateEnum newState){
	
	if (stateChange) {
		//Serial.print("Changing state:");
		//Serial.println(newState);
		// depending of new case, determine whih relays should be ON or OFF
		// todo: can be optimized by lookup table (lina, linb, hina, hinb)
		switch (newState)
		{
		case HIGHZ:
			lina = 0;
			linb = 0;
			hina = 0;
			hinb = 0;
			break;
		case HVA:
			lina = 0;
			linb = 1;
			hina = 1;
			hinb = 0;
			break;
		case HVB:
			lina = 1;
			linb = 0;
			hina = 0;
			hinb = 1;
			break;
		case GND:
		default:
			lina = 1;
			linb = 1;
			hina = 0;
			hinb = 0;
			break;
		}
		currentState = newState;
		// next state: disconnect all that has to be disconnected
		stateMachine = disconnect;
	}

	if (stateMachine == disconnect) 
	{
		//disconnect all that has to be disconnected
		if (!hina) {
			digitalWrite(HB_HINA_PIN, LOW);
		}
		if (!hinb) {
			digitalWrite(HB_HINB_PIN, LOW);
		}
		if (!lina) {
			digitalWrite(HB_LINA_PIN, LOW);
		}
		if (!linb) {
			digitalWrite(HB_LINB_PIN, LOW);
		}
		gTDCDC.reset_stabilization_timer();  // so we don't detect a short just after switching the HB
		//start a timer to wait (non blocking) for a few milliseconds to be sure relays switched
		timer = millis();
		//netxt state: reconnect what has to be reconnected
		stateMachine = reconnect;
	}

	if (stateMachine == reconnect && millis() - timer >= REL_DELAY_MS)
	{
		// reconnect what has to be reconnected
		if (lina) {
			digitalWrite(HB_LINA_PIN, HIGH);
		}
		if (linb) {
			digitalWrite(HB_LINB_PIN, HIGH);
		}
		if (hina) {
			digitalWrite(HB_HINA_PIN, HIGH);
		}
		if (hinb) {
			digitalWrite(HB_HINB_PIN, HIGH);
		}
		gTDCDC.reset_stabilization_timer();  // so we don't detect a short just after switching the HB
		//finished, go back to standby
		stateMachine = standby;
	}

}


