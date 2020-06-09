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
		if (micros() - timer_freq_us > period_us) {
			timer_freq_us = micros();
			frequency_toggler = ((frequency_toggler + 1) % 2); //0,1,0,1,...
			internalRun(true, (frequency_toggler+1)); //1,2,1,2,...
		}
		else {
			internalRun(false, DONTCARE);
		}
	}

	// If we have a short circuit for too long, disconnect H-Bridge as a protection for DCDC
	if (long_shortCircuitProtection()) {
		operationMode = OPMANUAL;
		newStateS.stateChanged = true;
		newStateS.state = GND;
		Serial.println("[WARN]: long short circuit detected - deactivating H-Bridge");
	}
}

void THB::stateChange(uint8_t newState) {
	newStateS.stateChanged = true;
	newStateS.state = newState;
}

uint8_t THB::getState(){
	return newStateS.state;
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
		// next state: disconnect all that has to be disconnected
		stateMachine = disconnect;
	}

	switch (stateMachine) {
	case standby:
		// do nothing while no change is requested
		break;
	case disconnect:
		//disconnect all that has to be disconnected
		if (!lina) {
			digitalWrite(HB_LINA_PIN, LOW);
		}
		if (!linb) {
			digitalWrite(HB_LINB_PIN, LOW);
		}
		if (!hina) {
			digitalWrite(HB_HINA_PIN, LOW);
		}
		if (!hinb) {
			digitalWrite(HB_HINB_PIN, LOW);
		}
		//start a timer to wait (non blocking) for a few milliseconds to be sure relays switched
		timer = millis();
		//netxt state: reconnect what has to be reconnected
		stateMachine = reconnect;
		break;
	case reconnect:
		//wait until delay over
		if (millis() - timer > REL_DELAY_MS) {
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
			//finished, go back to standby
			stateMachine = standby;
		}
		break;

	case reset:
	default:
		// apply GND short if we arrive here (shouldn't happen)
		lina = 1;
		linb = 1;
		hina = 0;
		hinb = 0;
		stateMachine = disconnect;
		break;
	}
}

bool THB::long_shortCircuitProtection() {
	// This methods compares target voltage and measured voltage
	// If the measured voltage is too log for a defined time: LSCP_MAX_TIME_MS, the functions return true
	// To cancel the LSCP_MAX_TIME_MS timer, voltage has to be restored over threshold for at least LSCP_CANCEL_TIME_MS
	// Method is non blocking, and the called should read return value
	
	// Return value is true if a long short-circuit has been detected


	// detect begin of short circuit
	uint16_t Vnow = gTDCDC.get_last_Vnow();
	uint16_t Vset = gTDCDC.get_Vset();

	if ((Vnow < (Vset / 2)) && (Vset > 50) && (gTDCDC.get_enable_switch())) { // low voltage
		timer_lscp_1 = 0;
		if (timer_lscp_2 == 0) { // Low voltage first time
			timer_lscp_2 = millis();
		}
		else if ((timer_lscp_2 != 0) && (millis() - timer_lscp_2) > LSCP_MAX_TIME_MS) {
			timer_lscp_2 = 0;
			return true;
		}
	}
	else { // not low voltage anymore for 500ms
		if (timer_lscp_1 == 0) {
			timer_lscp_1 = millis();
		}
		if ((millis() - timer_lscp_1) > LSCP_CANCEL_TIME_MS) {
			timer_lscp_2 = 0;
			timer_lscp_1 = 0;
		}
	}
	return false;
}
