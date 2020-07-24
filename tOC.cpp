#include "Arduino.h"
#include "include/tOC.h"
#include "include/tDCDC.h"

// external instances of others tasks
extern TDCDC gTDCDC;

//enum stateEnum { GND, HIGHZ, HVA, HBB };
//enum stateMachineEnum { reset, standby, newState, disconnect, reconnect };

// TODO: handle PWM / high speed

void TOC::setup() {
	// 2 control pins as output
	pinMode(OC_L_PIN, OUTPUT);
	pinMode(OC_L_PIN, OUTPUT);

	// At start, OC shorts output to ground as a protection
	digitalWrite(OC_H_PIN, LOW);
	delay(TOC::OC_DELAY_MS); // delay for short circuit protection
	digitalWrite(OC_L_PIN, HIGH);

	// stay in standby mode until there is a change requested by user
	stateMachine = standby;

	//Operation mode
	//TODO: could be stored in EEPROM, as well as frequency
	operationMode = OPMANUAL;
}

void TOC::run() {
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
		if (micros() - timer_freq_us > period_us && !ac_paused) {
			timer_freq_us = micros();
			//Serial.println("YOLO");
			frequency_toggler = ((frequency_toggler + 1) % 2); //0,1,0,1,...
			//Serial.println(frequency_toggler);
			internalRun(true, (frequency_toggler));
		}
		else {
			internalRun(false, DONTCARE);
		}
	}
}

void TOC::stateChange(uint8_t newState) {
	newStateS.stateChanged = true;
	newStateS.state = newState;
  gTDCDC.reset_stabilization_timer();  // so we don't detect a short just after switching the OCs
  // TODO: figure something out so we don't trigger short detectin on each cycle if switching at low frequency
}

uint8_t TOC::getState() {
	return newStateS.state;
}

uint8_t TOC::getOperationMode() {
	return operationMode;
}

uint16_t TOC::getMaxFrequencyHz() {
	return MAXFREQUENCY_HZ;
}

void TOC::setOperationMode(operationModeEnum newOpMode, double newFrequency) {
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



void TOC::internalRun(bool stateChange, stateEnum newState) {
	if (stateChange) {
		//Serial.print("Changing state:");
		//Serial.println(newState);
		// depending of new case, determine whih optocoupler should be ON or OFF
		// todo: can be optimized by lookup table (lin, hin)
		switch (newState)
		{
		case HIGHZ:
			hin = 0;
			lin = 0;
			break;
		case HV:
			hin = 1;
			lin = 0;
			break;
		case GND:
		default:
			hin = 0;
			lin = 1;
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
		if (!hin) {
			digitalWrite(OC_H_PIN, LOW);
		}
		if (!lin) {
			digitalWrite(OC_L_PIN, LOW);
		}
		//start a timer to wait (non blocking) for a few milliseconds to be sure relays switched
		timer = millis();
		//netxt state: reconnect what has to be reconnected
		stateMachine = reconnect;
		break;
	case reconnect:
		//wait until delay over
		if (millis() - timer >= OC_DELAY_MS) {
			// reconnect what has to be reconnected
			if (hin) {
				digitalWrite(OC_H_PIN, HIGH);
			}
			if (lin) {
				digitalWrite(OC_L_PIN, HIGH);
			}
			//finished, go back to standby
			stateMachine = standby;
		}
		break;

	case reset:
	default:
		// apply GND if we arrive here (shouldn't happen)
		hin = 0;
		lin = 1;
		stateMachine = disconnect;
		break;
	}
}
