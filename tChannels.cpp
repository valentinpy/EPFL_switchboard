#include "Arduino.h"
#include "include/tChannels.h"
#include "include/tDCDC.h"
#include "include/tOC.h"


// external instances of others tasks
extern TDCDC gTDCDC;
extern TOC gTOC;

void TChannels::setup() {
	for (int i = 0; i < NBREL; i++) {
		pinMode(Rel_pin[i], OUTPUT);
		digitalWrite(Rel_pin[i], LOW);
		Rel_status[i] = false;
		RelState_testing[i] = 0;
	}
	state = STATE_RST;
	auto_disconnect_enabled = false;
	auto_reconnect_enabled = false;
	short_detected = 0;
	main_timer = millis();
}

void TChannels::voltage_drop_detected_callback() {
	if (auto_disconnect_enabled && !short_detected) {
		// unless we're already testing for shorts, disconnect immediately!
		state = STATE_SHORT_DETECTED;
		run(); // run immediately to execute disconnection routine
	}
}

// private
bool TChannels::set1(unsigned int channel, bool state) {
	
	if (Rel_status[channel] != state) {  // only need to do anything if the new state is different
		digitalWrite(Rel_pin[channel], state);
		gTDCDC.reset_stabilization_timer(); // any time we switch anything, we have to assume the voltage is no longer stable
		Rel_status[channel] = state;
	}
	return true;
}

// private
bool TChannels::set6(bool* state) {
	for (int i = 0; i < NBREL; i++) {
		set1(i, state[i]);
	}
	return true;
}

// public
void TChannels::allOn() {
	bool relays[6] = { 1,1,1,1,1,1 };
	setAllRelays(relays);
}

// public
void TChannels::allOff() {
	bool relays[6] = { 0,0,0,0,0,0 };
	setAllRelays(relays);
}

// public
bool TChannels::setRelay(unsigned int channel, bool rel_state)
{
	if (channel >= NBREL) { // channel out of range
		Serial.print("[WARN]: Channel ");
		Serial.print(channel);
		Serial.println("does not exist!");
		return false;
	}

	Rel_enabled[channel] = rel_state; // since this is a command from outside, we need to make this state permanent
	set1(channel, rel_state); // switch relay
	state = STATE_RST;  // trigger reset
	return true;
}

// public
bool TChannels::setAllRelays(bool* state)
{
	for (int i = 0; i < NBREL; i++) {
		setRelay(i, state[i]);
	}
	return true;
}

void TChannels::setAutoRestartDelay(int16_t delay)
{
	autoRestartDelay_s = delay;
	timer_relretry = millis(); // start counting from now
}

void TChannels::reset()
{
	state = STATE_RST;
}

void TChannels::autoMode(int aNewAutoRestartDelay_s, bool reconnect_enabled, bool* aListChannelsUsed) {

	//Serial.println("[INFO]: activating automode");

	auto_disconnect_enabled = true;
	auto_reconnect_enabled = reconnect_enabled;
	autoRestartDelay_s = aNewAutoRestartDelay_s;

	// enable selected relays
	setAllRelays(aListChannelsUsed);

	// tell state machine to start!
	state = STATE_RST;
}

void TChannels::getChannelsStatus(bool* retVal) {
	for (int i = 0; i < NBREL; i++) {
		retVal[i] = Rel_status[i];
	}
}

bool TChannels::isTestingShort() {
	return short_detected && auto_reconnect_enabled;
}

bool TChannels::isShortDetected(){
	return short_detected;
}

void TChannels::printChannelsStatus() {
	Serial.print(Rel_status[0]);
	for (int i = 1; i < NBREL; i++) {
		Serial.print(",");
		Serial.print(Rel_status[i]);
	}
	Serial.println("");
}

void TChannels::run() {
	
	//if (state == STATE_OFF) {
	//	// Disabled
	//	timer_relretry = 0; // reset relretry timer
	//	timer1 = 0; // reset generic timer

	//	shortcircuit_finder_index = 0;

	//	gTDCDC.restore_voltage();

	//	//Next state: stay here
	//	state = STATE_OFF;
	//	testingShort = 0;
	//}

	if (state == STATE_RST) {
		// Reset
		Serial.println("[INFO]: Resetting relay state");

		//reset everything that has to be reseted and activate all relays
		timer_relretry = millis(); // store current time for automatic reconnection
		timer1 = 0; // reset generic timer
		shortcircuit_finder_index = 0;

		// enable relays selected by user
		set6(Rel_enabled);
		short_detected = 0;

		// Ensure we have correct target voltage
		gTDCDC.restore_voltage();
		gTOC.ac_paused = false;  // make sure AC is not disabled

		// Next state, normal mode
		state = STATE_NORMAL;
	}

	if (state == STATE_NORMAL) {

		if (auto_disconnect_enabled) {

			// check reset timer and trigger reset if necessary
			if ((autoRestartDelay_s > 0) && ((millis() - timer_relretry) > (autoRestartDelay_s * 1000))) {
				// Autoretry mode activated and time interval for checking elapsed
				// Go to reset state
				state = STATE_RST;
				return; // reset pending so nothing else matters
			}

			// check voltages
			uint16_t Vset = gTDCDC.get_Vset();
			uint16_t Vnow = gTDCDC.get_last_Vnow();
			
			//if (((Vnow < Vthreshold) && ((millis() - timer1) >= RELAUTO_LOW_VOLTAGE_TIME_THRESH_MS))) { // don't need to check the actual voltage anymore. tDCDC does that now
			if (gTDCDC.get_duration_voltage_low() >= RELAUTO_LOW_VOLTAGE_TIME_THRESH_MS) {
				Serial.println("[INFO]: Voltage drop detected! [slow]");
				// Short circuit confirmed
				// Go to confirmed short circuit init case
				timer1 = 0;
				state = STATE_SHORT_DETECTED;
			}
		}
		else {
			state = STATE_NORMAL; // auto disconnect is not enabled so we don't do anything and remain in normal state
			return;
		}
		
	}

	if (state == STATE_SHORT_DETECTED) {
		// decrease voltage, wait, and disconnect everything, and cancel Switching
		// reset temporary relay states
		for (int i = 0; i < NBREL; i++) {
			RelState_testing[i] = 0;
		}
		set6(RelState_testing); // disconnect all relays
		Serial.println("[INFO]: All channels disconnected!");

		short_detected = 1;

		if (auto_reconnect_enabled) {
			shortcircuit_finder_index = 0; // start testing from relay 0
			//Decrease voltage for testing
			gTDCDC.target_voltage_modifier = TEMP_DECREASE_MODIFIER;
			//  turn AC off
			gTOC.ac_paused = true;
			timer1 = millis();  // start timer for next state
			// go to next state
			state = STATE_SHORT_WAITING;
			Serial.println("[INFO]: Testing for short circuits...");

		}
		else {
			state = STATE_NORMAL; // keep relays off, otherwise pretend everything is normal
		}
	}
	
	else if (state == STATE_SHORT_WAITING) {
		// with all relays off, wait until voltage is stable
		if (gTDCDC.is_voltage_stable()) {
			// voltage is stable, let's turn on the next relay and see if it remains stable

			// first, find out which one is the next enabled relay
			while (shortcircuit_finder_index < NBREL && !Rel_enabled[shortcircuit_finder_index])
				shortcircuit_finder_index++; // keep increasing if the current channel is not enabled, so it will be skipped

			//Serial.print("[INFO]: Relays off, voltage stabilized -> switching on channel ");
			//Serial.println(shortcircuit_finder_index);

			if (shortcircuit_finder_index < NBREL) {
				// next channel index is in range, so we're not done testing yet
				set1(shortcircuit_finder_index, true);
				timer1 = millis();  // reset timer for next state
				state = STATE_SHORT_TESTING;
			}
			else {
				// all channels tested, so let's finish up and resume normal operation
				state = STATE_SHORT_TESTING_DONE;
			}
		}
		else if ((millis() - timer1) > RELAUTO_WAITING_VOTLAGE_REG_TIME_MS) {
			// timeout expired, voltage did not stabilize -> seems to be a problem with the HVPS, not any of the samples
			Serial.println("[ERR]: Voltage did not stabilize with all relays of. Shutting down!");
			gTDCDC.shutdown();
			state = STATE_ERR;
		}
		else {
			// not yet stable -> stay in this state to keep waiting
			state = STATE_SHORT_WAITING;
		}
	}

	else if (state == STATE_SHORT_TESTING) {
		// with one relay on, wait until voltage is stable (or timeout expired)
		bool v_stable = gTDCDC.is_voltage_stable();
		if (v_stable || (millis() - timer1) > RELAUTO_TESTING_TIME_MS) {
			// we're done with this channel -> store result and move on to the next

			RelState_testing[shortcircuit_finder_index] = v_stable; // store whether or not this channel was fine

			//if (v_stable) {
			//	Serial.print("[INFO]: Voltage stabilized, channel ");
			//	Serial.print(shortcircuit_finder_index);
			//	Serial.println(" is OK");
			//}
			//else {
			//	Serial.print("[INFO]: Voltage did not stabilize, channel ");
			//	Serial.print(shortcircuit_finder_index);
			//	Serial.println(" is faulty");
			//}
			set1(shortcircuit_finder_index, false); // turn this channel off again


			shortcircuit_finder_index++; // move on to next channel

			timer1 = millis();  // reset timer for next state
			state = STATE_SHORT_WAITING; // back to waiting with relays off
		}
		else {
			// not yet stable -> stay in this state to keep waiting
			state = STATE_SHORT_TESTING;
		}
	}

	else if (state == STATE_SHORT_TESTING_DONE) {
		// test is done -> reconnect good samples, restore the proper voltage and resume

		set6(RelState_testing);

		Serial.print("[INFO]: Test finished. Reconnecting channels: ");
		printChannelsStatus();

		// test finished so we can restore the proper voltage
		gTDCDC.restore_voltage();
		gTOC.ac_paused = false;
		timer1 = 0;  // reset timer so we don't immediatly detect another short circuit while the voltage is restored

		short_detected = false; // detected short has been handled
		state = STATE_NORMAL;
	}

}
