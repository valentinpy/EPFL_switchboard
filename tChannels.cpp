#include "Arduino.h"
#include "include/tChannels.h"
#include "include/tDCDC.h"
#include "include/tOC.h"
#include "include/tHB.h"


// external instances of others tasks
extern TDCDC gTDCDC;
extern TOC gTOC;
extern THB gTHB;

void TChannels::setup() {
	for (int i = 0; i < NBREL; i++) {
		pinMode(Rel_pin[i], OUTPUT);
		digitalWrite(Rel_pin[i], LOW);
		Rel_status[i] = false;
		Rel_enabled[i] = false;
		Rel_test_result[i] = false;
		Channels_to_test[i] = false;
	}
	state = STATE_RST;
	auto_disconnect_enabled = false;
	auto_reconnect_enabled = false;
	keep_faulty_channels_off = false;
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
void TChannels::set1(unsigned int channel, bool state) {
	
	if (Rel_status[channel] != state) {  // only need to do anything if the new state is different
		digitalWrite(Rel_pin[channel], state);
		gTDCDC.reset_stabilization_timer(); // any time we switch anything, we have to assume the voltage is no longer stable
		Rel_status[channel] = state;
	}
}

// private
void TChannels::set6(bool* state) {
	for (int i = 0; i < NBREL; i++) {
		set1(i, state[i]);
	}
}

// public
void TChannels::allOn() {
	bool relays[NBREL] = { 1,1,1,1,1,1 };
	setAllRelays(relays);
}

// public
void TChannels::allOff() {
	bool relays[NBREL] = { 0,0,0,0,0,0 };
	setAllRelays(relays);
}

// public
void TChannels::setRelay(unsigned int channel, bool rel_state)
{
	if (channel >= NBREL) { // channel out of range
		Serial.print("[WARN]: Channel ");
		Serial.print(channel);
		Serial.println("does not exist!");
	}

	Rel_enabled[channel] = rel_state; // since this is a command from outside, we need to make this state permanent
	set1(channel, rel_state); // switch relay
	state = STATE_RST;  // trigger reset
}

// public
void TChannels::setAllRelays(bool* state)
{
	for (int i = 0; i < NBREL; i++) {
		setRelay(i, state[i]);
	}
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

void TChannels::autoMode(bool _reconnect_enabled, bool _keep_faulty_channels_off, int _aNewAutoRestartDelay_s, bool* _aListChannelsUsed) {

	//Serial.println("[INFO]: activating automode");

	auto_disconnect_enabled = true;
	auto_reconnect_enabled = _reconnect_enabled;
	keep_faulty_channels_off = _keep_faulty_channels_off;
	autoRestartDelay_s = _aNewAutoRestartDelay_s;

	// enable selected relays
	setAllRelays(_aListChannelsUsed);

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

void TChannels::printChannelsStatus(bool newLine) {
	Serial.print((int)Rel_status[0]);
	for (int i = 1; i < NBREL; i++) {
		Serial.print(",");
		Serial.print((int)Rel_status[i]);
	}
	if (newLine) {
		Serial.println("");
	}
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
		//Serial.println("[INFO]: Resetting relay state");

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
gTHB.ac_paused = false;  // make sure AC is not disabled

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
				gTDCDC.target_voltage_modifier = 0; // turn voltage off to prevent surge when disconnecting channels
				timer1 = millis();
				state = STATE_SHORT_DETECTED_PREVENT_SURGE;
			}
		}
		else {
			state = STATE_NORMAL; // auto disconnect is not enabled so we don't do anything and remain in normal state
			return;
		}

	}

	if (state == STATE_SHORT_DETECTED_PREVENT_SURGE) {
		// after test of one channel is done, make sure voltage is stable or we waited a moment at 0V before we switch the relays
		if ((millis() - timer1) > RELAUTO_SURGE_SUPPRESSION_TIME_MS) {
			state = STATE_SHORT_DETECTED; // back to waiting with relays off
		}
		else {
			// not yet stable -> stay in this state to keep waiting
			state = STATE_SHORT_DETECTED_PREVENT_SURGE;
		}
	}

	if (state == STATE_SHORT_DETECTED) {
		// disconnect everything, decrease voltage, cancel Switching

		// check which channels to test and clear previous test results
		// must be done before we switch the relays off since we might need to store the current state before it changes
		for (int i = 0; i < NBREL; i++) {
			if (keep_faulty_channels_off)  // only test channels which were previously on
				Channels_to_test[i] = Rel_status[i];
			else  // always test all channels that were enabled initially
				Channels_to_test[i] = Rel_enabled[i];
			Rel_test_result[i] = 0; // reset test results
		}

		//  short outputs to GND and turn AC off
		// this is done first to make sure samples are not exposed to a voltage spike when all loads are suddnely disconnected
		// (relays are leaky so exposure to high voltage could occur even when relays are open)
		save_hb_state = gTHB.getState();
		save_oc_state = gTOC.getState();
		gTOC.forceState(0); // switch off (short output to GND) immediately
		gTHB.forceState(0); // switch off (short output to GND) immediately
		gTOC.ac_paused = true;
		gTHB.ac_paused = true;

		allOff(); // disconnect all relays
		Serial.println("[INFO]: All channels disconnected!");

		short_detected = 1;

		if (auto_reconnect_enabled) {
			shortcircuit_finder_index = 0; // start testing from relay 0
			//Decrease voltage for testing
			gTDCDC.target_voltage_modifier = TEMP_DECREASE_MODIFIER;
			timer1 = millis();  // start timer for next state
			// go to next state
			state = STATE_SHORT_WAITING;
			Serial.println("[INFO]: Testing for short circuits...");
		}
		else {
			gTDCDC.target_voltage_modifier = TEMP_DECREASE_MODIFIER; // set back to normal (in case it was set to 0 for surge prevention)
			gTDCDC.shutdown();  // turn everything off
			state = STATE_NORMAL; // pretend everything is normal
		}
	}

	else if (state == STATE_SHORT_WAITING) {
		// with all relays off, wait until voltage is stable
		if (gTDCDC.is_voltage_stable()) {
			// voltage is stable, let's turn on the next relay and see if it remains stable

			// measure current (average of 10 measurements) in stable state with no load
			current_stable = 0;
			int ncur = 10;
			for (int i = 0; i < ncur; i++)
			{
				current_stable += gTDCDC.get_last_Inow();
			}
			current_stable /= ncur;

			// first, find out which one is the next enabled relay
			while (shortcircuit_finder_index < NBREL && !Channels_to_test[shortcircuit_finder_index])
				shortcircuit_finder_index++; // keep increasing if the current channel should not be tested so it will be skipped

			//Serial.print("[INFO]: Relays off, voltage stabilized -> switching on channel ");
			//Serial.println(shortcircuit_finder_index);

			if (shortcircuit_finder_index < NBREL) // next channel index is in range, so we're not done testing yet
			{
				// connect next relay and reconnect OC and HB
				set1(shortcircuit_finder_index, true);
				gTOC.forceState(1); // switch on to allow testing (doesn't need to be immediate so we can use normal state change)
				gTHB.forceState(1);
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

			if (v_stable) {
				//Serial.print("[INFO]: Voltage stabilized, channel ");
				//Serial.print(shortcircuit_finder_index);
				//Serial.println(" is OK");

				// measure current (average of 10 measurements) with one relay connected
				float current = 0;
				int ncur = 10;
				for (int i = 0; i < ncur; i++)
				{
					current += gTDCDC.get_last_Inow();
				}
				current /= ncur;
				// check if the current increase is above a certain threshold (indicative of a short without detectable voltage drop)
				if (current / current_stable > CURRENT_INCREASE_THRESH)
				{
					Rel_test_result[shortcircuit_finder_index] = false; // store whether or not this channel was fine
				}
				else
				{
					Rel_test_result[shortcircuit_finder_index] = true; // store whether or not this channel was fine
				}
			}
			else {
				//Serial.print("[INFO]: Voltage did not stabilize, channel ");
				//Serial.print(shortcircuit_finder_index);
				//Serial.println(" is faulty");
				Rel_test_result[shortcircuit_finder_index] = false; // store whether or not this channel was fine

				gTDCDC.target_voltage_modifier = 0; // turn voltage off briefly to avoid spike when we disconnect the channel
				gTDCDC.reset_stabilization_timer();
			}
			timer1 = millis();  // reset timer for next state
			state = STATE_SHORT_TESTING_PREVENT_SURGE;
		}
		else {
			// not yet stable -> stay in this state to keep waiting
			state = STATE_SHORT_TESTING;
		}
	}

	else if (state == STATE_SHORT_TESTING_PREVENT_SURGE) {
		// after test of one channel is done, make sure voltage is stable or we waited a moment at 0V before we switch the relays
		if (gTDCDC.is_voltage_stable() || (millis() - timer1) > RELAUTO_SURGE_SUPPRESSION_TIME_MS) {

			if (gTDCDC.target_voltage_modifier == 0) {
				gTDCDC.target_voltage_modifier = TEMP_DECREASE_MODIFIER; // turn on DCDC again to testing voltage
				gTDCDC.reset_stabilization_timer();
			}
			// short sample to GND before switching relay back off
			gTOC.forceState(0); // switch off (short output to GND) immediately
			gTHB.forceState(0); // switch off (short output to GND) immediately
			delay(10); // wait a few ms to make sure that OC and HB switch before the channel relay
			set1(shortcircuit_finder_index, false); // turn this channel off again

			shortcircuit_finder_index++; // move on to next channel

			timer1 = millis();  // reset timer for next state
			state = STATE_SHORT_WAITING; // back to waiting with relays off
		}
		else {
			// not yet stable -> stay in this state to keep waiting
			state = STATE_SHORT_TESTING_PREVENT_SURGE;
		}
	}

	else if (state == STATE_SHORT_TESTING_DONE) {
		// test is done -> reconnect good samples, restore the proper voltage and resume

		gTDCDC.restore_voltage();
		gTDCDC.reset_stabilization_timer();
		
		state = STATE_RESUME_OPERATION;
		
	}

	else if (state == STATE_RESUME_OPERATION) {
		// with voltage restored (still all relays off), wait until voltage is stable
		if (gTDCDC.is_voltage_stable()) {
			// voltage is stable, let's turn relays back on and resume operation
			set6(Rel_test_result);
			gTOC.forceState(save_oc_state); // switch back to previous state before the test
			gTHB.forceState(save_hb_state);

			short_detected = false; // short should have been dealt with
			timer1 = 0;  // reset timer so we don't immediatly detect another short circuit while the voltage is restored

			Serial.print("[INFO]: Test finished. Reconnecting channels: ");
			printChannelsStatus(true);

			// test finished so we can continue AC cycling (if enabled)
			gTOC.ac_paused = false;
			gTHB.ac_paused = false;
			gTDCDC.reset_stabilization_timer(); // reset this timer also (better do it explicitly because it might not happen if no relays get reconnected)

			state = STATE_NORMAL;
		}
	}


}
