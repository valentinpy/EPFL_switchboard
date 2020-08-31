#ifndef _TCHANNELS_H
#define _TCHANNELS_H

class TChannels
{
public:
	enum autoModeEnum { Manual, AutoDisconnect, AutoReconnect };

	TChannels() = default;
	void setup();
	void run();

	void allOn();
	void allOff();
	bool setRelay(unsigned int channel, bool state);
	bool setAllRelays(bool* state);
	void setAutoRestartDelay(int16_t delay);
	void reset();

	void autoMode(int aNewAutoRestartDelay_s, bool reconnect_enabled, bool* aListChannelsUsed);
	void getChannelsStatus(bool* retVal);
	void printChannelsStatus();
	bool isTestingShort();
	bool isShortDetected();
	void voltage_drop_detected_callback();

	bool auto_disconnect_enabled;
	bool auto_reconnect_enabled;

private:
	const int Rel0_PIN = 23;
	const int Rel1_PIN = 22;
	const int Rel2_PIN = 21;
	const int Rel3_PIN = 12;
	const int Rel4_PIN = 8;
	const int Rel5_PIN = 7;

	const int Rel_pin[6] = { Rel0_PIN, Rel1_PIN, Rel2_PIN, Rel3_PIN, Rel4_PIN, Rel5_PIN }; // digital pin used to control relay 0-5

	int Rel_status[6] = {};
	bool Rel_enabled[6] = {};
	bool RelState_testing[6] = { 0,0,0,0,0,0 };
	const byte NBREL = 6;
	bool set1(unsigned int channel, bool state);
	bool set6(bool* state);

	bool short_detected;
	
	int16_t autoRestartDelay_s;
	enum autoModeStateEnum {
		STATE_RST,
		STATE_NORMAL,
		STATE_SHORT_DETECTED,
		STATE_SHORT_WAITING,
		STATE_SHORT_TESTING,
		STATE_SHORT_TESTING_DONE,
		STATE_ERR
	};
	autoModeStateEnum state;

	uint32_t timer_relretry;
	uint32_t timer1;

	uint32_t supplementary_delay_ms = 0;
	int8_t shortcircuit_finder_index;
	const double TEMP_DECREASE_MODIFIER = 0.8;  // by how much to reduce target voltage during short circuit testing
	const uint16_t RELAUTO_LOW_VOLTAGE_TIME_THRESH_MS = 1000; // Minimum time [ms] that voltage needs to be low (as determined by tDCDC) for a short circuit to be detected
	const uint16_t RELAUTO_TESTING_TIME_MS = 1500; // Test duration before declaring a faulty sample
	const uint16_t RELAUTO_WAITING_VOTLAGE_REG_TIME_MS = 3000; // Max time to wait to wait for voltage to stabilize when no samples are connected
	const uint16_t RELAUTO_REL_TIME_MS = 200;

	uint32_t main_timer;
	const uint16_t MAIN_DELAY_MS = 50;
};
#endif
