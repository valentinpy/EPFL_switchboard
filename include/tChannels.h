#ifndef _TCHANNELS_H
#define _TCHANNELS_H

class TChannels
{
public:
	enum autoModeEnum { AllOff, AllOn, Manual, AutoMode };

	TChannels() = default;
	void setup();
	void run();

	void allOn();
	void allOff();
	bool set1(unsigned int channel, bool state);

	void autoMode(int aNewAutoRestartDelay_s, bool* aListChannelsUsed);
	void getChannelsStatus(bool* retVal);
	void printChannelsStatus();
	void setCurrentMode(autoModeEnum newCurrentMode);
	int8_t isTestingShort();

private:
	const int Rel0_PIN = 23;
	const int Rel1_PIN = 22;
	const int Rel2_PIN = 21;
	const int Rel3_PIN = 12;
	const int Rel4_PIN = 8;
	const int Rel5_PIN = 7;

	const int Rel_pin[6] = { Rel0_PIN, Rel1_PIN, Rel2_PIN, Rel3_PIN, Rel4_PIN, Rel5_PIN }; // digital pin used to control relay 0-5

	int Rel_status[6] = {};
	bool Rel_automode_enabled[6] = {};
	bool RelState_testing[6] = { 0,0,0,0,0,0 };
	const byte NBREL = 6;

	autoModeEnum currentMode;
	uint8_t testingShort;


	bool set6(bool* state);


	bool relay_state_machine();
	int16_t autoRestartDelay_s;
	enum autoModeStateEnum {
		AUTOSTATE_OFF,
		AUTOSTATE_RST,
		AUTOSTATE_NORMAL,
		AUTOSTATE_CONFIRMED_SHORT_INIT,
		AUTOSTATE_CONFIRMED_SHORT_WAITING_DECO,
		AUTOSTATE_CONFIRMED_SHORT_START_SEARCHING,
		AUTOSTATE_CONFIRMED_SHORT_SEARCHING_1,
		AUTOSTATE_CONFIRMED_SHORT_SEARCHING_2,
		AUTOSTATE_CONFIRMED_SHORT_SEARCHING_3,
		AUTOSTATE_CONFIRMED_SHORT_SEARCHING_4,
		AUTOSTATE_CONFIRMED_SHORT_SEARCHING_5
	};
	autoModeStateEnum autoModeState;

	uint32_t timer_relretry;
	uint32_t timer1;

	uint32_t supplementary_delay_ms = 0;
	int8_t shortcircuit_finder_index;
	const uint8_t THRESHOLD_PERCENT = 60; // VP TODO: This should be 80% of the target voltage, taking into account the 80% temp decrase during testing
	const uint8_t TEMP_DECREASE = 80;
	const uint16_t RELAUTO_MIN_LOW_VOLTAGE_TIME_MS = 200; // Minimum time [ms] for a short circuit to be detected (avoid trigger when voltage target increases)
	const uint16_t RELAUTO_TESTING_TIME_MS = 500; // Time for testing disconnexion
	const uint16_t RELAUTO_WAITING_VOTLAGE_REG_TIME_MS = 500; // Time to wait to wait for voltage to change
	const uint16_t RELAUTO_REL_TIME_MS = 200;

	uint32_t main_timer;
	const uint16_t MAIN_DELAY_MS = 50;
};
#endif
