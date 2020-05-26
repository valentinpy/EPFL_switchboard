#ifndef _THB_H
#define _THB_H

class THB
{
public:
	enum stateEnum { GND=0, HVA=1, HVB=2, HIGHZ = 3, DONTCARE };

	THB() = default;
	void setup();
	void run();
	void stateChange(stateEnum newState);
	uint8_t getState();

private:
	enum stateMachineEnum { reset, standby, disconnect, reconnect };
	stateMachineEnum stateMachine;

	const uint8_t HB_LINA_PIN = 16; // H-Bridge: low side, side A
	const uint8_t HB_LINB_PIN = 5; // H-Bridge: low side, side B
	const uint8_t HB_HINA_PIN = 17; // H-Bridge: high side, side A
	const uint8_t HB_HINB_PIN = 6; // H-Bridge: high side, side B
	const uint32_t REL_DELAY_MS = 10;

	// Could be changed by an array of bool
	bool lina;
	bool linb;
	bool hina;
	bool hinb;

	struct newStatetruct {
		bool stateChanged;
		stateEnum state;
	};
	struct newStatetruct newStateS = { false, GND };

	uint32_t timer;

	void internalRun(bool stateChange, stateEnum newState);


};
#endif
