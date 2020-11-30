#ifndef _TOC_H
#define _TOC_H

class TOC
{
public:
	enum operationModeEnum { OPMANUAL = 0, OPFREQUENCY = 1 };
	operationModeEnum operationMode;

	TOC() = default;
	void setup();
	void run();
	void stateChange(uint8_t newState);

	void forceState(uint8_t newState);

	uint8_t getState();
	uint8_t getOperationMode();
	uint16_t getMaxFrequencyHz();

	void setOperationMode(operationModeEnum newOpMode, double newFrequency=0);
	bool ac_paused = false;  // flag to indicate if AC should be paused (for short circuit testing)
	
	
private:
	// pins definition
	const uint8_t OC_H_PIN = 28; // OC: high side
	const uint8_t OC_L_PIN = 27; // OC: low side

	// timing constants
	const uint32_t OC_DELAY_MS = 0; // OCs switch fast enough, we don't need a delay
	const uint16_t MAXFREQUENCY_HZ = 100; //TODO: Test and change, especially if implementing hard PWM

	// state machine enum + var for transition without short-circuit
	enum stateMachineEnum { reset, standby, disconnect, reconnect };
	stateMachineEnum stateMachine;

	// enum of possible states
	enum stateEnum { GND = 0, HV = 1, HIGHZ = 3, DONTCARE };
	stateEnum currentState = GND;

	// struct used for state machine when a new state change is requested
	struct newStatetruct {
		bool stateChanged;
		stateEnum state;
	};
	struct newStatetruct newStateS = { false, GND };

	// pins expected output for requested state
	// TODO: Could be changed by an array of bool
	bool lin;
	bool hin;

	// Frequency mode variables
	uint32_t period_us; // computed (half)-period (us)
	uint8_t frequency_toggler; //variable to switch states
	
	//Timers
	uint32_t timer;
	uint32_t timer_freq_us; // for frequency mode only

	// switching state machine
	void internalRun(bool stateChange, stateEnum newState);
};
#endif
