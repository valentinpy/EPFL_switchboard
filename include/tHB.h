#ifndef _THB_H
#define _THB_H

class THB
{
public:
	enum operationModeEnum { OPMANUAL = 0, OPFREQUENCY = 1 };
	operationModeEnum operationMode;

	THB() = default;
	void setup();
	void run();
	void stateChange(uint8_t newState);

	uint8_t getState();
	uint8_t getOperationMode();
	uint16_t getMaxFrequencyHz();

	void setOperationMode(operationModeEnum newOpMode, double newFrequency = 0);
	

private:
	// pins definition
	const uint8_t HB_LINA_PIN = 16; // H-Bridge: low side, side A
	const uint8_t HB_LINB_PIN = 5; // H-Bridge: low side, side B
	const uint8_t HB_HINA_PIN = 17; // H-Bridge: high side, side A
	const uint8_t HB_HINB_PIN = 6; // H-Bridge: high side, side B

	// timing constants
	const uint32_t REL_DELAY_MS = 10;
	const uint16_t MAXFREQUENCY_HZ = 100; //TODO: Test and change, especially if implementing hard PWM

	// state machine enum + var for transition without short-circuit
	enum stateMachineEnum { reset, standby, disconnect, reconnect };
	stateMachineEnum stateMachine;

	// enum of possible states
	enum stateEnum { GND = 0, HVA = 1, HVB = 2, HIGHZ = 3, DONTCARE };

	// struct used for state machine when a new state change is requested
	struct newStatetruct {
		bool stateChanged;
		stateEnum state;
	};
	struct newStatetruct newStateS = { false, GND };

	// pins expected output for requested state
	// TODO: Could be changed by an array of bool
	bool lina;
	bool linb;
	bool hina;
	bool hinb;
	
	uint32_t period_us; // computed (half)-period (us)
	uint8_t frequency_toggler; //variable to switch states

	// Timers
	uint32_t timer;
	uint32_t timer_freq_us; // for frequency mode only

	// switching state machine
	void internalRun(bool stateChange, stateEnum newState);
};
#endif
