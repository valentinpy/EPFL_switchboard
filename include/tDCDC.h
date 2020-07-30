#ifndef _TDCDC_H
#define _TDCDC_H

#include "hPID_v1.h"

class TDCDC
{
public:
	TDCDC();
	void setup();
	void run();

	bool restore_voltage();  // reset the voltage modifier to 1.0 so the target voltage is the one set by the user
	
	double target_voltage_modifier = 1.0;  // a modifier that will be applied to the set point (to decrease voltage temporarily)
	

	//------------------------------------------------------
	// Getters
	//------------------------------------------------------
	double get_C0();
	double get_C1();
	double get_C2();

	double get_Kp();
	double get_Ki();
	double get_Kd();

	uint16_t get_last_Vnow();
	uint16_t get_last_PWM();
	uint16_t get_last_Inow();
	uint16_t get_Vset();
	uint16_t get_Vmax();
	int16_t get_Verror_percent();

	bool get_enable_switch();
	
	bool is_voltage_stable();
	

	//------------------------------------------------------
	// Setters
	//------------------------------------------------------
	void set_C0(double C0);
	void set_C1(double C1);
	void set_C2(double C2);
	void set_Kp(double Kp);
	void set_Ki(double Ki);
	void set_Kd(double Kd);

	void set_target_voltage(uint16_t  voltage);
	void reset_stabilization_timer();

private:
	//------------------------------------------------------
	// High voltage feebdback measurment
	//------------------------------------------------------

	// Filtering:
	//According to: https://en.wikipedia.org/wiki/Exponential_smoothing
	// Let T be sampling frequency
	// Let tau be expected time constant of filter
	// Assuming tau >> T
	// alpha = T/tau
	// T = 1ms, alpha = 0.1 => tau = 10ms
	uint32_t timerHVmeas; //timer for high voltage feedback filtering
	const uint32_t PERIOD_HVMEAS_MS = 1; //sampling for high voltage feedback filtering
	const float HVMEAS_ALPHA = 0.1; //alpha constant for high voltage feedback filtering
	const float CURMEAS_ALPHA = 0.1; //alpha constant for current feedback filtering (1 = no filtering)
	const uint32_t PERIOD_V_STABLE_MS = 500; // how long voltage needs to be on target to be considered "stable"
	const uint16_t V_STABLE_THRESHOLD = 20; // how close voltage needs to be to set point to be considered on target
	const uint16_t V_MIN = 500; // minimum allowed target voltage


	double C0;
	double C1;
	double C2;
	uint16_t last_Vnow;
	uint16_t last_PWM;
	uint16_t last_Inow;
	uint32_t timer_last_V_off_target;
	bool voltage_stable = false;
	bool voltage_drop_detected = false;
	
	void measure_HV_voltage_fast(float alpha);
	double get_filtered_enable_switch(float alpha);
	void measure_current_fast(float alpha);


	//------------------------------------------------------
	// PID stuff
	//------------------------------------------------------
	double Kp;
	double Ki;
	double Kd;
	double input, output, setpoint; //3 parameters for PID regulator
	double setpoint_save;
	PID HVPS_PID;


	//------------------------------------------------------
	// PWM
	//------------------------------------------------------
	void initPWM();
	void setPWMDuty(uint16_t duty);

	//------------------------------------------------------
	// Kill switch
	//------------------------------------------------------
	const uint8_t KILL_SWITCH_PIN = 19; //A1
	const uint32_t KILL_SWITCH_PERIOD_MS = 10;
	bool enable_switch;
	uint32_t timer_enable_switch; //timer for high voltage feedback filtering

	//------------------------------------------------------
	// Misc.
	//------------------------------------------------------
	uint32_t timer;
	const uint32_t PERIOD_MS = 1;
	uint16_t Vmax;
};
#endif
