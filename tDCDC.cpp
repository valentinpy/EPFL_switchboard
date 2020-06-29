#include "Arduino.h"
#include "include/tDCDC.h"
#include "include/hPID_v1.h"
#include "include/mEEPROM.h"
#include "EEPROM.h"
#include "include/hDebug.h"

//#define DCDC_CURRENT_FB_PIN 0 // Voltage representing current on the input of DCDC (=> analog input)
//#define KILL_STATE_PIN 0 // Voltage for "kill switch" state. (analog input, due to voltage divider)

static const uint8_t HV_FB_PIN = 18; // Voltage representing high voltage output, voltage divider ratio: 1000
static const uint8_t CURRENT_FB_PIN = 20;
static const uint8_t DCDC_CTRL_PIN = 13; // Digital output pin for driving transistor of buck converter before HV DCDC
extern HDBG gHDBG;

TDCDC::TDCDC() : HVPS_PID(&input, &output, &setpoint, 0.02, 0.0, 0.0, DIRECT) {}

void TDCDC::setup(){
    // setup HW pins
	pinMode(DCDC_CTRL_PIN, OUTPUT);
	pinMode(HV_FB_PIN, INPUT);
    pinMode(KILL_SWITCH_PIN, INPUT);

    //-------------------------------------
    // Read important data from EEPROM
    //-------------------------------------

    // Calibration factors
    EEPROM.get(MEEPROM::ADR_C0_DBL, C0);
    EEPROM.get(MEEPROM::ADR_C1_DBL, C1);
    EEPROM.get(MEEPROM::ADR_C2_DBL, C2);

    // PID coeffircients
    EEPROM.get(MEEPROM::ADR_KP_DBL, Kp);
    EEPROM.get(MEEPROM::ADR_KI_DBL, Ki);
    EEPROM.get(MEEPROM::ADR_KD_DBL, Kd);

    // Max voltage
    EEPROM.get(MEEPROM::ADR_VMAX_2B, Vmax);
  
    //intial target voltage

    HVPS_PID.SetOutputLimits(0, 1023); //set the PID output to a 10bit value
    HVPS_PID.SetSampleTime(5); //PID output is updated every 5ms
    HVPS_PID.SetTunings(Kp, Ki, Kd); //set the regulators parameters with data read from EEPROM
    HVPS_PID.SetMode(AUTOMATIC);
    HVPS_PID.SetControllerDirection(REVERSE);

    //setup PWM
    //TODO: add a general PWM Class, with:
    // - setup function(pin, frequency, initDutyCycle=0)
    // - updateDutyCycle(newDutyCycle)
    // - updateFrequency(neFFrequency)
    // - getDutyCycle()
    initPWM();

    setpoint = 0;
    setpoint_save = 0;

    //initialize timer
    timer = millis();
}

void TDCDC::run(){
    // Read and filter and convert voltage
    // This should run at a defined frequency to keep expected low pass frequency
    // See tDCDC.h
    if (millis() - timerHVmeas >= PERIOD_HVMEAS_MS) {
        timerHVmeas = millis();
        last_Vnow = get_HV_voltage_fast(HVMEAS_ALPHA);
        last_Vcurrent = get_VCurrent_fast(HVMEAS_ALPHA);
    }

    if (millis() - timer_enable_switch >= KILL_SWITCH_PERIOD_MS) {  
        timer_enable_switch = millis();
        //enable_switch = analogRead(KILL_SWITCH_PIN) > 200 ? 1 : 0;
        enable_switch = get_filtered_enable_switch(0.02) > 200 ? 1 : 0;
        
        // if disabled, PID target => 0
        if (!enable_switch) {
            //setpoint_save = setpoint;
            setpoint = 0.0;
        }
        else {
            setpoint = setpoint_save;
        }
    }

    
    // Run PID: frequency is internally limited
    // HVPS_PID.Compute() returns true if a new value is computed
    // For PID: values are passed by pointers (TODO: change the lib?) for following variables:
    // - setpoint => target voltage
    // - input => measured voltage
    // - output => result of PID computation => PWM duty cycle

    input = last_Vnow; // Tell PID what was the measured voltage
    if (HVPS_PID.Compute()) {
        setPWMDuty(output); // Apply output if computed
        //Serial.print("in: ");
        //Serial.print(input);
        //Serial.print("\tsetpoint: ");
        //Serial.print(setpoint);
        //Serial.print("\tPWM out: ");
        //Serial.println(output);
    }
}

void TDCDC::set_target_voltage(uint16_t voltage){
    //setpoint = voltage;
    setpoint_save = voltage;
}

double TDCDC::get_HV_voltage(uint8_t nAvg) { //deprecated
    float input_V = 0;
    for (uint8_t i = 0; i < nAvg; i++) {
        input_V = input_V + analogRead(HV_FB_PIN);
    }
    input_V = input_V / nAvg; //average value
    
    input_V = input_V * (float)Vmax / 1024.0; // conversion 10bit ADC => voltage 0..Vmax, assuming voltage divider ratio is 1:1000
    input_V = C2 * 1E-6 * pow(input_V, 2) + C1 * input_V + C0;
    return input_V;
}

bool TDCDC::decrease_temporary_voltage(uint8_t percentage) {
    if (old_voltage != -1) {
        // can only be applied once, please restore voltage first
        return false;
    }
    else {
        uint32_t new_voltage = (((uint32_t)get_Vset()) * percentage) / 100;
        old_voltage = get_Vset();
        //Serial.print("[INFO]: Decrease: new voltage: ");
        //Serial.println(new_voltage);
        set_target_voltage(new_voltage);
        return true;
    } 
}

bool TDCDC::restore_voltage() {
    if (old_voltage == -1) {
        // can only be applied once and after voltage has been decreased, please decrease voltage first
        return false;
    }
    else {
        uint32_t new_voltage = old_voltage;
        //Serial.print("[INFO]: Increase: new voltage: ");
        //Serial.println(new_voltage);
        set_target_voltage(new_voltage);
        old_voltage = -1;
        return true;
    }
}


double TDCDC::get_HV_voltage_fast(float alpha) {
    float x;
    static float y=0;
    float return_V;

    // Exponential smoothing:
    // https://en.wikipedia.org/wiki/Exponential_smoothing

    x = (double)analogRead(HV_FB_PIN); //TODO: can we do that reading non blocking: we lose at least 100uS!
    y = alpha * x + (1.0 - alpha) * y;
    

    // calibration factor
    return_V = y * (float)Vmax / 1024.0; // conversion 10bit ADC => voltage 0..Vmax, assuming voltage divider ratio is 1:1000
    return_V = C2 * 1E-6 * pow(return_V, 2) + C1 * return_V + C0;
    return return_V;
}

double TDCDC::get_VCurrent_fast(float alpha) {
    float x;
    static float y = 0;
    float return_V;

    // Exponential smoothing:
    // https://en.wikipedia.org/wiki/Exponential_smoothing

    x = (double)analogRead(CURRENT_FB_PIN); //TODO: can we do that reading non blocking: we lose at least 100uS!
    y = alpha * x + (1.0 - alpha) * y;


    // calibration factor
    return_V = y * (float)5 / 1024.0; // conversion 10bit ADC => voltage 0..5
    return return_V;
}

double TDCDC::get_filtered_enable_switch(float alpha) {
    float x;
    static float y = 0;

    // Exponential smoothing:
    // https://en.wikipedia.org/wiki/Exponential_smoothing

    x = (double)analogRead(KILL_SWITCH_PIN); //TODO: can we do that reading non blocking: we lose at least 100uS!
    y = alpha * x + (1.0 - alpha) * y;
    return y;
}


void TDCDC::initPWM() {
    //Setup of 10bit 7.8kHz PWM on pin 13 (connected to OCR4A)
    // 10-bit operation
    TC4H = 0x03;
    OCR4C = 0xFF; //The OCR4C holds the Timer/Counter TOP value, i.e. the clear on compare match value

    //Configuration of Timer 4 Registers, OC4A (D13): Clear on compare match
    TCCR4A = 0b10000010;
    //Prescaler
    TCCR4B = 0b00000001; //no prescaler (frequency of signal is 16E6/2/1024=7.8 kHz).
}

void TDCDC::setPWMDuty(uint16_t duty) {
    //duty is a 10bit value
    TC4H = duty >> 8;
    OCR4A = 0xFF & duty;
}

//---------------------------------
// getters
//---------------------------------

double TDCDC::get_C0() {
    return C0;
}
double TDCDC::get_C1() {
    return C1;
}
double TDCDC::get_C2() {
    return C2;
}
double TDCDC::get_Kp() {
    return Kp;
}
double TDCDC::get_Ki() {
    return Ki;
}
double TDCDC::get_Kd() {
    return Kd;
}
uint16_t TDCDC::get_last_Vnow() {
    return last_Vnow;
}
uint16_t TDCDC::get_last_Vcurrent() {
    return last_Vcurrent;
}
uint16_t TDCDC::get_Vset() {
    return old_voltage != -1 ? old_voltage : setpoint_save;
}
uint16_t TDCDC::get_Vmax() {
    return Vmax;
}

int16_t TDCDC::get_Verror_percent() {
    int32_t Vset = (int16_t)setpoint;
    int32_t Vnow = (int16_t)last_Vnow;
    int32_t error = ((Vnow - Vset) * 100) / Vset;
    /*Serial.print(Vset);
    Serial.print("\t");
    Serial.print(Vnow);
    Serial.print("\t");
    Serial.println(error);*/
    return error;
}

bool TDCDC::get_enable_switch() {
    return enable_switch;
}

//---------------------------------
// setters
//---------------------------------
void TDCDC::set_C0(double C0) {
    this->C0 = C0;
}
void TDCDC::set_C1(double C1) {
    this->C1 = C1;
}
void TDCDC::set_C2(double C2) {
    this->C2 = C2;
}
void TDCDC::set_Kp(double Kp) {
    this->Kp = Kp;
    HVPS_PID.SetTunings(Kp, Ki, Kd);
}
void TDCDC::set_Ki(double Ki) {
    this->Ki = Ki;
    HVPS_PID.SetTunings(Kp, Ki, Kd);
}
void TDCDC::set_Kd(double Kd) {
    this->Kd = Kd;
    HVPS_PID.SetTunings(Kp, Ki, Kd);
}
