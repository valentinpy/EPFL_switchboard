#include "Arduino.h"
#include "include/tDCDC.h"
#include "include/tChannels.h"
#include "include/hPID_v1.h"
#include "include/mEEPROM.h"
#include "EEPROM.h"
#include "include/hDebug.h"

//#define DCDC_CURRENT_FB_PIN 20 // Voltage representing current on the input of DCDC (=> analog input)
//#define KILL_STATE_PIN 0 // Voltage for "kill switch" state. (analog input, due to voltage divider)

static const uint8_t HV_FB_PIN = 18; // Voltage representing high voltage output, voltage divider ratio: 1000
static const uint8_t DCDC_CTRL_PIN = 13; // Digital output pin for driving transistor of buck converter before HV DCDC
static const uint8_t DCDC_CURRENT_FB_PIN = 20; // Voltage representing DCDC input current, gain: 2 V/A -> 1 V = 500 mA
extern HDBG gHDBG;
extern TChannels gTChannels;

TDCDC::TDCDC() : HVPS_PID(&input, &output, &setpoint, 0.02, 0.0, 0.0, DIRECT) {}

void TDCDC::setup(){
    // setup HW pins
  	pinMode(DCDC_CTRL_PIN, OUTPUT);
  	pinMode(HV_FB_PIN, INPUT);
    pinMode(DCDC_CURRENT_FB_PIN, INPUT);
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
    // Read, filter and convert voltage, update PWM output
    // This should run at a defined frequency to keep expected low pass frequency
    // See tDCDC.h

    // check kill switch
    if (millis() - timer_enable_switch >= KILL_SWITCH_PERIOD_MS) {  
        timer_enable_switch = millis();
        // TODO: can't we do this with a digital read? would be much faster, no?
        enable_switch = get_filtered_enable_switch(0.02) > 200 ? 1 : 0;
    }
    if (!enable_switch) {
        setpoint = 0.0; // if disabled, PID target => 0
    }
    else {
        setpoint = setpoint_save * target_voltage_modifier;  // always apply modifier to setpoint specified by user
    }
    
    // measure voltage
    if (millis() - timerHVmeas >= PERIOD_HVMEAS_MS) {
        timerHVmeas = millis();
        measure_HV_voltage_fast(HVMEAS_ALPHA);
    
        uint16_t vnow = get_last_Vnow(); // use getter to apply low voltage threshold

        // now that we have a new voltage measurement, let's update the PID
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

        // check if voltage is stable
        if (abs(vnow - setpoint) > V_STABLE_THRESHOLD) { // measured voltage is not in range
          
            if(voltage_stable && vnow < setpoint) { // voltage is below target now and it was previously stable --> voltage just dropped
                // immediately disconnect and start testing!
                Serial.println("[INFO]: Voltage drop detected!");
                voltage_drop_detected = true;
                gTChannels.voltage_drop_detected_callback();
            }
            timer_last_V_off_target = millis();  // mark the last time the voltage was not close to the setpoint
        }
    
        // check if voltage can be considered stable (last time it was off the target is sufficiently long ago)
        voltage_stable = millis() - timer_last_V_off_target > PERIOD_V_STABLE_MS;

        // measure current
        measure_current_fast(CURMEAS_ALPHA);
    }
}

void TDCDC::set_target_voltage(uint16_t voltage){
    //setpoint = voltage;
    if (voltage > Vmax){
        Serial.print("[WARN]: Target exceeds maximum voltage. Setting to maximum instead: ");
        Serial.print(Vmax);
        Serial.println(" V");
        voltage = Vmax;
    }
    setpoint_save = voltage;
    reset_stabilization_timer();
}

bool TDCDC::restore_voltage() {
    target_voltage_modifier = 1.0;  // reset modifier to 1 (apply full target voltage)
}

void TDCDC::measure_HV_voltage_fast(float alpha) {
    float x;
    static float y=0;
    float return_V;

    x = (float)analogRead(HV_FB_PIN); //TODO: can we do that reading non blocking: we lose at least 100uS!
    
    // Exponential smoothing:
    // https://en.wikipedia.org/wiki/Exponential_smoothing

    y = alpha * x + (1.0 - alpha) * y;
    


    // calibration factor
    // TODO: Why does the conversion depend on Vmax? surely, this should be Vin (5V) instead!
//    return_V = y * (float)Vmax / 1024.0; // conversion 10bit ADC => voltage 0..Vmax, assuming voltage divider ratio is 1:1000
    return_V = y * 5.0 / 1023.0 * 1000; // conversion 10bit ADC => voltage 0..5V, assuming voltage divider ratio is 1:1000
    return_V = C2 * 1E-6 * pow(return_V, 2) + C1 * return_V + C0;

//    if(setpoint == 0 && return_V < 100)
    if(return_V < 100)
      last_Vnow = 0;  // make sure it gets set to 0 and never ends up negative and causes overflow
    else
      last_Vnow = return_V; // store measured value
}

void TDCDC::measure_current_fast(float alpha) {
    float x;
    static float y=0;
    float v, cur_mA;

    x = (float)analogRead(DCDC_CURRENT_FB_PIN); //TODO: can we do that reading non blocking: we lose at least 100uS!
    
    // Exponential smoothing:
    // https://en.wikipedia.org/wiki/Exponential_smoothing
    y = alpha * x + (1.0 - alpha) * y;

    // calibration factor
    v = y * 5.0 / 1023.0; // conversion 10bit ADC => voltage (5V = 1023)
    cur_mA = v * 500; // convert measured voltage to current in mA
    last_Inow = cur_mA; // store measured value
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

void TDCDC::reset_stabilization_timer() {
    voltage_stable = false;
    timer_last_V_off_target = millis();
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
    last_PWM = duty;
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
uint16_t TDCDC::get_last_Inow() {
    return last_Inow;
}
uint16_t TDCDC::get_last_PWM() {
    return last_PWM;
}

bool TDCDC::is_voltage_stable() {
    // return true if the last time the voltage was not on target is sufficiently long ago
    return voltage_stable;
}

uint16_t TDCDC::get_Vset() {
    return setpoint_save;
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
