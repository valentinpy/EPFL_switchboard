#include "Arduino.h"
#include "include/tDCDC.h"
#include "include/tChannels.h"
#include "include/tOC.h"
#include "include/tHB.h"
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
extern TOC gTOC;
extern THB gTHB;

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

    // Min and max voltage
    EEPROM.get(MEEPROM::ADR_VMAX_2B, Vmax);
    EEPROM.get(MEEPROM::ADR_VMIN_2B, Vmin);
  
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
    enable_switch = read_enable_switch();

    if (!enable_switch) {
        setpoint = 0.0; // if disabled, PID target => 0
    }
    else {
        setpoint = setpoint_save * target_voltage_modifier;  // always apply modifier to setpoint specified by user
    }
    
    // measure voltage
    if (millis() - timerHVmeas >= PERIOD_HVMEAS_MS) {
        timerHVmeas = millis();
        last_Vnow = measure_HV_voltage_fast(HVMEAS_ALPHA);
    
        // now that we have a new voltage measurement, let's update the PID
        // Run PID: frequency is internally limited
        // HVPS_PID.Compute() returns true if a new value is computed
        // For PID: values are passed by pointers (TODO: change the lib?) for following variables:
        // - setpoint => target voltage
        // - input => measured voltage
        // - output => result of PID computation => PWM duty cycle
        if (setpoint == 0) { // if setpoint is 0, we ignore controller and set output off
            setPWMDuty(1023); // set to max value -> inverted PWM to 0 -> DCDC off

            if (output < 1023) {  // last output was not yet 0 V (1023)  -> let controller know it hasn't reached the set point yet by giving a fake input
                input = 100;
                HVPS_PID.Compute();  // update controler
            }
        }
        else {
            input = last_Vnow; // Tell PID what was the measured voltage
            bool newValue = HVPS_PID.Compute(); // update PID controller on every cycle when setpoint is > 0
            if (newValue)
                setPWMDuty(output); // Apply output if computed
            //Serial.print("in: ");
            //Serial.print(input);
            //Serial.print("\tsetpoint: ");
            //Serial.print(setpoint);
            //Serial.print("\tPWM out: ");
            //Serial.println(output);
        }

        // check if voltage is stable
        if (abs(last_Vnow - setpoint) > V_STABLE_THRESHOLD) { // measured voltage is not in range
            //Serial.print("setpoint: ");
            //Serial.print(setpoint);
            //Serial.print("; last_Vnow: ");
            //Serial.println(last_Vnow);

            if(voltage_stable && last_Vnow < setpoint && enable_switch) { // voltage is below target now and it was previously stable --> voltage just dropped
                // immediately disconnect and start testing!
                Serial.println("[INFO]: Voltage drop detected! [fast]");
                voltage_drop_detected = true;
                gTChannels.voltage_drop_detected_callback();
            }
            timer_last_V_off_target = millis();  // mark the last time the voltage was not close to the setpoint
        }
        
        // check if voltage can be considered stable (last time it was off the target is sufficiently long ago)
        if (setpoint > 0) {
            voltage_stable = millis() - timer_last_V_off_target > PERIOD_V_STABLE_MS;
        }
        else {
            voltage_stable = false; // can't be considered stable if output is off
        }

        // check for voltage drop during rise (before stable state was reached)
        if (!voltage_stable && !voltage_drop_detected) {  // if stable state has already been reached or we already detected a drop, we don't need to do this
            if (millis() - timer_last_state_change > STATE_CHANGE_COOLDOWN_MS) {  // last state change is sufficiently long ago that the voltage should no longer be dropping no matter what (unless there is a short) so we can engage voltage drop detection
                if (setpoint - last_Vnow > V_STABLE_THRESHOLD) {  // voltage must be below set point and outside the stable range to be considered a voltage drop
                    if (prev_Vnow > -1) {  // valid previous voltage available
                        if (prev_Vnow - last_Vnow > V_STABLE_THRESHOLD) {  // if voltage is dropping (by more than a few volts)
                            Serial.println("[INFO]: Voltage drop detected! [during rise]");
                            voltage_drop_detected = true;
                            gTChannels.voltage_drop_detected_callback();
                        }
                    }
                }
                prev_Vnow = last_Vnow;
            }
        }
    
        
        if (long_shortCircuitProtection()) { // voltage has been low for a long time --> shut everything down to avoid damage to DCDC!
            Serial.println("[ERR]: Long voltage drop detected. Shutting down!");
            shutdown();
        }

    }
}

void TDCDC::shutdown()
{
    // DCDC off
    set_target_voltage(0);
    // set HB to GND
    gTHB.setOperationMode(THB::OPMANUAL);
    gTHB.stateChange(0);
    // set OC to GND
    gTOC.setOperationMode(TOC::OPMANUAL);
    gTOC.stateChange(0);
    // disconnect all channels
    gTChannels.allOff();
}

uint16_t TDCDC::set_target_voltage(uint16_t voltage){
    //setpoint = voltage;
    if (voltage > Vmax){
        Serial.print("[WARN]: Target exceeds maximum voltage. Setting to maximum instead: ");
        Serial.print(Vmax);
        Serial.println(" V");
        voltage = Vmax;
    }
    if (voltage > 0 && voltage < Vmin) {
        Serial.println("[WARN]: Target is below minimum voltage. Setting to 0 V instead.");
        voltage = 0;
    }
    if (voltage != setpoint_save) { // only need to apply change if new target is different
        setpoint_save = voltage;
        reset_stabilization_timer();
    }
    return voltage;
}

bool TDCDC::restore_voltage() {
    target_voltage_modifier = 1.0;  // reset modifier to 1 (apply full target voltage)
}

uint16_t TDCDC::measure_HV_voltage_fast(float alpha) {
    float x;
    static float y=0;
    float return_V;

    x = (float)analogRead(HV_FB_PIN); //TODO: can we do that reading non blocking: we lose at least 100uS!
    
    // Exponential smoothing:
    // https://en.wikipedia.org/wiki/Exponential_smoothing
    y = alpha * x + (1.0 - alpha) * y;
    
    // calibration factor
    return_V = y * 5.0 / 1023.0 * 1000; // conversion 10bit ADC => voltage 0..5V, assuming voltage divider ratio is 1:1000
    return_V = C2 * 1E-6 * pow(return_V, 2) + C1 * return_V + C0;

    if(return_V < 100)
        return_V = 0;  // make sure it gets set to 0 and never ends up negative and causes overflow
    
    return return_V;
}

uint16_t TDCDC::measure_current_fast(float alpha) {
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
    return cur_mA; // return measured value
}

bool TDCDC::read_enable_switch() {

    bool pin_state = digitalRead(KILL_SWITCH_PIN);

    // debounce: only change state if the pin has been in a differnt state for a certain amount of time
    if (pin_state != enable_switch) { // pin state has changed
        if (timer_enable_switch == 0) {
            timer_enable_switch = millis(); // start timer
        }
        else if (millis() - timer_enable_switch > KILL_SWITCH_PERIOD_MS) { // pin has been in that state for long enough (finished debouncing)
            enable_switch = pin_state; // apply new state
            if (enable_switch)
                Serial.println("[INFO]: Safety switch on. HV output enabled.");
            else
                Serial.println("[INFO]: Safety switch off. HV output disabled.");
            return pin_state;
        }
        return enable_switch; // we're not sure yet of the new state. return old state
    }
    else { // state has not changed
        timer_enable_switch = 0; // disable the timer
        return enable_switch; // return the current state, since it hasn't changed
    }

}

void TDCDC::reset_stabilization_timer() {
    voltage_stable = false;
    voltage_drop_detected = false;
    prev_Vnow = -1;
    timer_last_state_change = millis();
    timer_last_V_off_target = timer_last_state_change;
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
    return measure_current_fast(1);
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
uint16_t TDCDC::get_Vmin() {
    return Vmin;
}

void TDCDC::set_Vmax(uint16_t new_vmax)
{
    Vmax = new_vmax;
}

void TDCDC::set_Vmin(uint16_t new_vmin)
{
    Vmin = new_vmin;
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

bool TDCDC::long_shortCircuitProtection() {
    // This methods compares target voltage and measured voltage
    // If the measured voltage is too log for a defined time: LSCP_MAX_TIME_MS, the functions return true
    // To cancel the LSCP_MAX_TIME_MS timer, voltage has to be restored over threshold for at least LSCP_CANCEL_TIME_MS
    // Method is non blocking, and the called should read return value

    // Return value is true if a long short-circuit has been detected

    // detect begin of short circuit
    if (last_Vnow < (setpoint * LSCP_VOLTAGE_THRESHOLD_REL)) { // low voltage
        timer_lscp_VOK = 0; // voltage is not OK. Reset OK timer
        
        if (timer_lscp_Vlow == 0) { // Low voltage first time
            timer_lscp_Vlow = millis();  // start voltage low timer
            duration_voltage_low = 0;
        }
        else { // voltage low timer is running. check for how long...
            duration_voltage_low = millis() - timer_lscp_Vlow;
            if (duration_voltage_low > LSCP_MAX_TIME_MS) { // low for too long -> raise alarm!
                timer_lscp_Vlow = 0;
                return true; // return true to indicate that there is a short circuit!
            }
        }
    }
    else { // not low voltage anymore
        if (timer_lscp_VOK == 0) {
            timer_lscp_VOK = millis();  // start voltage OK timer
        }
        if ((millis() - timer_lscp_VOK) > LSCP_CANCEL_TIME_MS) {  // voltage has been OK for long enough
            timer_lscp_Vlow = 0; // we can reset the voltage low timer
            duration_voltage_low = 0;
        }
    }
    return false;
}

uint32_t TDCDC::get_duration_voltage_low()
{
    return duration_voltage_low;
}
