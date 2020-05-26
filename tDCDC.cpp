#include "Arduino.h"
#include "include/tDCDC.h"
#include "include/hPID_v1.h"
#include "include/mEEPROM.h"
#include "EEPROM.h"

//#define DCDC_CURRENT_FB_PIN 0 // Voltage representing current on the input of DCDC (=> analog input)
//#define KILL_STATE_PIN 0 // Voltage for "kill switch" state. (analog input, due to voltage divider)

static const uint8_t HV_FB_PIN = 18; // Voltage representing high voltage output, voltage divider ratio: 1000
static const uint8_t DCDC_CTRL_PIN = 13; // Digital output pin for driving transistor of buck converter before HV DCDC


TDCDC::TDCDC() : HVPS_PID(&input, &output, &setpoint, 0.02, 0.0, 0.0, DIRECT) {}

void TDCDC::setup(){
	pinMode(DCDC_CTRL_PIN, OUTPUT);
	pinMode(HV_FB_PIN, INPUT);

    lastRun = millis();

    EEPROM.get(MEEPROM::ADR_C0_DBL, C0);
    EEPROM.get(MEEPROM::ADR_C1_DBL, C1);
    EEPROM.get(MEEPROM::ADR_C2_DBL, C2);

    EEPROM.get(MEEPROM::ADR_KP_DBL, Kp);
    EEPROM.get(MEEPROM::ADR_KI_DBL, Ki);
    EEPROM.get(MEEPROM::ADR_KD_DBL, Kd);

    EEPROM.get(MEEPROM::ADR_VMAX_2B, Vmax);
  
    HVPS_PID.SetOutputLimits(0, 1023); //set the PID output to a 10bit value
    HVPS_PID.SetSampleTime(5); //PID output is updated every 5ms
    HVPS_PID.SetTunings(Kp, Ki, Kd); //set the regulators parameters with data read from EEPROM
    HVPS_PID.SetMode(AUTOMATIC);

    initPWM();

}

void TDCDC::run(){
    if (millis() - lastRun > PERIOD_MS) {
        lastRun = millis();
        //Serial.println(get_HV_voltage(20));
        setPWMDuty(100);
    }
}

void TDCDC::set_target_voltage(int voltage){
    ////TODO add regulation
    //setPWMDuty(voltage);
    //Serial.print("DBG: new duty cycle:");
    //Serial.println(voltage);
}


void TDCDC::get_measured_voltage(){

}

void TDCDC::get_target_voltage(){

}

double TDCDC::get_HV_voltage(uint8_t nAvg) {
    float input_V = 0;
    for (uint8_t i = 0; i < nAvg; i++) {
        input_V = input_V + analogRead(HV_FB_PIN);
    }
    input_V = input_V / nAvg; //average value
    // TODO VPY: add calibration factor!
    input_V = input_V * (float)Vmax / 1024.0; // conversion 10bit ADC => voltage 0..Vmax, assuming voltage divider ratio is 1:1000
    input_V = C2 * 1E-6 * pow(input_V, 2) + C1 * input_V + C0;
    return input_V;
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
    return Kd;
}
double TDCDC::get_Ki() {
    return Ki;
}
double TDCDC::get_Kd() {
    return Kd;
}
uint16_t TDCDC::get_last_Vnow() {
    return lastRun;
}
uint16_t TDCDC::get_Vset() {
    return Vset;
}
uint16_t TDCDC::get_Vmax() {
    return Vmax;
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
}
void TDCDC::set_Ki(double Ki) {
    this->Ki = Ki;
}
void TDCDC::set_Kd(double Kd) {
    this->Kd = Kd;
}
