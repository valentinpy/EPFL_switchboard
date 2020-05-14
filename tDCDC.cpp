#include "Arduino.h"
#include "include/tDCDC.h"
#include "include/hPID_v1.h"
#include "include/mEEPROM.h"
#include "EEPROM.h"

#define DCDC_CURRENT_FB_PIN 0 // Voltage representing current on the input of DCDC (=> analog input)
#define KILL_STATE_PIN 0 // Voltage for "kill switch" state. (analog input, due to voltage divider)

static const byte HV_FB_PIN = 18; // Voltage representing high voltage output, voltage divider ratio: 1000
static const byte DCDC_CTRL_PIN = 13; // Digital output pin for driving transistor of buck converter before HV DCDC


TDCDC::TDCDC() : HVPS_PID(&input, &output, &setpoint, 0.02, 0.0, 0.0, DIRECT) {}

void TDCDC::setup(){
	pinMode(DCDC_CTRL_PIN, OUTPUT);
	pinMode(HV_FB_PIN, INPUT);

    EEPROM.get(MEEPROM::ADR_C0_DBL, C0);
    EEPROM.get(MEEPROM::ADR_C1_DBL, C1);
    EEPROM.get(MEEPROM::ADR_C2_DBL, C2);

    EEPROM.get(MEEPROM::ADR_KP_DBL, Kp);
    EEPROM.get(MEEPROM::ADR_KI_DBL, Ki);
    EEPROM.get(MEEPROM::ADR_KD_DBL, Kd);
  
    HVPS_PID.SetOutputLimits(0, 1023); //set the PID output to a 10bit value
    HVPS_PID.SetSampleTime(5); //PID output is updated every 5ms
    HVPS_PID.SetTunings(Kp, Ki, Kd); //set the regulators parameters with data read from EEPROM
    HVPS_PID.SetMode(AUTOMATIC);


}

void TDCDC::run(){

}

void TDCDC::set_target_voltage(){

}


void TDCDC::get_measured_voltage(){

}

void TDCDC::get_target_voltage(){

}
