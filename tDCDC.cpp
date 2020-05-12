#include "Arduino.h"
#include "include/tDCDC.h"


#define DCDC_CURRENT_FB_PIN 0 // Voltage representing current on the input of DCDC (=> analog input)
#define KILL_STATE_PIN 0 // Voltage for "kill switch" state. (analog input, due to voltage divider)
#define HV_FB_PIN 0 // Voltage representing high voltage output, voltage divider ratio: 1000
#define DCDC_CTRL_PIN 0 // Digital output pin for driving transistor of buck converter before HV DCDC


void TDCDC::setup(){

}

void TDCDC::run(){

}

void TDCDC::set_target_voltage(){

}


void TDCDC::get_measured_voltage(){

}

void TDCDC::get_target_voltage(){

}
