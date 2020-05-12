/*Code for the switchboard boards of the Project derivated from Peta-pico-Voltron
Author: Valentin Py
E-mail: valentin.py@epfl.ch / valentin.py@gmail.com
Company: EPFL - LMTS
Date: 17.12.2019

source:petapicovoltron.com
Target platform: Arduino micro + switchboard v1.0

Copyright 2016-2019 Samuel Rosset - Valentin Py
Distributed under the terms of the GNU General Public License GNU GPLv3

This file is part of Switchboard.

shvps is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

shvps is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with shvps.  If not, see  <http://www.gnu.org/licenses/>

*/

#include <Wire.h> //I2C link is used to communicate between the master board and the slave boards (controlling the channel voltage and switching)

#include "include/mEEPROM.h"
#include "include/tComm.h"
#include "include/tDCDC.h"
#include "include/tOC.h"
#include "include/tHB.h"
#include "include/tChannels.h"
#include "include/tLed.h"

//#include "PID_v1.h" //PID library

//some constant definition
#define Ver 1 //defines the current firmware version. For ease of transfer by I2C, the slave firmware versions are numbered with consecutive integers.

//------------------------------------------------------
// Stupid global variables to remove here:
//------------------------------------------------------

// char gName[21]; //store the board name
//
// //serial link is used when programming the board to define parameters (slave address, maximal voltage, PID parameters, etc.) can also be used for debugging
// char gSerial_buffer[30]; //buffer when reading serial commands
// byte gSerial_buffer_i=0; //index used to store characters read on the serial link in the buffer
// bool gSerial_EOS=false; //is a complete command stored in buffer?
//
// word gVmax; //Maximum voltage of the board (depends on which DC/DC converter is mounted). Will be read from EEPROM
// word gVset; //the current setpoint of the board
// word gVset_arb; //the setpoint of the arbitrary waveform (will vary from 0 to Vset)
// word gVnow; //the actual voltage read on the ADC channel
// byte gSwSrc; //Switching source: 0-onboard timer, 1-external frequency control (via master board), 2-push button. will be read from EEPROM
// byte gSwMode; //Switching mode: 0-off, 1-DC, 2-Switching at defined frequency. 3-Arbitrary waveform. Only used when switching source (SwSrc) is 0
// byte gSwMode_save; //used to store the previous mode (useful when limited cycles to know if pressing the button means to restart in Switching or Waveform mode)
// byte gVMode; //Voltage control mode: 0-internal regulator, 1-External voltage control, 2-internal open loop
// double gC0,gC1,gC2; //calibration factors to correct output voltage reading to account for imprecisions of the resistor bridge.
// word gCycle_max=0; //Used to remain in switching mode for a precise number of cycles only (Cyles_max).
// word gCycle_counter=0;  //The current cycle number is stored into Cycles_counter. Currently on 16bits, so 2^16-1 cycles maximum
// bool gDebug=false; //when true values of the voltage regulator are streamed on the serial link.
// bool gSwState=false; //bool to store the state of the optocoupler. This is used to decide whether or not turn on the LED (if voltage >0 and output on (true)) and to remember the state in case button is in latch mode
// bool gLatchMode; //0: button acts as a push button: HV is on while button is pressed. 1: button acts as a switch with a latching action (press to change state)
// double gF; //frequency
// word gPrescaler; //internal clock prescaler. Optimal value chosen as function of frequency set point
// int gFreq_div; //additional software prescaler. Optimal value chosen as function of frequency set point. Must be even! frequency divider: additional prescaler to slow down the counter more than the maximal prescaler (1024) allows.
// byte gFreq_div_counter=0; //counter for the software prescaler. Only when this counter reaches the value of Freq_div is the code in the interrupt function executed
// double gInput, gOutput, gSetpoint; //3 parameters for PID regulator
// PID gHVPS_PID(&input,&output,&setpoint,0.02,0,0,DIRECT); //creates the PID with some default parameters that will be changed later.
// double gKd_Save; //because the derivative term is turned on/off depending on error, we save it in a global variable
// double gKi_Save; //because the integral term is turned on/off depending on setpoint and error, we save it in a global variable

MEEPROM gMEEPROM;
TComm gTComm;
TDCDC gTDCDC;
TOC gTOC;
THB gTHB;
TChannels gTChannels;
TLed gTLed;


//------------------------------------------------------
// Setup
//------------------------------------------------------
void setup() {
  gMEEPROM.setup();
  gTDCDC.setup();
  gTOC.setup();
  gTHB.setup();
  gTChannels.setup();
  gTLed.setup();

  delay(1000);
  gTComm.setup();
}

//------------------------------------------------------
// Big old loop
//------------------------------------------------------
void loop() {
  // gMEEPROM.run();
  gTComm.run();
  gTDCDC.run();
  gTOC.run();
  gTHB.run();
  gTChannels.run();
  gTLed.run();

  static unsigned int i = 0;
  gTChannels.set1(i, true);
  gTChannels.set1(i-1, false);
  i++;
  delay(5000);





}
