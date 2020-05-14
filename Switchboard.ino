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

#include "include/mEEPROM.h"
#include "include/tComm.h"
#include "include/tDCDC.h"
#include "include/tOC.h"
#include "include/tHB.h"
#include "include/tChannels.h"
#include "include/tLed.h"
#include "include/mSerialCommand.h"

#include "userdef.h"

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
  gTComm.setup();
}

//------------------------------------------------------
// Big old loop
//------------------------------------------------------
void loop() {
  gTComm.run();
  gTDCDC.run();
  gTOC.run();
  gTHB.run();
  gTChannels.run();
  gTLed.run();
}
