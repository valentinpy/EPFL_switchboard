#include "Arduino.h"
#include "include/tComm.h"
#include "include/mEEPROM.h"
#include <EEPROM.h>
#include "include/mSerialCommand.h"
#include "userdef.h"


#include "include/tComm.h"
#include "include/tDCDC.h"
#include "include/tOC.h"
#include "include/tHB.h"
#include "include/tChannels.h"
#include "include/tLed.h"
#include "include/mSerialCommand.h"


mSerialCommand sCmd;     // The demo SerialCommand object

// external instances of all tasks
extern TDCDC gTDCDC;
extern TOC gTOC;
extern THB gTHB;
extern TChannels gTChannels;
extern TLed gTLed;



void TComm::setup(){
  Serial.begin(115200);
  Serial.println("Switchboard V2");

  sCmd.addCommand("QVmax", this->QVmax);
  sCmd.addCommand("SVmax", this->SVmax);
  sCmd.addCommand("SVset", this->SVset);
  sCmd.addCommand("QVset", this->QVset);
  sCmd.addCommand("QVnow", this->QVnow);
  sCmd.addCommand("QName", this->QName);
  sCmd.addCommand("SName", this->SName);
  sCmd.addCommand("QMem", this->QMem);
  sCmd.addCommand("QVer", this->QVer);
  sCmd.addCommand("Conf", this->Conf);
  sCmd.addCommand("SC0", this->SC0);
  sCmd.addCommand("SC1", this->SC1);
  sCmd.addCommand("SC2", this->SC2);
  sCmd.addCommand("QC0", this->QC0);
  sCmd.addCommand("QC1", this->QC1);
  sCmd.addCommand("QC2", this->QC2);
  sCmd.addCommand("SKp", this->SKp);
  sCmd.addCommand("SKi", this->SKi);
  sCmd.addCommand("SKd", this->SKd);
  sCmd.addCommand("QKp", this->QKp);
  sCmd.addCommand("QKi", this->QKi);
  sCmd.addCommand("QKd", this->QKd);
  sCmd.addCommand("SRelOn", this->SRelOn);
  sCmd.addCommand("SRelOff", this->SRelOff);
  sCmd.addCommand("SRelAuto", this->SRelAuto);
  sCmd.addCommand("QRelState", this->QRelState);
  sCmd.addCommand("SOCon", this->SOCon);
  sCmd.addCommand("SOCoff", this->SOCoff);
  sCmd.addCommand("SOCF", this->SOCF);
  sCmd.addCommand("QOC", this->QOC);
  sCmd.addCommand("SHB", this->SHB);
  sCmd.addCommand("SHBF", this->SHBF);
  sCmd.addCommand("QHB", this->QHB);
  sCmd.setDefaultHandler(this->unrecognized); // Handler for command that isn't matched

}

void TComm::run(){
  sCmd.readSerial();     // We don't do much, just process serial commands
  return;
}

void TComm::QVmax(){
  unsigned int Vmax;
  EEPROM.get(MEEPROM::ADR_VMAX_2B, Vmax);
  Serial.println(Vmax);
}
void TComm::SVmax(){
    unsigned int val = (unsigned int)sCmd.parseLongArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_VMAX_2B, val);
    Serial.println("Saved - requires reboot");
}
void TComm::QVset(){
  unsigned int val;
  EEPROM.get(MEEPROM::ADR_VSET_2B, val);
  Serial.println(val);
}
void TComm::SVset(){
  unsigned int val = (unsigned int) sCmd.parseLongArg();
  Serial.println(val);
  EEPROM.put(MEEPROM::ADR_VSET_2B,val);
  Serial.println("Saved - requires reboot");
}
void TComm::QVnow(){
    Serial.println("Not implemented yet");
}
void TComm::QName(){
    char buff[21];
    MEEPROM::read_string(MEEPROM::ADR_NAME_STR, 21, buff);
    Serial.println(buff);
}
void TComm::SName(){
    char *buffptr;
    buffptr = sCmd.next();
    MEEPROM::update_string(MEEPROM::ADR_NAME_STR, 21, buffptr);
    Serial.println(buffptr);
}
void TComm::QMem(){
    Serial.println("Not implemented yet");
}
void TComm::QVer(){
    Serial.println(SOFTWARE_VERSION);
}
void TComm::Conf(){
    Serial.println("Not implemented yet");
}
void TComm::SC0(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_C0_DBL, val);
    gTDCDC.set_C0(val);
}
void TComm::SC1(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_C1_DBL, val);
    gTDCDC.set_C1(val);
}
void TComm::SC2(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_C2_DBL, val);
    gTDCDC.set_C2(val);
}
void TComm::QC0(){
    double val = gTDCDC.get_C0();
    Serial.println(val);

}
void TComm::QC1(){
    double val = gTDCDC.get_C1();
    Serial.println(val);
}
void TComm::QC2(){
    double val = gTDCDC.get_C2();
    Serial.println(val);
}
void TComm::SKp(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_KP_DBL, val);
    gTDCDC.set_Kp(val);
}
void TComm::SKi(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_KI_DBL, val);
    gTDCDC.set_Ki(val);
}
void TComm::SKd(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_KD_DBL, val);
    gTDCDC.set_Kd(val);
}
void TComm::QKp(){
    double val;
    EEPROM.get(MEEPROM::ADR_KP_DBL, val);
    Serial.println(val);
}
void TComm::QKi(){
    double val;
    EEPROM.get(MEEPROM::ADR_KI_DBL, val);
    Serial.println(val);
}
void TComm::QKd(){
    double val;
    EEPROM.get(MEEPROM::ADR_KD_DBL, val);
    Serial.println(val);
}
void TComm::SRelOn(){
    gTChannels.allOn();
    gTChannels.printChannelsStatus();

}
void TComm::SRelOff(){
    gTChannels.allOff();
    gTChannels.printChannelsStatus();
}
void TComm::SRelAuto(){

}
void TComm::QRelState(){
    gTChannels.printChannelsStatus();
}
void TComm::SOCon(){

}
void TComm::SOCoff(){
}
void TComm::SOCF(){

}
void TComm::QOC(){

}
void TComm::SHB(){
    uint8_t val = (uint8_t)sCmd.parseLongArg();
    Serial.println(val);
    gTHB.stateChange((THB::stateEnum)val);
}
void TComm::SHBF(){
    Serial.println("Not implemented yet");
}
void TComm::QHB(){
    Serial.println(gTHB.getState());
}

// This gets set as the default handler, and gets called when no other command matches.
void TComm::unrecognized(const char *command) {
  Serial.println("Err");
}