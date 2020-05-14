#include "Arduino.h"
#include "include/tComm.h"
#include "include/mEEPROM.h"
#include <EEPROM.h>
#include "include/mSerialCommand.h"
#include "userdef.h"



mSerialCommand sCmd;     // The demo SerialCommand object

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
  sCmd.addCommand("SHBp", this->SHBp);
  sCmd.addCommand("SHBm", this->SHBm);
  sCmd.addCommand("SHB0", this->SHB0);
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
    Serial.println("Not implemented yet");
}
void TComm::SName(){
    Serial.println("Not implemented yet");
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
    Serial.println("Saved - requires reboot");
}
void TComm::SC1(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_C1_DBL, val);
    Serial.println("Saved - requires reboot");
}
void TComm::SC2(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_C2_DBL, val);
    Serial.println("Saved - requires reboot");
}
void TComm::QC0(){
    double val;
    EEPROM.get(MEEPROM::ADR_C0_DBL, val);
    Serial.println(val);
}
void TComm::QC1(){
    double val;
    EEPROM.get(MEEPROM::ADR_C1_DBL, val);
    Serial.println(val);
}
void TComm::QC2(){
    double val;
    EEPROM.get(MEEPROM::ADR_C2_DBL, val);
    Serial.println(val);
}
void TComm::SKp(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_KP_DBL, val);
    Serial.println("Saved - requires reboot");
}
void TComm::SKi(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_KI_DBL, val);
    Serial.println("Saved - requires reboot");
}
void TComm::SKd(){
    double val = (double)sCmd.parseDoubleArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_KD_DBL, val);
    Serial.println("Saved - requires reboot");
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

}
void TComm::SRelOff(){

}
void TComm::SRelAuto(){

}
void TComm::QRelState(){

}
void TComm::SOCon(){

}
void TComm::SOCoff(){
}
void TComm::SOCF(){

}
void TComm::QOC(){

}
void TComm::SHBp(){

}
void TComm::SHBm(){

}
void TComm::SHB0(){

}
void TComm::QHB(){

}

// This gets set as the default handler, and gets called when no other command matches.
void TComm::unrecognized(const char *command) {
  Serial.println("Err");
}