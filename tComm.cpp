#include "Arduino.h"
#include "include/tComm.h"
#include "include/mEEPROM.h"
#include <EEPROM.h>
#include "include/mSerialCommand.h"

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
  EEPROM.get(MEEPROM::ADR_VMAX, Vmax);
  Serial.println(Vmax);
}
void TComm::SVmax(){
    char *arg;
    arg = sCmd.next();
    if (arg == NULL) {
      Serial.println("Err");
      return;
    }
    else {
      Serial.println(arg);
      unsigned int val = atoi(arg); // TODO: to be changed to handle args which are not a number!
      EEPROM.put(MEEPROM::ADR_VMAX,val);
    }
}
void TComm::SVset(){

}
void TComm::QVset(){

}
void TComm::QVnow(){

}
void TComm::QName(){

}
void TComm::SName(){

}
void TComm::QMem(){

}
void TComm::QVer(){

}
void TComm::Conf(){

}
void TComm::SC0(){

}
void TComm::SC1(){

}
void TComm::SC2(){

}
void TComm::QC0(){

}
void TComm::QC1(){

}
void TComm::QC2(){

}
void TComm::SKp(){

}
void TComm::SKi(){

}
void TComm::SKd(){

}
void TComm::QKp(){

}
void TComm::QKi(){

}
void TComm::QKd(){

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

// void TComm::testCmd() {
//   char *arg;
//   arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
//   if (arg != NULL) {    // As long as it existed, take it
//     Serial.print("Tata ");
//     Serial.println(arg);
//   }
//   else {
//     Serial.println("Hello, whoever you are");
//   }
// }
