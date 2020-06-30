#include "Arduino.h"
#include "include/tComm.h"
#include "include/mEEPROM.h"
#include <EEPROM.h>
#include <avr/wdt.h>
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

bool TComm::debugEnabled = false;
uint32_t TComm::timer = 0;

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
  sCmd.addCommand("QTestingShort", this->QTestingShort);
  sCmd.addCommand("QCurrent", this->QCurrent);
  sCmd.addCommand("SOC", this->SOC);
  sCmd.addCommand("SOCF", this->SOCF);
  sCmd.addCommand("QOC", this->QOC);
  sCmd.addCommand("SHB", this->SHB);
  sCmd.addCommand("SHBF", this->SHBF);
  sCmd.addCommand("QHB", this->QHB);
  sCmd.addCommand("QEnable", this->QEnable);
  sCmd.addCommand("Reboot", this->Reboot);
  sCmd.addCommand("Debug", this->Debug);
  sCmd.addCommand("vpy", this->vpy); // vpy testing command
  sCmd.setDefaultHandler(this->unrecognized); // Handler for command that isn't matched

  TComm::debugEnabled = false;
  TComm::timer = millis();
}

void TComm::run(){
  sCmd.readSerial();     // We don't do much, just process serial commands
  if ((millis() - TComm::timer) > TComm::DELAY_MS) {
      TComm::timer = millis();
      if (TComm::debugEnabled) {
          debugPrint();
      }
  }
  return;
}

void TComm::debugPrint() {
    Serial.print(gTDCDC.get_last_Vnow());
    Serial.println("");
}

void TComm::QVmax(){
  unsigned int val;
  val = gTDCDC.get_Vmax();
  Serial.println(val);
}
void TComm::SVmax(){
    unsigned int val = (unsigned int)sCmd.parseLongArg();
    Serial.println(val);
    EEPROM.put(MEEPROM::ADR_VMAX_2B, val);
    Serial.println("Saved - requires reboot");
}
void TComm::QVset(){
  unsigned int val;
  val = gTDCDC.get_Vset();
  Serial.println(val);
}
void TComm::SVset(){
  unsigned int val = (unsigned int) sCmd.parseLongArg();
  Serial.println(val);
  gTDCDC.set_target_voltage(val);
  //EEPROM.put(MEEPROM::ADR_VSET_2B,val);
  //Serial.println("Saved - requires reboot");
  
}
void TComm::QVnow(){
    // If Vset = 0, measured voltage can be non 0 due to noise,... 
    if ((gTDCDC.get_Vset() == 0 || gTDCDC.get_enable_switch() == 0) && (gTDCDC.get_last_Vnow() < 100)) {
        Serial.println("0");
    }
    else {
        Serial.println((int16_t)gTDCDC.get_last_Vnow());
    }
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
    Serial.println("Not fully implemented yet");

    EEPROM.put(MEEPROM::ADR_C0_DBL, (double)0.0);
    EEPROM.put(MEEPROM::ADR_C1_DBL, (double)1.0);
    EEPROM.put(MEEPROM::ADR_C2_DBL, (double)0.0);

    EEPROM.put(MEEPROM::ADR_KP_DBL, (double)0.23);
    EEPROM.put(MEEPROM::ADR_KI_DBL, (double)2.2);
    EEPROM.put(MEEPROM::ADR_KD_DBL, (double)0.004);

    MEEPROM::update_string(MEEPROM::ADR_NAME_STR, 21, "NOT DEFINED");

    EEPROM.put(MEEPROM::ADR_VMAX_2B, 5000);
    EEPROM.put(MEEPROM::ADR_VSET_2B, 0);
    Serial.println("!!REBOOT REQUIRED!!");
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
    //EEPROM.get(MEEPROM::ADR_KP_DBL, val);
    val = gTDCDC.get_Kp();
    Serial.println(val);
}
void TComm::QKi(){
    double val;
    //EEPROM.get(MEEPROM::ADR_KI_DBL, val);
    val = gTDCDC.get_Ki();
    Serial.println(val);
}
void TComm::QKd(){
    double val;
    //EEPROM.get(MEEPROM::ADR_KD_DBL, val);
    val = gTDCDC.get_Kd();
    Serial.println(val);
}
void TComm::SRelOn(){
    gTChannels.allOn();
    gTChannels.setCurrentMode(TChannels::AllOn);
    gTChannels.printChannelsStatus();

}
void TComm::SRelOff(){
    gTChannels.allOff();
    gTChannels.setCurrentMode(TChannels::AllOff);
    gTChannels.printChannelsStatus();
}
void TComm::SRelAuto(){
    uint16_t retry_duration_s = sCmd.parseLongArg();
    char *relays_to_use_raw = sCmd.next();
    bool relays_to_use[6] = {1,1,1,1,1,1};
    for (uint8_t i = 0; i < 6; i++) {
            if ((relays_to_use_raw[i] == '0') || (relays_to_use_raw[i]) == '1') {
                relays_to_use[i] = relays_to_use_raw[i] - '0';
            }
            else {
                break;
            }
    }
    Serial.print("[INFO]: Activation auto mode. Retry: ");
    Serial.print(retry_duration_s);
    Serial.print("[s]; ");
    for (int i = 0;i < 6;i++) {
        Serial.print(relays_to_use[i]);
        Serial.print(",");
    }
    Serial.println("");
    gTChannels.setCurrentMode(TChannels::AutoMode);
    gTChannels.autoMode(retry_duration_s, relays_to_use);

}
void TComm::QRelState(){
    gTChannels.printChannelsStatus();
}

void TComm::QTestingShort() {
    Serial.println(gTChannels.isTestingShort());
}

void TComm::QCurrent() {
    Serial.println(gTDCDC.get_last_Vcurrent());
}

void TComm::SOC(){
    uint8_t val = (uint8_t)sCmd.parseLongArg();
    if ((val == 0) || (val == 1) || (val == 3)) {
        Serial.println(val);
        gTOC.setOperationMode(TOC::OPMANUAL);
        gTOC.stateChange(val);
    }
    else {
        Serial.println("Err: param");
    }    
}

void TComm::SOCF() {
    double val = (double)sCmd.parseDoubleArg();
    if (val <= gTOC.getMaxFrequencyHz()) {
        Serial.println(val);
        gTOC.setOperationMode(TOC::OPFREQUENCY, val);
    }
    else {
        Serial.println("Err: param");
    }
    //gTOC.stateChange(0);
    //Serial.println("Not implemented yet");
}

void TComm::QOC(){
    Serial.print(gTOC.getOperationMode());
    Serial.print(",");
    Serial.println(gTOC.getState());
}
void TComm::SHB(){
    uint8_t val = (uint8_t)sCmd.parseLongArg();
    if ((val >= 0) && (val <= 3)) {
        Serial.println(val);
        gTHB.setOperationMode(THB::OPMANUAL);
        gTHB.stateChange(val);
    }
    else {
        Serial.println("Err: param");
    }
}
void TComm::SHBF(){
    gTHB.setOperationMode(THB::OPFREQUENCY);
    //gTHB.stateChange(0);

    double val = (double)sCmd.parseDoubleArg();
    if (val <= gTHB.getMaxFrequencyHz()) {
        Serial.println(val);
        gTHB.setOperationMode(THB::OPFREQUENCY, val);
    }
    else {
        Serial.println("Err: param");
    }
    //gTOC.stateChange(0);
    //Serial.println("Not implemented yet");

}
void TComm::QHB(){
    Serial.print(gTHB.getOperationMode());
    Serial.print(",");
    Serial.println(gTHB.getState());
}

void TComm::QEnable() {
    Serial.println(gTDCDC.get_enable_switch());
}

void TComm::Reboot() {
    Serial.println("[INFO]: Rebooting due to user request");
    wdt_enable(WDTO_15MS);
    while (1) {}
}

void TComm::Debug() {
    uint8_t val = (uint8_t)sCmd.parseLongArg();
    Serial.println(val);
    TComm::debugEnabled = (bool)val;
}

void TComm::vpy() {
    uint8_t val = (uint8_t)sCmd.parseLongArg();
    Serial.println(val);
    
    switch (val) {
    case 0:
        gTDCDC.decrease_temporary_voltage(80);
        break;
    case 1:
        gTDCDC.restore_voltage();
        break;
    case 2:
        gTDCDC.set_target_voltage(2000);
        gTHB.stateChange(1);
        gTOC.stateChange(1);
        gTChannels.setCurrentMode(TChannels::AutoMode);
        bool tmp[] = { 1,0,1,0,1,1 };
        gTChannels.autoMode(20, tmp);
        break;
    default:
        Serial.println("Err VPY");
        break;
    }
}


// This gets set as the default handler, and gets called when no other command matches.
void TComm::unrecognized(const char *command) {
  Serial.print("Err: ");
  Serial.println(command);
}
