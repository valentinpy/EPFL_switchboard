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
uint32_t TComm::deadManSwitchTimeout_ms = 0;
uint32_t TComm::timer = 0;

void TComm::setup() {
	Serial.begin(115200);
	Serial.println("Switchboard V2");

	//SERIALCOMMAND_MAXCOMMANDLENGTH 5 -> Maximum length of a command excluding the terminating null
	//SERIALCOMMAND_MAXCOMMANDCOUNT 44 -> Maximum number of different commands to store
	// Do not exceed these values! Can be changed in mSerialCommand.h if necesssary but watch out for maximum memory usage
	sCmd.addCommand("QVmax", this->QVmax);
	sCmd.addCommand("SVmax", this->SVmax);
	sCmd.addCommand("QVmin", this->QVmin);
	sCmd.addCommand("SVmin", this->SVmin);
	sCmd.addCommand("SVset", this->SVset);
	sCmd.addCommand("QVset", this->QVset);
	sCmd.addCommand("QVnow", this->QVnow);
	sCmd.addCommand("QCur", this->QCurrent);
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
	sCmd.addCommand("SROn", this->SRelOn);
	sCmd.addCommand("SROff", this->SRelOff);
	sCmd.addCommand("SRAut", this->SRelAuto);
	sCmd.addCommand("QR", this->QRelState);
	sCmd.addCommand("QTest", this->QTestingShort);
	sCmd.addCommand("QShrt", this->QShortDetected);
	sCmd.addCommand("QStbl", this->QStable);
	sCmd.addCommand("SOC", this->SOC);
	sCmd.addCommand("SOCF", this->SOCF);
	sCmd.addCommand("QOC", this->QOC);
	sCmd.addCommand("SHB", this->SHB);
	sCmd.addCommand("SHBF", this->SHBF);
	sCmd.addCommand("QHB", this->QHB);
	sCmd.addCommand("QEnbl", this->QEnable);
	sCmd.addCommand("SDMS", this->SDeadManSwitch);
	sCmd.addCommand("Rebt", this->Reboot);
	sCmd.addCommand("Debug", this->Debug);
	sCmd.addCommand("vpy", this->vpy); // vpy testing command
	sCmd.setDefaultHandler(this->unrecognized); // Handler for command that isn't matched

	TComm::debugEnabled = false;
	TComm::deadManSwitchTimeout_ms = 0;
	TComm::timer = millis();
}

void TComm::run() {
	sCmd.readSerial();     // We don't do much, just process serial commands

	// if dead man's switch is enabled (>0), check when the last message was received from the host
	if (TComm::deadManSwitchTimeout_ms) {
		if ((millis() - sCmd.t_last_message_received) > TComm::deadManSwitchTimeout_ms) {
			Serial.print("[WARN]: No message from host in ");
			Serial.print(TComm::deadManSwitchTimeout_ms/1000.0);
			Serial.println("s. Setting voltage to 0!");

			TComm::deadManSwitchTimeout_ms = 0; // switch off DMS so it doesn't keep triggering on every cycle
			gTDCDC.set_target_voltage(0);
		}
	}

	// if debug is enabled, print current state every 100 ms
	if (TComm::debugEnabled) {
		if ((millis() - TComm::timer) > TComm::DELAY_MS) {
			TComm::timer = millis();
			debugPrint();
		}
	}
	return;
}

void TComm::debugPrint() {
	Serial.println("Vnow[V] Stable[0/1000] PWM[1023-0] Inow[0.1mA]");
	Serial.print(gTDCDC.get_last_Vnow());
	Serial.print(", ");
	Serial.print(gTDCDC.is_voltage_stable() * 1000);
	Serial.print(", ");
	Serial.print(gTDCDC.get_last_PWM());
	Serial.print(", ");
	Serial.print(gTDCDC.get_last_Inow() * 10);
	Serial.println("");
}

void TComm::QVmax() {
	unsigned int val;
	val = gTDCDC.get_Vmax();
	Serial.println(val);
}
void TComm::SVmax() {
	unsigned int val = (unsigned int)sCmd.parseLongArg();
	EEPROM.put(MEEPROM::ADR_VMAX_2B, val);
	gTDCDC.set_Vmax(val);
	Serial.println(gTDCDC.get_Vmax());
}
void TComm::QVmin() {
	unsigned int val;
	val = gTDCDC.get_Vmin();
	Serial.println(val);
}
void TComm::SVmin() {
	unsigned int val = (unsigned int)sCmd.parseLongArg();
	EEPROM.put(MEEPROM::ADR_VMIN_2B, val);
	gTDCDC.set_Vmin(val);
	Serial.println(gTDCDC.get_Vmin());
}
void TComm::QVset() {
	unsigned int val;
	val = gTDCDC.get_Vset();
	Serial.println(val);
}
void TComm::SVset() {
	unsigned int val = (unsigned int)sCmd.parseLongArg();
	val = gTDCDC.set_target_voltage(val); // returns the voltage is actually set after applying min and max limits
	if (val > 0 && !gTDCDC.get_enable_switch())
		Serial.println("[WARN]: Safety switch disabled or 12V disconnected!");
	Serial.println(val); // print the value that was actually set


	//EEPROM.put(MEEPROM::ADR_VSET_2B,val);
	//Serial.println("Saved - requires reboot");
	
}
void TComm::QVnow() {
	//Serial.println("Not implemented yet");
	Serial.println(gTDCDC.get_last_Vnow());
}
void TComm::QName() {
	char buff[21];
	MEEPROM::read_string(MEEPROM::ADR_NAME_STR, 21, buff);
	Serial.println(buff);
}
void TComm::SName() {
	char* buffptr;
	buffptr = sCmd.next();
	MEEPROM::update_string(MEEPROM::ADR_NAME_STR, 21, buffptr);
	Serial.println(buffptr);
}
void TComm::QMem() {
	Serial.println("Not implemented yet");
}
void TComm::QVer() {
	Serial.println(SOFTWARE_VERSION);
}
void TComm::Conf() {
	Serial.println("Initializing switchboard config with default values");

	EEPROM.put(MEEPROM::ADR_C0_DBL, (double)0.0);
	EEPROM.put(MEEPROM::ADR_C1_DBL, (double)1.0);
	EEPROM.put(MEEPROM::ADR_C2_DBL, (double)0.0);

	EEPROM.put(MEEPROM::ADR_KP_DBL, (double)0.15);
	EEPROM.put(MEEPROM::ADR_KI_DBL, (double)1.0);
	EEPROM.put(MEEPROM::ADR_KD_DBL, (double)0.0);

	MEEPROM::update_string(MEEPROM::ADR_NAME_STR, 21, "NOT DEFINED");

	EEPROM.put(MEEPROM::ADR_VMAX_2B, 5000);
	EEPROM.put(MEEPROM::ADR_VMIN_2B, 100);
	EEPROM.put(MEEPROM::ADR_VSET_2B, 0);
	Serial.println("!!REBOOT REQUIRED!!");
}
void TComm::SC0() {
	double val = (double)sCmd.parseDoubleArg();
	Serial.println(val);
	EEPROM.put(MEEPROM::ADR_C0_DBL, val);
	gTDCDC.set_C0(val);
}
void TComm::SC1() {
	double val = (double)sCmd.parseDoubleArg();
	Serial.println(val);
	EEPROM.put(MEEPROM::ADR_C1_DBL, val);
	gTDCDC.set_C1(val);
}
void TComm::SC2() {
	double val = (double)sCmd.parseDoubleArg();
	Serial.println(val);
	EEPROM.put(MEEPROM::ADR_C2_DBL, val);
	gTDCDC.set_C2(val);
}
void TComm::QC0() {
	double val = gTDCDC.get_C0();
	Serial.println(val);
}
void TComm::QC1() {
	double val = gTDCDC.get_C1();
	Serial.println(val);
}
void TComm::QC2() {
	double val = gTDCDC.get_C2();
	Serial.println(val);
}
void TComm::SKp() {
	double val = (double)sCmd.parseDoubleArg();
	Serial.println(val);
	EEPROM.put(MEEPROM::ADR_KP_DBL, val);
	gTDCDC.set_Kp(val);
}
void TComm::SKi() {
	double val = (double)sCmd.parseDoubleArg();
	Serial.println(val);
	EEPROM.put(MEEPROM::ADR_KI_DBL, val);
	gTDCDC.set_Ki(val);
}
void TComm::SKd() {
	double val = (double)sCmd.parseDoubleArg();
	Serial.println(val);
	EEPROM.put(MEEPROM::ADR_KD_DBL, val);
	gTDCDC.set_Kd(val);
}
void TComm::QKp() {
	double val;
	//EEPROM.get(MEEPROM::ADR_KP_DBL, val);
	val = gTDCDC.get_Kp();
	Serial.println(val);
}
void TComm::QKi() {
	double val;
	//EEPROM.get(MEEPROM::ADR_KI_DBL, val);
	val = gTDCDC.get_Ki();
	Serial.println(val);
}
void TComm::QKd() {
	double val;
	//EEPROM.get(MEEPROM::ADR_KD_DBL, val);
	val = gTDCDC.get_Kd();
	Serial.println(val);
}
void TComm::SRelOn() {
	char* relays_to_use_raw = sCmd.next();
	bool relays_to_use[6] = { 1,1,1,1,1,1 }; // default to 1 so that all relays turn on if nothing is specified
	for (uint8_t i = 0; i < 6; i++) {
		if ((relays_to_use_raw[i] == '0') || (relays_to_use_raw[i]) == '1') {
			relays_to_use[i] = relays_to_use_raw[i] - '0';
		}
		else {
			break;
		}
	}
	gTChannels.auto_disconnect_enabled = false;
	gTChannels.auto_reconnect_enabled = false;
	gTChannels.setAllRelays(relays_to_use);
	gTChannels.printChannelsStatus();

}
void TComm::SRelOff() {
	gTChannels.auto_disconnect_enabled = false;
	gTChannels.auto_reconnect_enabled = false;
	gTChannels.allOff();
	gTChannels.printChannelsStatus();
}
void TComm::SRelAuto() {
	uint16_t retry_duration_s = sCmd.parseLongArg();
	bool enable_reconnect = sCmd.parseLongArg();
	char* relays_to_use_raw = sCmd.next();
	bool relays_to_use[6] = { 1,1,1,1,1,1 }; // default to 1 so that all relays turn on if nothing is specified
	for (uint8_t i = 0; i < 6; i++) {
		if ((relays_to_use_raw[i] == '0') || (relays_to_use_raw[i]) == '1') {
			relays_to_use[i] = relays_to_use_raw[i] - '0';
		}
		else {
			break;
		}
	}

	// print auto mode parameter info
	Serial.print("[INFO]: Activating auto mode. Retry: ");
	Serial.print(retry_duration_s);
	Serial.print(" s; ");
	Serial.print("Auto reconnect: ");
	if(enable_reconnect)
		Serial.print("on");
	else
		Serial.print("off");
	Serial.println();

	gTChannels.autoMode(retry_duration_s, enable_reconnect, relays_to_use);

	gTChannels.printChannelsStatus();
}

//void TComm::SRelAutoDisconnect()
//{
//	bool enabled = sCmd.parseLongArg();
//	gTChannels.auto_disconnect_enabled = enabled;
//	if (!enabled) {
//		gTChannels.auto_reconnect_enabled = enabled; // can't do auto reconnect without auto disconnect
//		gTChannels.reset(); // if turning auto reconnect off, we need should reset to restore user defined state and stop any ongoing testing
//	}
//}

//void TComm::SRelAutoReconnect()
//{
//	bool enabled = sCmd.parseLongArg();
//	gTChannels.auto_reconnect_enabled = enabled;
//	if (!enabled) {
//		gTChannels.reset(); // if turning auto reconnect off, we need should reset to restore user defined state and stop any ongoing testing
//	}
//}

//void TComm::SRelAutoReset()
//{
//	uint16_t delay = sCmd.parseLongArg();
//	gTChannels.setAutoRestartDelay(delay);
//}

void TComm::QRelState() {
	gTChannels.printChannelsStatus();
}

void TComm::QShortDetected()
{
	Serial.println(gTChannels.isShortDetected());
}

void TComm::QTestingShort() {
	Serial.println(gTChannels.isTestingShort());
}

void TComm::QStable()
{
	Serial.println(gTDCDC.is_voltage_stable());
}

void TComm::QCurrent() {
	Serial.println(gTDCDC.get_last_Inow());
}

void TComm::SOC() {
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

void TComm::QOC() {
	Serial.print(gTOC.getOperationMode() * !gTOC.ac_paused);  // if ac_paused is true, op mode is temporarily manual (0) regardless of the current setting
	Serial.print(",");
	Serial.println(gTOC.getState());
}
void TComm::SHB() {
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
void TComm::SHBF() {
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
void TComm::QHB() {
	Serial.print(gTHB.getOperationMode() * !gTHB.ac_paused);
	Serial.print(",");
	Serial.println(gTHB.getState());
}

void TComm::QEnable() {
	Serial.println(gTDCDC.get_enable_switch());
}

void TComm::SDeadManSwitch()
{
	double val = (double)sCmd.parseDoubleArg();
	TComm::deadManSwitchTimeout_ms = val * 1000; // convert to ms and store as uint32
	Serial.println(TComm::deadManSwitchTimeout_ms/1000.0);
}

void TComm::Reboot() {
	Serial.println("Rebooting");
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
		gTDCDC.target_voltage_modifier = 0.8;
		break;
	case 1:
		gTDCDC.restore_voltage();
		break;
	case 2:
		gTDCDC.set_target_voltage(2000);
		gTHB.stateChange(1);
		gTOC.stateChange(1);
		bool tmp[] = { 1,1,0,1,0,1 };
		gTChannels.autoMode(0, 1, tmp);
		break;
	default:
		Serial.println("Err VPY");
		break;
	}
}


// This gets set as the default handler, and gets called when no other command matches.
void TComm::unrecognized(const char* command) {
	Serial.print("Err: ");
	Serial.println(command);
}
