#ifndef _tEEPROM_h
#define _tEEPROM_h

class MEEPROM
{
public:
  MEEPROM() = default;
  void setup();

  static void read_string(word aAddr, byte an, char * c); //reads a string from EEPROM at address addr, and with a length of n or less (n includes the termination character \0) and store it in C
  static void update_string(word aAddr, byte an, char * c); //writes string c at address addr, but not more than n bytes

  //------------------------------------------------------
  //EEPROM map
  //------------------------------------------------------

  // O free
  static const int ADR_VMAX=1;       // (2B) Voltage rating of the board
  static const int ADR_SWSRC=3;      // (1B) Switching source: 0: onboard timer,  1: push button
  static const int ADR_VSET=4;       // (2B) setpoint voltage is read from EEPROM at startup only if board in mode 0 (used as stand alone single channel)
  const int ADR_SWMODE=6;     // (1B) switching mode: 0-off, 1-DC, 2-Switching at defined frequency. Only used when switching source (SwSrc) is 0
  static const int ADR_LATCHMODE=7;  // (1B) 0: button acts as a push button: HV is on while button is pressed. 1: button acts as a switch with a latching action (press to change state)
  // 8 free
  // 9 free
  static const int ADR_C0=10;        // (4B) calibration factor for the voltage reading to account for resistive bridge imprecision. VoutCorr=C2*1E-6*V^2+C1*V+C0 (double values).
  static const int ADR_C1=14;        // (4B) calibration factor for the voltage reading to account for resistive bridge imprecision. VoutCorr=C2*1E-6*V^2+C1*V+C0 (double values).
  static const int ADR_C2=18;        // (4B) calibration factor for the voltage reading to account for resistive bridge imprecision. VoutCorr=C2*1E-6*V^2+C1*V+C0 (double values).
  static const int ADR_CYCLE=22;     // (2B) number of switching cycles to make in limited switching mode
  static const int ADR_F=24;         // (4B) double
  static const int ADR_KP=28;        // (4B) PID gain double
  static const int ADR_KI=32;        // (4B) PID integral term double
  static const int ADR_KD=36;        // (4B) PID derivative term double
  // 40-99 free
  static const int ADR_NAME=100; //store the board name (21 bytes) located at 100 to leave some space to add new stored variables without having to resave the name


private:

};
#endif
