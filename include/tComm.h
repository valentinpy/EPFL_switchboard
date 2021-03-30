#ifndef _TCOMM_H
#define _TCOMM_H

class TComm {
public:
  TComm() = default;
  void setup();
  void run();
  

  static uint8_t debug;
private:
  static void QVmax();
  static void SVmax();
  static void QVmin();
  static void SVmin();
  static void SVset();
  static void QVset();
  static void QVnow();
  static void QName();
  static void SName();
  static void QMem();
  static void QVer();
  static void Conf();
  static void SC0();
  static void SC1();
  static void SC2();
  static void QC0();
  static void QC1();
  static void QC2();
  static void SKp();
  static void SKi();
  static void SKd();
  static void QKp();
  static void QKi();
  static void QKd();
  static void SHvdiv();
  static void QHvdiv();
  static void SRelOn();
  static void SRelOff();
  static void SRelAuto();
  static void QRelState();
  static void QShortDetected();
  static void QTestingShort();
  static void QStable();
  static void QCurrent();
  static void SOC();
  static void SOCF();
  static void QOC();
  static void SHB();
  static void SHBF();
  static void QHB();
  static void QEnable();
  static void SDeadManSwitch();
  static void Reboot();
  static void Debug();
  static void vpy(); // vpy testing command
  static void unrecognized(const char *command);

  static void debugPrint(int level);

  static uint32_t deadManSwitchTimeout_ms;

  static uint32_t timer;
  static const uint32_t DELAY_MS = 100;
};
#endif
