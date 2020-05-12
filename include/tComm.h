#ifndef _TCOMM_H
#define _TCOMM_H

class TComm {
public:
  TComm() = default;
  void setup();
  void run();

private:
  static void QVmax();
  static void SVmax();
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
  static void SRelOn();
  static void SRelOff();
  static void SRelAuto();
  static void QRelState();
  static void SOCon();
  static void SOCoff();
  static void SOCF();
  static void QOC();
  static void SHBp();
  static void SHBm();
  static void SHB0();
  static void QHB();

  static void testCmd();
  static void unrecognized(const char *command);
};
#endif
