#ifndef _TDCDC_H
#define _TDCDC_H

#include "hPID_v1.h"

class TDCDC
{
public:
  TDCDC();
  void setup();
  void run();

  void set_target_voltage(int voltage);
  void get_measured_voltage();
  void get_target_voltage();

  double get_C0();
  double get_C1();
  double get_C2();
  double get_Kp();
  double get_Ki();
  double get_Kd();

  void set_C0(double C0);
  void set_C1(double C1);
  void set_C2(double C2);
  void set_Kp(double Kp);
  void set_Ki(double Ki);
  void set_Kd(double Kd);

  uint16_t get_last_Vnow();
  uint16_t get_Vset();
  uint16_t get_Vmax();


private:
    double C0;
    double C1;
    double C2;

    double Kp;
    double Ki;
    double Kd;
    double Ki_save;
    double Kd_save;

    double input, output, setpoint; //3 parameters for PID regulator
    double get_HV_voltage(uint8_t nAvg);

    void initPWM();
    void setPWMDuty(uint16_t duty);

    uint32_t lastRun;
    const uint32_t PERIOD_MS = 10;
    
    PID HVPS_PID;

    uint16_t last_Vnow;
    uint16_t Vmax;
    uint16_t Vset;

//private:

    
};
#endif
