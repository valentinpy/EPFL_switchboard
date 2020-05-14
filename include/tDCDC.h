#ifndef _TDCDC_H
#define _TDCDC_H

#include "hPID_v1.h"

class TDCDC
{
public:
  TDCDC();
  void setup();
  void run();

  void set_target_voltage();
  void get_measured_voltage();
  void get_target_voltage();

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

    PID HVPS_PID;

// private:
};
#endif
