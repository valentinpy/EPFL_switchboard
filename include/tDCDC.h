#ifndef _TDCDC_H
#define _TDCDC_H

#include "hPID_v1.h"

class TDCDC
{
public:
  TDCDC();
  void setup();
  void run();

  void set_target_voltage(uint16_t  voltage);

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
    /*double Ki_save;
    double Kd_save;*/

    double input, output, setpoint; //3 parameters for PID regulator
    PID HVPS_PID;

    uint32_t timer;
    const uint32_t PERIOD_MS = 1;
    
    uint16_t last_Vnow;
    uint16_t Vmax;

    void initPWM();
    void setPWMDuty(uint16_t duty);
    double get_HV_voltage(uint8_t nAvg);
    double get_HV_voltage_fast(float alpha);


    //------------------------------------------------------
    // High voltage feedback filtering stuff
    //------------------------------------------------------
    // According to: https://en.wikipedia.org/wiki/Exponential_smoothing
    // Let T be sampling frequency
    // Let tau be expected time constant of filter
    // Assuming tau >> T
    // alpha = T/tau
    // T = 1ms, alpha = 0.1 => tau = 10ms
    uint32_t timerHVmeas; //timer for high voltage feedback filtering
    const uint32_t PERIOD_HVMEAS_MS = 1; //sampling for high voltage feedback filtering
    const float HVMEAS_ALPHA = 0.1; //alpha constant for high voltage feedback filtering
};
#endif
