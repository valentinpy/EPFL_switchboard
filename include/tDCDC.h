#ifndef _TDCDC_H
#define _TDCDC_H

class TDCDC
{
public:
  TDCDC() = default;
  void setup();
  void run();

  void set_target_voltage();
  void get_measured_voltage();
  void get_target_voltage();

// private:
};
#endif
