#ifndef _TLED_H
#define _TLED_H

class TLed
{
public:
  TLed() = default;
  void setup();
  void run();

private:
  bool ledState;
  unsigned long lastRun;

  const int PERIOD_MS=100;
  const int HV_LED_PIN=11;
};
#endif
