#ifndef _TLED_H
#define _TLED_H

class TLed
{
public:
  TLed() = default;
  void setup();
  void run();

private:
  bool state;
  unsigned long lastRun;

  const int PERIOD_MS=500;
  const int HV_LED_PIN=13;
};
#endif
