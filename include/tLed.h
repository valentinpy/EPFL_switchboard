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
  const uint8_t HV_LED_PIN = 11;

  uint32_t lastRun;
  const uint32_t PERIOD_MS=100;
};
#endif
