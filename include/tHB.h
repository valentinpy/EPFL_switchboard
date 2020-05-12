#ifndef _THB_H
#define _THB_H

class THB
{
public:


  THB() = default;
  void setup();
  void run();

  void gnd();
  void highZ();
  void hv_A();
  void hv_B();

private:
  const int HB_LINA_PIN=16; // H-Bridge: low side, side A
  const int HB_LINB_PIN=5; // H-Bridge: low side, side B
  const int HB_HINA_PIN=17; // H-Bridge: high side, side A
  const int HB_HINB_PIN=6; // H-Bridge: high side, side B

};
#endif
