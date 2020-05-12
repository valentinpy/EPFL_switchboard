#ifndef _TCHANNELS_H
#define _TCHANNELS_H

class TChannels
{
public:
  TChannels() = default;
  void setup();
  void run();

  void allOn();
  void allOff();
  bool set1(unsigned int channel, bool state);
  void autoMode(int aAutoRestart, bool* aListChannelsUsed);
  void getChannelsStatus(bool *retVal);

private:
  const int NBREL=6;
  const int Rel0_PIN=23;
  const int Rel1_PIN=22;
  const int Rel2_PIN=21;
  const int Rel3_PIN=12;
  const int Rel4_PIN=8;
  const int Rel5_PIN=7;

  const int Rel_pin[6]={Rel0_PIN, Rel1_PIN, Rel2_PIN, Rel3_PIN, Rel4_PIN, Rel5_PIN}; // digital pin used to control relay 0-5

  int Rel_status[6]={};

};
#endif
