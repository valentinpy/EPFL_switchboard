#ifndef _HDBG_H_
#define _HDBG_H_

#include "Arduino.h"

class HDBG
{
public:
    HDBG() = default;
    void setup();
    void toggle_0();
    void toggle_1();

    void set_0();
    void reset_0();
    void set_1();
    void reset_1();

private:
    bool dbg0State;
    bool dbg1State;
    const uint8_t DBG0_PIN = 4;
    const uint8_t DBG1_PIN = 3;
};

#endif