#include "Arduino.h"
#include "include/tLed.h"
#include "include/tDCDC.h"

// external instances of tasks
extern TDCDC gTDCDC;


void TLed::setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	lastRun = millis();
}

void TLed::run() {
	if (millis() - lastRun > PERIOD_MS) {
		if (gTDCDC.get_last_Vnow() > 100) {
			ledState = !ledState;
		}
		else {
			ledState = LOW;
		}
		lastRun = millis();
		digitalWrite(HV_LED_PIN, ledState);
	}
}
