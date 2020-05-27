#include "include/hDebug.h"
#include "Arduino.h"

void HDBG::setup() {
	pinMode(DBG0_PIN, OUTPUT);
	pinMode(DBG1_PIN, OUTPUT);
	dbg0State = false;
	dbg1State = false;

	digitalWrite(DBG0_PIN, dbg0State);
	digitalWrite(DBG1_PIN, dbg1State);
}

void HDBG::toggle_0() {
	dbg0State = !dbg0State;
	digitalWrite(DBG0_PIN, dbg0State);
}

void HDBG::set_0() {
	dbg0State = true;
	digitalWrite(DBG0_PIN, dbg0State);
}

void HDBG::reset_0() {
	dbg0State = false;
	digitalWrite(DBG0_PIN, dbg0State);
}


void HDBG::toggle_1() {
	dbg1State = !dbg1State;
	digitalWrite(DBG1_PIN, dbg1State);
}

void HDBG::set_1() {
	dbg1State = true;
	digitalWrite(DBG1_PIN, dbg1State);
}

void HDBG::reset_1() {
	dbg1State = false;
	digitalWrite(DBG1_PIN, dbg1State);
}