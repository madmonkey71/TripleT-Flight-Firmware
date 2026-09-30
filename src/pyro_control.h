#ifndef PYRO_CONTROL_H
#define PYRO_CONTROL_H

// Pyrotechnic output ownership.
//
// audit #1: pyro_init_safe() is the FIRST thing setup() runs so both channels are
// forced LOW before any slow initialisation (Serial wait, SD, sensors) can leave
// the pins floating.

// Force both pyro pins to OUTPUT / LOW. Safe to call repeatedly.
void pyro_init_safe();

#endif // PYRO_CONTROL_H
