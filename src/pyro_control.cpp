#include "pyro_control.h"
#include <Arduino.h>
#include "config.h"

void pyro_init_safe() {
  // Latch LOW first, then enable the output driver, then assert LOW again, so the
  // pin can never present a transient HIGH while it changes from input to output.
  digitalWrite(PYRO_CHANNEL_1, LOW);
  digitalWrite(PYRO_CHANNEL_2, LOW);
  pinMode(PYRO_CHANNEL_1, OUTPUT);
  pinMode(PYRO_CHANNEL_2, OUTPUT);
  digitalWrite(PYRO_CHANNEL_1, LOW);
  digitalWrite(PYRO_CHANNEL_2, LOW);
}
