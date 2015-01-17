#include "Motion.h"

volatile unsigned long lastActivation[3] = {0,0,0};

ISR (PCINT0_vect) {
  unsigned long time = millis();
  if (digitalRead (PIN_XLIMIT) == LOW) lastActivation[0] = time;
  if (digitalRead (PIN_YLIMIT) == LOW) lastActivation[1] = time;
  if (digitalRead (PIN_ZLIMIT) == LOW) lastActivation[2] = time;
}

char Motion::checkEndstops () {
  char res = 0;
  if (digitalRead (PIN_XLIMIT) == LOW) res |= XAXIS_BIT;
  if (digitalRead (PIN_YLIMIT) == LOW) res |= YAXIS_BIT;
  if (digitalRead (PIN_ZLIMIT) == LOW) res |= ZAXIS_BIT;
  unsigned long time = millis();
  int i;
  for (i=0; i < 3; i++) {
    if (time - lastActivation[i] < DEBOUNCE_TIME) res |= (1 << i);
  }
  return res;
}


// units are in steps, or steps/second. 
void Motion::runMove (pos_t xt, pos_t yt, pos_t zt, int feedrate){
  if (state == STOPPED) {
    state = MOVING;
    target[0] = xt;
    target[1] = yt;
    target[2] = zt;
    int dx = xt - pos[0];
    int dy = yt - pos[1];
    int dz = zt - pos[2];
    digitalWrite (PIN_XDIR, (dx < 0 ^ INVERT_X) ? HIGH: LOW);
    digitalWrite (PIN_YDIR, (dy < 0 ^ INVERT_Y) ? HIGH: LOW);
    digitalWrite (PIN_ZDIR, (dz < 0 ^ INVERT_Z) ? HIGH: LOW);
    
    
  }
}
void Motion::homeAxes (char axes){
  state = HOMING;
  target[0] = 0;
  target[1] = 0;
  target[2] = 0;
  digitalWrite (PIN_XDIR, INVERT_X ? HIGH : LOW);
  digitalWrite (PIN_YDIR, INVERT_Y ? HIGH : LOW);
  digitalWrite (PIN_ZDIR, INVERT_Z ? HIGH : LOW);
  char e = checkEndstops();
  while (e != 7) {  // while some endstop is not pressed:
    if ( ~(e & XAXIS_BIT) ) pulse (PIN_XSTEP);
    if ( ~(e & YAXIS_BIT) ) pulse (PIN_YSTEP);
    if ( ~(e & ZAXIS_BIT) ) pulse (PIN_ZSTEP);
    delayMicroseconds (HOMING_DELAY);
  }  
}

void pulse (int pin) {
  digitalWrite (pin, HIGH);
  delayMicroseconds (PULSE_WIDTH);
  digitalWrite (pin, LOW);
}
