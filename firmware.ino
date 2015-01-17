#include "RingInput.h"
//#include <Arduino.h>
RingInput input(0);
Motion motion(0);
void setup(){
  Serial.begin(115200);
}

char inp[8];
int idx = 0;
void loop (){
  int n = Serial.available();
  while (n > 0) {
    inp[idx] = Serial.read();
    idx++;
    if (idx == 8) {
      int st = input.push (inp);
      
      Serial.print("PUSH r");
      Serial.print(input.readptr, DEC);
      Serial.print(" w");
      Serial.println (input.writeptr, DEC);
      idx = 0;
    }
    n--;
  }
    
}


