#include "RingInput.h"
#include "Motion.h"
//#include <Arduino.h>
RingInput input(0);
Motion motion(0);
void setup(){
  Serial.begin(BAUDRATE);
  motion.begin();
  delay (2000);
  while (Serial.read() == -1) {
    delay (500);
  }
  while (Serial.available() > 0) Serial.read();  // get rid of any gunk.
  Serial.write ('a');
  Serial.write (0);

}

char inp[COMMAND_SIZE];
int idx = 0;
char checksum = 0;

void loop (){
  /*
  if (Serial.peek() != -1) {
    char in = Serial.read();
    Serial.write (in);
  }
  */




//void serialEvent () {
  int n = Serial.available();
  while (n > 0) {
    inp[idx] = Serial.read();
    if (idx == 0 && inp[idx] == 255) {
      // EMERGENCY STOP NOW!
    }
//    Serial.write (inp[idx]);
    idx++;
    if (idx == COMMAND_SIZE) {
      if (checksum == inp[COMMAND_SIZE-1]) {
        input.push (inp);
        char id = input.peek_id();
        if (input.isFull()) {
          runCommand();
          ack(id);
        } else {
          ack(id);
          runCommand();
        }

        idx = 0;
        checksum = 0;
      } else {
        idx = 0;
        checksum = 0;
        Serial.write ('t');
        Serial.write ('x');  // retransmit request
      }
    } else {
      checksum ^= inp[idx-1];
    }
    n--;
  }
}

void ack (char id) {
  Serial.write ('a');
  Serial.write (id);
}

void runCommand () {
  char com[COMMAND_SIZE];
  int *comi = (int *) (&com[0]);
  input.pop (com);
//  Serial.write (com[0]);
  int i = 0;
  switch (com[0]) {
    case 0:  break;
    case 1: // move 
      break;
    case 2:  // home
      break;
    case 3:  // steppers on/off
      digitalWrite (ENABLE, com[2] ? HIGH : LOW);
      break;
    case 4:  // spindle on/off
      break;
    case 5:  // wait
       delay ((((int) com[2]) << 8) + com[3]);
       break;
    case 6:  // pause
      break;
    case 7:  // beep
    #ifdef PIEZO
      tone (PIEZO, comi[1], comi[2]);
    #endif
      break;
    case 8: // set position
      break; 
    case 9:  // get position
      break;
    case 10:  // get endstop status
      break;
    case 11:  // get spindle speed
      break;
    case 16:  // echo
      i = 2;
      Serial.write ('r');
      Serial.write (com[1]);
      while (i < 10) {
        if (com[i] == 0) break;
        i++;
      }
      Serial.write (i - 2);
      i = 2;
      while (i < 10) {
        if (com[i] == 0) break;
        Serial.write (com[i]);
        i++;
      }
      break;
    case 255:  // estop
      return;
  }

}

