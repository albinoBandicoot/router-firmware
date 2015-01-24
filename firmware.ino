#include "RingInput.h"
#include "Motion.h"
//#include <Arduino.h>
RingInput input(0);
Motion motion(0);

boolean connection_established = false;

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
  connection_established = true;
  
  PCMSK1 |= bit(PCINT8);  // A0 pin
  PCIFR  |= bit(PCIF0);
  PCICR  |= bit (PCIE1);

}

unsigned char inp[COMMAND_SIZE];
int idx = 0;
unsigned char checksum = 0;

volatile boolean retransmit_request = false;
volatile boolean command_received = false;  // ISR sets true when a complete command has been received.
    // we're guaranteed to only get one command in, because it comes as a response to an ACK. 

volatile boolean command_running = false;

unsigned char acknum = 0;    

void loop (){
  if (retransmit_request) {
    Serial.write ('t');
    Serial.write ('x');
    retransmit_request = false;
  } else {
    
    if (command_received) {
      if (!input.isFull()) {
        command_received = false;
        ack(acknum++);
      }
    }
    
    if (!input.isEmpty() && !command_running) {
      command_running = true;
      startCommand();
    }   
  }
}

void serialEvent () {  // this is an ISR, right?
    if (!connection_established) return;

    inp[idx] = Serial.read();
    if (idx == 0 && inp[idx] == 255) {
      // EMERGENCY STOP NOW!
    }
    idx++;
    if (idx == COMMAND_SIZE) {
      if (checksum == inp[COMMAND_SIZE-1]) {
        input.push (inp);
        command_received = true;
      } else {
        retransmit_request = true;
      }
      idx = 0;
      checksum = 0;
    } else {
      checksum ^= inp[idx-1];
    }
}

void ack (char id) {
  Serial.write ('a');
  Serial.write (id);
}

volatile unsigned int ntimer_wraps = 0;
volatile unsigned int timer_leftovers = 0;
volatile boolean timer_mode = WAIT_MODE;

ISR (TIMER1_COMPA_vect) {
  if (timer_mode == WAIT_MODE) {
    if (ntimer_wraps == 0) {
      // pull the plug.
      TIMSK1 = 0;  // stop the timer
      command_running = false;
    } else if (ntimer_wraps == 1) {
      // do the leftovers
      TCNT1 = 0;
      OCR1A = timer_leftovers;
      ntimer_wraps = 0;
    } else {
      ntimer_wraps --;
    }
  } else {
    motion.tick();
    if (motion.state == STOPPED) {
      command_running = false;
    }
  }
}

void startCommand () {
  unsigned char com[COMMAND_SIZE];
  noInterrupts();
  input.pop (com);
  interrupts();
  
  int i = 0;
  int j = 0;
  unsigned long time = 0;
  switch (com[0]) {
    float x,y,z,f;
    case 0:  
      command_running = false;
      break;
    case 1: // move
      x = ((((int) com[2]) << 8) + com[3]) * 0.01f;
      y = ((((int) com[4]) << 8) + com[5]) * 0.01f;
      z = ((((int) com[6]) << 8) + com[7]) * 0.01f;
      f = ((((int) com[8]) << 8) + com[9]) * 0.01f;
      timer_mode = MOTION_MODE;
      motion.startMove (x, y, z, f);
      break;
    case 2:  // home
      break;
    case 3:  // steppers on/off
      digitalWrite (ENABLE, com[2] ? HIGH : LOW);
      command_running = false;
      break;
    case 4:  // spindle on/off
      command_running = false;
      break;
    case 5:  // wait
       // set up a timer.
       
       time = (((int) com[2]) << 8) + com[3];
       time = (unsigned long) (time * 15.625f);  // now we're in units of 64 usec.
       if (time > 65535) {
         ntimer_wraps = (unsigned int) (time >> 16);
         timer_leftovers = (unsigned int) time;
       }
       
       timer_mode = WAIT_MODE;
       TCCR1A = 0;
       TCCR1B = bit(WGM12) | bit(CS12) | bit(CS10);  // 1024x prescaling gives 1 click = 64 usec.
       OCR1A = ntimer_wraps == 0 ? timer_leftovers : 65535;
       TIMSK1 = bit(OCIE1A);  
       //delay ((((int) com[2]) << 8) + com[3]);
       break;
    case 6:  // pause
      break;
    case 7:  // beep
    #ifdef SPEAKER
      tone (SPEAKER, (((int) com[2]) << 8) + com[3], (((int) com[4]) << 8) + com[5]);
    #endif
      command_running = false;
      break;
    case 8: // set position
      command_running = false;
      break; 
    case 9:  // get position
      Serial.write ('r');
      Serial.write (com[1]);
      Serial.write (6);
      for (i=0; i < 3; i++) {
        j = (int) (motion.pos[i] * 100);
        Serial.write (j >> 8);
        Serial.write ((byte) j);
      }
      command_running = false;      
      break;
    case 10:  // get endstop status
      Serial.write ('r');
      Serial.write (com[1]);
      Serial.write (1);
      Serial.write (motion.checkEndstops());
      command_running = false;
      break;
    case 11:  // get spindle speed
      i = analogRead (SPINDLE_SPEED);
      Serial.write ('r');
      Serial.write (com[1]);
      Serial.write (2);
      Serial.write ((byte) (i >> 8));
      Serial.write ((byte) i);
      command_running = false;
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
      command_running = false;
      break;
    case 255:  // estop
      break;
  }
}

