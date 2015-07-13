#include "RingInput.h"
#include "Motion.h"

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
}

unsigned char inp[COMMAND_SIZE];  // buffer that stores the bytes of a partially received command as they come in
int idx = 0;  // how many bytes of the incoming command we've read
unsigned char checksum = 0;  // used to help detect transmission errors

volatile boolean retransmit_request = false;
volatile boolean command_received = false;  // ISR sets true when a complete command has been received.
    // we're guaranteed to only get one command in, because it comes as a response to an ACK. 

volatile boolean command_running = false;

unsigned int acknum = 0;    

// this is the main loop. Of course, this may be getting interrupted left and right by the motion timer
// or communications interrupts.

// see the host documentation for details on the communications protocol and command format. 
void loop (){
  // first check the state of the abort flag. 
  if (motion.abort == ABORT_STATUS_ABORT) {
    // something has caused an abort (probably hitting an endstop)
    motion.abort = ABORT_STATUS_RESET; 
    input.clear(); 
    command_running = false;
    retransmit_request = false;
    acknum = 0;
    Serial.write ('A');  // let the host know we got aborted
  } else if (motion.abort == ABORT_STATUS_RESET) {
    // after an abort happens, we want to wait for the user to press the resume button before we do anything.
    if (motion.checkResume()) {
      motion.abort = ABORT_STATUS_CLEAR;
      Serial.write ('C');  // let the host know things are clear again
    }
  } else {  // abort is clear
    if (retransmit_request) {  // if the communications handler found something bad, this will be set.
      Serial.write ('t');
      Serial.write ('x');
      retransmit_request = false;  // clear the flag; we just want to ask once.
    } else {  // everything's good
    
      if (command_received) {  // we have received a complete command
        if (!input.isFull()) {  // add it to the buffer if there's room. It will eventually get added once 
                                // more commands have been run. 
          command_received = false;
          ack(acknum++);  // send an acknowledgement of receipt to the host. This is the cue to the host that
                          // it is OK to send another command. This is why we don't have to worry about multiple
                          // commands coming in (and overwriting each other in the 'inp' buffer) before enough
                          // commands execute to free up space in the buffer.
        }
      }
      // now, if there's stuff to do and we're not doing anything, do stuff.   
      if (!input.isEmpty() && !command_running) {
        command_running = true;
        startCommand();  // the name here is suggestive: for commands that take a long time (moves, waits, beeps),
        // it will simply get them started (they run based on timer interrupts) and return quickly so we can keep
        // going through the main loop. It will run shorter commands in their entirety.
      }     
    }
  }
}

/* This is an ISR that gets called whenever a byte comes in off the serial port */
void serialEvent () {
    if (!connection_established) return;
    if (motion.abort != ABORT_STATUS_CLEAR) {  // discard all serial communication until resume is pressed.
      while (Serial.available() > 0) Serial.read();
      return;
    }

    inp[idx] = Serial.read();
    if (idx == 0 && inp[idx] == 255) {
      // if we receive the opcode 255 (ESTOP) as the first byte of a command, stop things NOW. Notice this does
      // not wait for the rest of the commands in the buffer to execute.
      motion.abort = ABORT_STATUS_ABORT;
      motion.cleanup();  // turn off the motion timer, etc.
    }
    idx++;
    if (idx == COMMAND_SIZE) {
      if (checksum == inp[COMMAND_SIZE-1]) {
        input.push (inp);
        command_received = true;
      } else {
        retransmit_request = true;  // if the checksum didn't match, something went bad. We probably want to ask the 
                                    // host to send it again.
      }
      idx = 0;
      checksum = 0;
    } else {
      checksum ^= inp[idx-1];  // the checksum is just the XOR of the first 10 bytes of the command. Here we keep a 
                               // running 'total' of the checksum.
    }
}

void ack (unsigned int id) {
  Serial.write ('a');
  Serial.write (id & 0xff);
  Serial.write (id >> 8);
}

/* Here is the timer interrupt. It is a little yucky because everything that uses the 16-bit timer interrupt has to
use the same ISR. What happens is determined by the value of timer_mode, which will be one of WAIT_MODE, BEEP_MODE, 
or MOTION_MODE. 
*/

volatile unsigned long ntimer_wraps = 0;  // to get long enough times, the timer may have to wrap around several times.
                                          // this variable keeps track of how many more times around we have to go.
volatile unsigned int timer_leftovers = 0;  // on the last time around, we might not go the full time - this is the count to go to.
volatile int timer_mode = WAIT_MODE;

ISR (TIMER1_COMPA_vect) {
  if (timer_mode == WAIT_MODE || timer_mode == BEEP_MODE) {
    if (ntimer_wraps == 0) {
      // pull the plug.
      TIMSK1 = 0;  // stop the timer
      command_running = false;
      return;
    } else if (ntimer_wraps == 1 && timer_mode == WAIT_MODE) {
      // do the leftovers
      TCNT1 = 0;
      OCR1A = timer_leftovers;
      ntimer_wraps = 0;
    } else {
      ntimer_wraps --;
    }
    if (timer_mode == BEEP_MODE) {
      digitalWrite (SPEAKER, (ntimer_wraps & 1) == 0 ? HIGH : LOW);
    }
  } else {  // we're in motion mode
    motion.tick();
    if (motion.state == STOPPED) {
      command_running = false;
    }
  }
}

float getFloat (unsigned char *x) {
  unsigned long f = (((unsigned long) x[0]) << 24) | (((unsigned long) x[1]) << 16) | (((unsigned long) x[2]) << 8) | ((unsigned long) x[3]);
  return * ((float *) &f);
}

void response_header (int id, int len) {
  Serial.write ('r');
  Serial.write (id >> 8);
  Serial.write (id);
  Serial.write (len);
}

void write_float (float f) {
  Serial.write ((uint8_t *) &f, 4);
}

void write_stepcount (stepcount_t s) {
  Serial.write ((uint8_t *) &s, 4);
}

/* startCommand will pop the next command off the input buffer, figure out what it is, and get it running. */
void startCommand () {
  unsigned char com[COMMAND_SIZE];
  noInterrupts();
  input.pop (com);
  interrupts();
  
  int i = 0;
  int j = 0;
  int freq = 0;
  unsigned long time = 0;
  float dat[4];
  dat[0] = getFloat (&com[3]);
  dat[1] = getFloat (&com[7]);
  dat[2] = getFloat (&com[11]);
  dat[3] = getFloat (&com[15]);
  unsigned int id = (((unsigned int) com[1]) << 8) | com[2];
  
  switch (com[0]) {
    case 0:  // NOP
      command_running = false;
      break;
    case 1:  // MOVA
      timer_mode = MOTION_MODE;
      motion.startLinearMove (dat[0], dat[1], dat[2], dat[3]);
      break;
    case 2:  // MOVR
      timer_mode = MOTION_MODE;
      motion.startLinearMove (motion.vpos[0] + dat[0], motion.vpos[1] + dat[1], motion.vpos[2] + dat[2], dat[3]);
      break;
    case 3:  // MARC
      timer_mode = MOTION_MODE;
      motion.startHelicalMove (dat[0], dat[1], dat[2], 0, dat[3]);
      break;
    case 4:  // MHLX
      timer_mode = MOTION_MODE;
      motion.startHelicalMove (dat[0], dat[1], dat[2], dat[3], motion.feedrate);
      break;
    case 5:  // HOME
      motion.homeAxes (com[3]);
      command_running = false;
      break;
    case 6:  // CLWO
      motion.worigin[0] = 0;
      motion.worigin[1] = 0;
      motion.worigin[2] = 0;
      command_running = false;
      break;
    case 7:  // SWOX
      motion.setWorkingOrigin (dat[0], 0);
      command_running = false;
      break;
    case 8:  // SWOY
      motion.setWorkingOrigin (0, dat[0]);
      command_running = false;
      break;
    case 9:  // CROT
      motion.setRotation (0);
      command_running = false;
      break;
    case 10:  // SROT
      motion.setRotation (dat[0]);
      command_running = false;
      break;
    case 11:  // EDGX
      motion.edgefind (dat[0], 0);
      command_running = false;
      break;
    case 12:  // EDGY
      motion.edgefind (dat[0], 1);
      command_running = false;
      break;
    case 13:  // EFMX
      motion.edgefindMidpoint (dat[0], dat[1], dat[2], com[15] == 0, 0);
      command_running = false;
      break;
    case 14:  // EFMY
      motion.edgefindMidpoint (dat[0], dat[1], dat[2], com[15] == 0, 1);
      command_running = false;
      break;
    case 15:  // EF2X
      motion.edgefind2 (dat[0], dat[1], dat[2], com[15] == 0, (char) 0);
      command_running = false;
      break;
    case 16:  // EF2Y
      motion.edgefind2 (dat[0], dat[1], dat[2], com[15] == 0, 1);
      command_running = false;
      break;
    case 17:  // STPE
      digitalWrite (ENABLE, HIGH);
      command_running = false;
      break;
    case 18:  // STPD
      digitalWrite (ENABLE, LOW);
      command_running = false;
      break;
    case 19:  // SPNE
      digitalWrite (SPINDLE_POWER, HIGH);
      command_running = false;
      break;
    case 20:  // SPND
      digitalWrite (SPINDLE_POWER, LOW);
      command_running = false;
      break;
    case 21:  // SSPS
      // complicated
    case 22:  // WAIT
      time = (((unsigned long) com[3]) << 8) + com[4];
      time = (unsigned long) (time * 15.625f);  // now we're in units of 64 usec.
      if (time > 65535) {
        ntimer_wraps = (unsigned long) (time >> 16);
        timer_leftovers = (unsigned int) time;
      }
      
      timer_mode = WAIT_MODE;
      TCCR1A = 0;
      TCCR1B = bit(WGM12) | bit(CS12) | bit(CS10);  // 1024x prescaling gives 1 click = 64 usec.
      OCR1A = ntimer_wraps == 0 ? timer_leftovers : 65535;
      TCNT1 = 0;
      TIMSK1 = bit(OCIE1A);  
      break;
      
    case 23:  // WUSR
      motion.resume = false;
      digitalWrite (RESUME_LED, HIGH);
      while (!motion.checkResume()) {
        delay(100);
      }
      digitalWrite (RESUME_LED, LOW);
      motion.resume = false;
      command_running = false;
      break;
      
    case 24:  // BEEP
      time = (((unsigned long) com[5]) << 8) + com[6];
      freq = (((unsigned int) com[3]) << 8) + com[4];
      freq *= 2;
      ntimer_wraps = (freq * time)/1000;
      timer_mode = BEEP_MODE;
      TCCR1A = 0;
      TCCR1B = bit(WGM12) | bit (CS12);  // 256x prescaling gives 1 click = 16 usec
      OCR1A = (unsigned int) (62500.0f/freq);
      TCNT1 = 0;
      TIMSK1 = bit(OCIE1A);
      break;
      
    case 25:  // QPOS
      response_header (id, 12);
      write_float (motion.vpos[0]);
      write_float (motion.vpos[1]);
      write_float (motion.vpos[2]);
      command_running = false;
      break;
    case 26:  // QABS
      response_header (id, 12);
      write_stepcount (motion.steppos[0]);
      write_stepcount (motion.steppos[1]);
      write_stepcount (motion.steppos[2]);
      command_running = false;
      break;
    case 27:  // QWOR
      response_header (id, 12);
      write_float (motion.worigin[0]);
      write_float (motion.worigin[1]);
      write_float (motion.worigin[2]);
      command_running = false;
      break;
    case 28:  // QROT
      response_header (id, 4);
      write_float (motion.rotation);
      command_running = false;
      break;
    case 29:  // QEND
      response_header (id, 1);
      Serial.write (motion.checkEndstops());
      command_running = false;
      break;
    case 30:  // QSPS
      response_header (id, 4);
      write_float ((float) analogRead (SPINDLE_SPEED));
      command_running = false;
      break;
    case 31:  // ECHO
      // length
      if (com[3] > 15) com[3] = 15;
      response_header (id, com[3]);
      Serial.write (&com[4], com[3]);
      command_running = false;
      break;
    case 255:  // STOP
      break;    
  
  /*
  switch (com[0]) {
    float x,y,z,f;
    case 0:  // NOP
      command_running = false;
      break;
    case 1: // move
      x = ((((unsigned int) com[2]) << 8) + com[3]) * 0.01f;  // careful with the endianness!
      y = ((((unsigned int) com[4]) << 8) + com[5]) * 0.01f;
      z = ((((unsigned int) com[6]) << 8) + com[7]) * 0.01f;
      f = ((((unsigned int) com[8]) << 8) + com[9]) * 0.01f;
      timer_mode = MOTION_MODE;
      motion.startMove (x, y, z, f);
      break;
    case 2:  // relative move
      x = ((((int) com[2]) << 8) + com[3]) * 0.01f;
      y = ((((int) com[4]) << 8) + com[5]) * 0.01f;
      z = ((((int) com[6]) << 8) + com[7]) * 0.01f;
      f = ((((int) com[8]) << 8) + com[9]) * 0.01f;
      timer_mode = MOTION_MODE;
      motion.startMove (motion.pos[0] + x, motion.pos[1] + y, motion.pos[2] + z, f);
      break;
    case 3:  // home
      motion.homeAxes (com[2]);
      command_running = false;
      break;
    case 4:  // steppers on/off
      digitalWrite (ENABLE, com[2] ? HIGH : LOW);
      command_running = false;
      break;
    case 5:  // spindle on/off
      // do something!
      command_running = false;
      break;
    case 6:  // wait
       // set up a timer.
       
       time = (((unsigned long) com[2]) << 8) + com[3];
       time = (unsigned long) (time * 15.625f);  // now we're in units of 64 usec.
       if (time > 65535) {
         ntimer_wraps = (unsigned long) (time >> 16);
         timer_leftovers = (unsigned int) time;
       }
       
       timer_mode = WAIT_MODE;
       TCCR1A = 0;
       TCCR1B = bit(WGM12) | bit(CS12) | bit(CS10);  // 1024x prescaling gives 1 click = 64 usec.
       OCR1A = ntimer_wraps == 0 ? timer_leftovers : 65535;
       TCNT1 = 0;
       TIMSK1 = bit(OCIE1A);  
       break;
    case 7:  // pause: don't do anything until the resume button is pressed
      motion.resume = false;
      digitalWrite (RESUME_LED, HIGH);
      while (!motion.checkResume()) {
        delay(100);
      }
      digitalWrite (RESUME_LED, LOW);
      motion.resume = false;
      command_running = false;
      break;
    case 8:  // beep
      // unfortunately we can't use the tone() command because it messes up the timers
      time = (((unsigned long) com[4]) << 8) + com[5];
      freq = (((unsigned int) com[2]) << 8) + com[3];
      freq *= 2;
      ntimer_wraps = (freq * time)/1000;

      timer_mode = BEEP_MODE;
      TCCR1A = 0;
      TCCR1B = bit(WGM12) | bit (CS12);  // 256x prescaling gives 1 click = 16 usec
      OCR1A = (unsigned int) (62500.0f/freq);
      TCNT1 = 0;
      TIMSK1 = bit(OCIE1A);
      
      //tone (SPEAKER, (((int) com[2]) << 8) + com[3], (((int) com[4]) << 8) + com[5]);
      break;
    case 9: // set position
      for (i=0; i < 3; i++) {
        motion.pos[i] = ((((int) com[i*2+2]) << 8) + com[i*2+3]) * 0.01f;
      }
      command_running = false;
      break; 
    case 10:  // get position
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
    case 11:  // get endstop status
      Serial.write ('r');
      Serial.write (com[1]);
      Serial.write (1);
      Serial.write (motion.checkEndstops());
      command_running = false;
      break;
    case 12:  // get spindle speed
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
  */
  }
}

