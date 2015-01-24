#include "RingInput.h"

RingInput::RingInput (int x) {
  readptr = 0;
  writeptr = 0;
}

boolean RingInput::isEmpty () {
  return readptr == writeptr;
}

boolean RingInput::isFull () {
  char w = writeptr+1;
  if (w >= INBUF_SIZE) w = 0;
  return w == readptr;
}

int RingInput::freeSpace () {
  int used = writeptr - readptr;
  if (used < 0) used += INBUF_SIZE;
  return INBUF_SIZE - used;
}

// pops a command off the ring buffer 'inbuf', reading from readptr. 
// stores the result in 'dest,' which must be at least COMMAND_SIZE bytes
// returns 0 on success, -1 otherwise (eg, if the buffer is empty)
int RingInput::pop (unsigned char *dest) {
  if (readptr == writeptr) {  // buffer empty
    return -1;
  }
  int i;
  for (i=0; i < COMMAND_SIZE; i++) {
    dest[i] = buf[readptr][i];
  }
  readptr += 1;
  if (readptr >= INBUF_SIZE) readptr = 0;
  return 0;
}

int RingInput::push (unsigned char *src) {
  char w = writeptr+1;
  if (w >= INBUF_SIZE) w = 0;
  if (w == readptr) {  // buffer is full.
    return -1;
  }
  int i;
  for (i=0; i < COMMAND_SIZE; i++) {
    buf[writeptr][i] = src[i];
  }
  writeptr += 1;
  if (writeptr >= INBUF_SIZE) writeptr = 0;
  return 0;
}

char RingInput::peek_id () {
  if (isEmpty()) return 0;
  return buf[readptr][1];
}
