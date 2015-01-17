#ifndef RingInput_H
#define RingInput_H
#include <Arduino.h>

#define INBUF_SIZE 16
#define OUTBUF_SIZE 16

#define COMMAND_SIZE 8  // number of bytes in a command

class RingInput {
  private:
    char buf[INBUF_SIZE][COMMAND_SIZE];
  public:
    char readptr;
    char writeptr;

    RingInput (int);
    boolean isEmpty();
    boolean isFull();
    int freeSpace ();
    int pop (char *);
    int push (char *);
};

extern RingInput input;
#endif
