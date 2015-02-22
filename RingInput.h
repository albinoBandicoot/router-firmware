#ifndef RingInput_H
#define RingInput_H
#include <Arduino.h>

#define INBUF_SIZE 16	// number of commands in the input buffer

#define COMMAND_SIZE 11  // number of bytes in a command

class RingInput {
  private:
    unsigned char buf[INBUF_SIZE][COMMAND_SIZE];
  public:
    char readptr;
    char writeptr;

    RingInput (int);
    boolean isEmpty();
    boolean isFull();
    int freeSpace ();
    int pop (unsigned char *);
    int push (unsigned char *);
    char peek_id ();
    void clear ();
};

extern RingInput input;
#endif
