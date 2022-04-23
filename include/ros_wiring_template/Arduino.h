#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>

#define INPUT 0
#define OUTPUT 1

#define LOW 0
#define HIGH 1

#define F(x) x

inline void pinMode(uint16_t, uint16_t)
{

}

inline void digitalWrite(uint16_t, uint16_t)
{

}


class Stream
{
    public:
    void print(char)
    {

    }

    void println(char)
    {
        
    }

    void println()
    {
        
    }
};

extern Stream Serial;


inline void delay(uint32_t)
{

}

#define pgm_read_byte_near(x) *x
#define __FlashStringHelper void

#endif