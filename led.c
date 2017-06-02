/*
Author: Erik Andersen
*/
#include <avr/io.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>

#define ON 1
#define OUT 1
#define IN 0
#define OFF 0

#define PATMASK 0x7F
#define MASK0 0x6F
#define MASK1 0x03
#define MASK2 0x76
#define MASK3 0x37
#define MASK4 0x1B
#define MASK5 0x3D
#define MASK6 0x7D
#define MASK7 0x07
#define MASK8 0x7F
#define MASK9 0x3F

static uint8_t red=156, green=234, blue=98;

uint8_t getCharBits(uint8_t charVal)
{
    if (0 == charVal)
    {
        return MASK0;
    }
    else if (1 == charVal)
    {
        return MASK1;
    }
    else if (2 == charVal)
    {
        return MASK2;
    }
    else if (3 == charVal)
    {
        return MASK3;
    }
    else if (4 == charVal)
    {
        return MASK4;
    }
    else if (5 == charVal)
    {
        return MASK5;
    }
    else if (6 == charVal)
    {
        return MASK6;
    }
    else if (7 == charVal)
    {
        return MASK7;
    }
    else if (8 == charVal)
    {
        return MASK8;
    }
    else if (9 == charVal)
    {
        return MASK9;
    }
    return 0x7C;
}

uint32_t byteToDecPattern(uint8_t bite)
{
    uint8_t hundreds, tens, ones;
    hundreds = bite;
    tens = hundreds%100;
    hundreds -= tens;
    ones = tens%10;
    tens -= ones;
    hundreds /=100;
    tens /=10;
    
    uint32_t pattern = 0;
    pattern |= getCharBits(hundreds);
    pattern <<=7;
    pattern |= getCharBits(tens);
    pattern <<=7;
    pattern |= getCharBits(ones);
    return pattern;
}

uint64_t colorToPattern(uint8_t red, uint8_t green, uint8_t blue)
{
    uint64_t pattern = 0;
    pattern |= byteToDecPattern(red);
    pattern <<=21;
    pattern |= byteToDecPattern(green); 
    pattern <<=21;
    pattern |= byteToDecPattern(blue); 
    pattern <<=1;
    return pattern;
}

void shiftInPattern(uint64_t pattern)
{
                uint8_t i = 0; 
                for (; i<64; ++i)
                {
                        // Shift line low
                        PORTB &= ~(1<<DDB4);
                        if ((pattern>>i)&(0x00000001))
                        {
                                PORTB &= ~(1<<DDB5);
                        }
                        else
                        {
                                PORTB |= (1<<DDB5);
                        }
                        // Shift line High
                        PORTB |= (1<<DDB4);
                }
                // Latch line low
                PORTB &= ~(1<<DDB3);
                // Latch line high
                PORTB |= (1<<DDB3);
}
    

int main(void)
{
        srand(0);
        DDRB |= ((1<<DDB5)|(1<<DDB4)|(1<<DDB3));
        
        uint64_t pattern = 0x00000000;
        uint8_t i = 0;
	while (1)
	{
                //pattern = (pattern<<6);
                //pattern |= getCharBits(rand()%10);
                //pattern = (pattern<<1);
                shiftInPattern(colorToPattern(red, green, blue));
                _delay_ms(250);
                //++i;
                //if (i>9)
                //{
                //    i=0;
                //}
                red = rand()%256;
                green = rand()%256;
                blue = rand()%256;
	}
	return 0;
}
