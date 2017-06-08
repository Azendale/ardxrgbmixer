/*
Author: Erik Andersen

Input:
  3 Analog inputs (potentiometers) on analog pins A0-A2. Requires a
  reference on the Aref pin.
    A0: Red input
    A1: Green input
    A2: Blue input
*/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <stdlib.h>

// Note: These patterns are for a 7-segment wiring that is *not* standard.
// This was to make wiring the 7-segments on a breadboard next to shift
// registers much easier.
// The 7 segments were connected in the following order (numbers from LSB):
//   2222  
// 33    11
// 33    11
//   4444  
// 66    00
// 66    00
//   5555  
#define PATMASK 0x7F
// Numbers
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
// HEX letters
#define MASKA 0x5F
#define MASKB 0x79
#define MASKC 0x70
#define MASKD 0x73
#define MASKE 0x7C
#define MASKF 0x5C



static uint8_t red=4, green=0, blue=11;


void updateTimers(uint8_t red, uint8_t green, uint8_t blue)
{
    // If the new value is less than the timer, the timer will never go off.
    // If the new value is 0, the light may get stuck on, leading to flickering
    // around ADC values of 0
    if (TCNT0 >= red)
    {
        PORTD |= (1<<DDD0);
    }
    if (TCNT0 >= green)
    {
        PORTD |= (1<<DDD1);
    }
    if (TCNT2 >= blue)
    {
        PORTD |= (1<<DDD2);
    }
    // Update compares
    OCR0A = red;
    OCR0B = green;
    OCR2A = blue;
}

ISR(TIMER2_COMPA_vect)
{
    // End of cycle, set the bit for blue high to turn blue off
    PORTD |= (1<<DDD2);
}

ISR(TIMER2_OVF_vect)
{
    // Turn on blue LED
    // Only turn on the LED if we are running a cycle of more than 0
    if (blue>0)
    {
        PORTD &= ~(1<<DDD2);
    }
}

ISR(TIMER0_OVF_vect)
{
    // Turn on LEDs if we are doing a cycle more than 0
    if (red>0)
    {
        PORTD &= ~(1<<DDD0);
    }
    if (green>0)
    {
        PORTD &= ~(1<<DDD1);
    }
}

ISR(TIMER0_COMPA_vect)
{
    // Turn off red LED
    PORTD |= (1<<DDD0);
}

ISR(TIMER0_COMPB_vect)
{
    // Turn off green LED
    PORTD |= (1<<DDD1);
}

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
    return 0x7C; // Do an E for Error
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

ISR(ADC_vect)
{
    if (0x01 == (ADMUX & 0x07))
    {
        red = ADCH;
        ADMUX = (ADMUX & 0xF8) | (1<<MUX1);
    }
    else if (0x02 == (ADMUX & 0x07))
    {
        green = ADCH;
        ADMUX = (ADMUX & 0xF8);
    }
    else if (0x00 == (ADMUX & 0x07))
    {
        blue = ADCH;
        ADMUX = (ADMUX & 0xF8) | (1<<MUX0);
    }
    updateTimers(red, green, blue);
}


int main(void)
{
        // Seed random generator (random color mode)
        srand(0);

        // Set up shift register control pins
        DDRB |= ((1<<DDB5)|(1<<DDB4)|(1<<DDB3));
        // Set up RGB PWM pins
        DDRD |= ((1<<DDD0)|(1<<DDD1)|(1<<DDD2));
        
        // Turn on interrupts
        sei();
        
        // Set up Timer0
        TCNT0 = 0;
        OCR0A = red;
        OCR0B = green;
        // Enable interrupts for two compare matches and overflow
        TIMSK0 |= ((1<<OCIE0B)|(1<<OCIE0A)|(1<<TOIE0));
        // 1024 Prescaler
        TCCR0B |= ((1<<CS02)|(1<<CS00));

        // Set up Timer0
        TCNT2 = 0;
        OCR0A = blue;
        // Enable interrupts for two compare matches and overflow
        TIMSK2 |= ((1<<OCIE2A)|(1<<TOIE2));
        // 1024 Prescaler
        TCCR2B |= ((1<<CS22)|(1<<CS20));
        
        // Set up ADC
        // REFS0 and REFS1 0 to use external Aref
        // Start at a single ended input of 0, so ADMUX[0-3] = 0
        ADMUX |= ((1<<ADLAR));
        // Free run mode
        ADCSRA |= ((1<<ADEN)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2));
        // ADTS[0-2] already 0, which is free running mode
        // Disable digital input buffers
        DIDR0 |= ((1<<ADC0D)|(1<<ADC1D)|(1<<ADC2D));
        // Make sure the power is on to the ADC
        PRR &= ~(1<<PRADC);
        // Start a conversion
        ADCSRA |= (1<<ADSC);

        uint64_t pattern = 0x00000000;
        uint8_t i = 0;
    while (1)
    {
                shiftInPattern(colorToPattern(red, green, blue));
                _delay_ms(5);
                // Pick a random color for now
                //red = rand()%256;
                //green = rand()%256;
                //blue = rand()%256;
    }
    return 0;
}
