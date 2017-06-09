/*
Author: Erik Andersen

Hardware:
 9 TIL729 7-segment displays. Common Anode
 8 74HC595 Shift registers
 3 10K potentiometers
 1 Common Anode RGB LED
 66 680 Ohm resistors (3 for each RGB channel, 63 for each 7-segment
    display segment)
 Tons of wires and big breadboard!

Input:
  3 Analog inputs (potentiometers) on analog pins A0-A2. Requires a
  reference on the Aref pin.
    A0: Red input
    A1: Green input
    A2: Blue input

Output:
  Digital pins 0-2 and 11-13.
  0-2: RGB LED control
    0: PWM output for red channel RGB LED. Set low for on (Common Anode)
    1: PWM output for green channel RGB LED. Set low for on (Common Anode)
    2: PWM output for blue channel RGB LED. Set low for on (Common Anode)
  11-13: Shift register control (of 8 chained 74HC595 shift registers)
    11: Display refresh (rising edge displays currently shifted data)
    12: Shift clock (rising edge advances data in shift registers)
    13: Data line for shift registers. Set low for on (TIL729 7-segment
        displays are Common Anode)
   
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

#define REDMASK (1<<DDD0)
#define GREENMASK (1<<DDD1)
#define BLUEMASK (1<<DDD2)

// Global values because we need to update the displays in main, but set them
// in the ADC read interrupt
static uint8_t red=4, green=0, blue=11;


void updateTimers(uint8_t red, uint8_t green, uint8_t blue)
{
    // If the new value is less than the timer, the timer will never go off.
    // If the new value is 0, the light may get stuck on, leading to flickering
    // around ADC values of 0
    // Remember that this is active low, so |= turns off
    if (TCNT0 >= red)
    {
        PORTD |= REDMASK;
    }
    if (TCNT0 >= green)
    {
        PORTD |= GREENMASK;
    }
    if (TCNT2 >= blue)
    {
        PORTD |= BLUEMASK;
    }
    // Update compares
    OCR0A = red;
    OCR0B = green;
    OCR2A = blue;
}

ISR(TIMER2_COMPA_vect)
{
    // End of cycle, set the bit for blue high to turn blue off
    PORTD |= BLUEMASK;
}

ISR(TIMER2_OVF_vect)
{
    // Turn on blue LED
    // Only turn on the LED if we are running a cycle of more than 0
    if (blue>0)
    {
        PORTD &= ~BLUEMASK;
    }
}

ISR(TIMER0_OVF_vect)
{
    // Turn on LEDs if we are doing a cycle more than 0
    if (red>0)
    {
        PORTD &= ~REDMASK;
    }
    if (green>0)
    {
        PORTD &= ~GREENMASK;
    }
}

ISR(TIMER0_COMPA_vect)
{
    // Turn off red LED
    PORTD |= REDMASK;
}

ISR(TIMER0_COMPB_vect)
{
    // Turn off green LED
    PORTD |= GREENMASK;
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
    else if (10 == charVal)
    {
        return MASKA;
    }
    else if (11 == charVal)
    {
        return MASKB;
    }
    else if (12 == charVal)
    {
        return MASKC;
    }
    else if (13 == charVal)
    {
        return MASKD;
    }
    else if (14 == charVal)
    {
        return MASKE;
    }
    else if (15 == charVal)
    {
        return MASKF;
    }
    else
    {
        return MASKE; // Do an E for Error
    }
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
        // Set shift line low
        PORTB &= ~(1<<DDB4);
        // If the last bit in the pattern is 1
        if ((pattern>>i)&(0x00000001))
        {
            // Set the data line low (remember we are active low)
            PORTB &= ~(1<<DDB5);
        }
        else
        {
            // Set the data line high
            PORTB |= (1<<DDB5);
        }
        // Set shift line High
        PORTB |= (1<<DDB4);
    }
    // Set display refresh line low
    PORTB &= ~(1<<DDB3);
    // Set display refresh line high
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

    // Set up Timer2
    TCNT2 = 0;
    OCR0A = blue;
    // Enable interrupts for one compare match and overflow
    TIMSK2 |= ((1<<OCIE2A)|(1<<TOIE2));
    // 1024 Prescaler
    TCCR2B |= ((1<<CS22)|(1<<CS20));
    
    // Set up ADC
    // Set REFS0 and REFS1 to 0 to use external Aref
    // Start at a single ended input of 0, so ADMUX[0-3] = 0
    ADMUX |= ((1<<ADLAR));
    // Free run mode
    ADCSRA |= ((1<<ADEN)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2));
    // ADTS[0-2] already 0, which is free running mode
    // Disable digital input buffers
    DIDR0 |= ((1<<ADC0D)|(1<<ADC1D)|(1<<ADC2D));
    // Make sure the power is on to the ADC
    PRR &= ~(1<<PRADC);
    // Start a first conversion
    ADCSRA |= (1<<ADSC);

    while (1)
    {
        shiftInPattern(colorToPattern(red, green, blue));
        _delay_ms(5);
    }
    return 0;
}
