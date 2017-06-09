/*
Author: Erik Andersen <erik.b.andersen@gmail.com>
Changed: 2017-06-08 (added lots of comments)
Created: I honestly don't remember. :)
License: GPLv3

Purpose: Implement a RGB color mixer that uses potentiometers to individually
  select the values for the red, green, and blue channels.

Hardware:
 9 TIL729 7-segment displays. Common Anode
 8 74HC595 Shift registers
 3 10K potentiometers
 1 Common Anode RGB LED
 66 680 Ohm resistors (3 for each RGB channel, 63 for each 7-segment
    display segment)
 2 buttons
 Tons of wires and big breadboard!

Wiring:
 (I probably forgot something here)
 Aref pin should be connected to 5v
 Each potentiometer should have one end connected to 5v & the other to Gnd.
 Middle pin of potentiometer should go to corresponding A0-A2 pin (see inputs
 section).
 Each segment of the 7 segment display (other than decimal segment/pin) should
 be connected to a resistor (680 ohm) and that resistor should be connected to
 the corresponding output pin on the shift register. See note by the defines
 for digits to see what order the segments are connected in.
 The common anode of the 7-segment displays should be connected to 5v.
 The QH' output (pin 9) of a shift register should chain to the SER (pin 14) on
 the next register.
 The SER (pin 14) of the most significant shift register should be connected 
 to the ardiuno digital output pin 13.
 Obviously, the power and ground of the shift register should be connected. :)
 The !OE (pin 13) of the shift registers should be grounded.
 The RCLK (pin 12) of the shift registers should be connected together and to
 the ardiuno digital output pin 11.
 The SRCLK (pin 11) of the shift registers should be connected together and to
 the ardiuno digital output pin 12.
 The !SRCLR (pin 10) of each shift register should be connected to 5v.
 5v should be connected to the common anode of the RGB LED.
 The red cathode should be connected to a resistor and that resistor should be
 connected to ardiuno digital pin 0.
 The green cathode should be connected to a resistor and that resistor should be
 connected to ardiuno digital pin 1.
 The blue cathode should be connected to a resistor and that resistor should be
 connected to ardiuno digital pin 2.
 One button should have one side attached to digital pin 9. The other side
 should go to ground.
 The other button should have one side attached ot digital pin 10. The other
 side should go to ground.

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

#define SERDATAMASK (1<<DDB5)
#define SRCLKMASK (1<<DDB4)
#define REFRESHMASK (1<<DDB3)

#define BUTTON0_MASK (1<<DDB1)
#define BUTTON1_MASK (1<<DDB2)
#define BUTTONS_MASK (BUTTON0_MASK|BUTTON1_MASK)

// 0 if you want decimal output, 1 if you want hex
#define HEX 0
#define DEC 1

// Auto cycle fade mode or direct mixer mode
#define DIRECTMODE 0
#define FADEMODE 1

#define FADEMULT 10000

// States for auto fade
#define REDFADEDOWN 0
#define REDFADEUP 1
#define GREENFADEDOWN 2
#define GREENFADEUP 3
#define BLUEFADEDOWN 4
#define BLUEFADEUP 5
#define REDGREENTRANSITION 6
#define GREENBLUETRANSITION 7
#define BLUEREDTRANSITION 8

// Global values because we need to update the displays in main, but set them
// in the ADC read interrupt
static uint8_t red=4, green=0, blue=11;
static uint8_t g_displayFormat=DEC;
static uint8_t g_adc1, g_adc2, g_adc3;

// When in fade mode, the highest value any one color can get
static uint8_t g_autoCycleMaxValue=10;
// When in fade mode, the amount, when we are on a pure color, that we fade
// down toward 0
static uint8_t g_autoCyclePureFadeDown=3;
// When in fade mode, delay between steps = this*some constant 
static uint8_t g_autoCycleMult=100;
// Fade mode or direct mixer mode
static uint8_t g_mode=DIRECTMODE;

/********************************************************************************
Purpose: Generate the next RGB color in the fade
Precondition: g_autoCyclePureFadeDown < g_autoCycleMaxValue
Postcondition: Returns next fade color in the sequence in the 24 LSB
********************************************************************************/
static uint32_t fadeNextStep(void)
{
    static uint8_t fadeState=REDGREENTRANSITION;
    static uint8_t subStepFadeStep=0;
    uint8_t red, green, blue;
    if (REDFADEDOWN == fadeState)
    {
        ++subStepFadeStep;
        red = g_autoCycleMaxValue-subStepFadeStep;
        if (subStepFadeStep >= g_autoCyclePureFadeDown)
        {
            subStepFadeStep = 0;
            fadeState = REDFADEUP;
        }
    }
    else if (GREENFADEDOWN == fadeState)
    {
        ++subStepFadeStep;
        green = g_autoCycleMaxValue-subStepFadeStep;
        if (subStepFadeStep >= g_autoCyclePureFadeDown)
        {
            subStepFadeStep = 0;
            fadeState = GREENFADEUP;
        }
    }
    else if (BLUEFADEDOWN == fadeState)
    {
        ++subStepFadeStep;
        blue = g_autoCycleMaxValue-subStepFadeStep;
        if (subStepFadeStep >= g_autoCyclePureFadeDown)
        {
            subStepFadeStep = 0;
            fadeState = BLUEFADEUP;
        }
    }
    else if (REDFADEUP == fadeState)
    {
        ++subStepFadeStep;
        red = g_autoCycleMaxValue - g_autoCyclePureFadeDown + subStepFadeStep;
        if (subStepFadeStep >= g_autoCyclePureFadeDown)
        {
            subStepFadeStep = 0;
            fadeState = REDGREENTRANSITION;
        }
    }
    else if (GREENFADEUP == fadeState)
    {
        ++subStepFadeStep;
        green = g_autoCycleMaxValue - g_autoCyclePureFadeDown + subStepFadeStep;
        if (subStepFadeStep >= g_autoCyclePureFadeDown)
        {
            subStepFadeStep = 0;
            fadeState = GREENBLUETRANSITION;
        }
    }
    else if (BLUEFADEUP == fadeState)
    {
        ++subStepFadeStep;
        blue = g_autoCycleMaxValue - g_autoCyclePureFadeDown + subStepFadeStep;
        if (subStepFadeStep >= g_autoCyclePureFadeDown)
        {
            subStepFadeStep = 0;
            fadeState = BLUEREDTRANSITION;
        }
    }
    else if (REDGREENTRANSITION == fadeState)
    {
        ++subStepFadeStep;
        red = g_autoCycleMaxValue-subStepFadeStep;
        green = subStepFadeStep;
        if (subStepFadeStep >= g_autoCycleMaxValue)
        {
            subStepFadeStep = 0;
            if (g_autoCyclePureFadeDown > 0)
            {
                fadeState = GREENFADEDOWN;
            }
            else
            {
                fadeState = GREENBLUETRANSITION;
            }
        }
    }
    else if (GREENBLUETRANSITION == fadeState)
    {
        ++subStepFadeStep;
        green = g_autoCycleMaxValue-subStepFadeStep;
        blue = subStepFadeStep;
        if (subStepFadeStep >= g_autoCycleMaxValue)
        {
            subStepFadeStep = 0;
            if (g_autoCyclePureFadeDown > 0)
            {
                fadeState = BLUEFADEDOWN;
            }
            else
            {
                fadeState = BLUEREDTRANSITION;
            }
        }
    }
    else if (BLUEREDTRANSITION == fadeState)
    {
        ++subStepFadeStep;
        blue = g_autoCycleMaxValue-subStepFadeStep;
        red = subStepFadeStep;
        if (subStepFadeStep >= g_autoCycleMaxValue)
        {
            subStepFadeStep = 0;
            if (g_autoCyclePureFadeDown > 0)
            {
                fadeState = REDFADEDOWN;
            }
            else
            {
                fadeState = REDGREENTRANSITION;
            }
        }
    }
    uint32_t returnValue = 0;
    returnValue |= red;
    returnValue <<= 8;
    returnValue |= green;
    returnValue <<= 8;
    returnValue |= blue;
    return returnValue;
}

/********************************************************************************
Purpose: Update the compare values for the timers (to control PWM)
Precondition: None
Postcondition: Timer compare registers updated. RGB LED channels sometimes
 changed.
********************************************************************************/
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

/********************************************************************************
Purpose: Check to see if either of the buttons have stabilized in a new state.
Precondition: None
Postcondition: If new button stable button state, update display output format.
********************************************************************************/
static inline void debounce (void)
{
    // Track how long button 0 has been in this state
    static uint8_t button0Count = 0;
    // Track how long button 1 has been in this state
    static uint8_t button1Count = 0;
    // Bit field with the debounced states of the buttons
    static uint8_t buttons_state = 0;
    // Get the current high/low state
    uint8_t current_states = (~PINB & BUTTONS_MASK);
    // Check if the current state is a change from the stabilized state. If so,
    // and it can do that successfully for a run of four times, then let it
    // transition to the new state as the new stabilized state
    if ((current_states & BUTTON0_MASK) != (buttons_state & BUTTON0_MASK))
    {
        // Button stayed the same, so count up to track if this is a run of
        // consistentcy
        ++button0Count;
        if (button0Count >= 4)
        {
            // Button has stabilized for four checks
            if (current_states & BUTTON0_MASK)
            {
                // New stable state is down
                buttons_state |= BUTTON0_MASK;
                if (HEX == g_displayFormat)
                {
                    g_displayFormat = DEC;
                }
                else
                {
                    g_displayFormat = HEX;
                }
            }
            else
            {
                // New stable state is up
                buttons_state &= ~BUTTON0_MASK;
            }
        }
    }
    else
    {
        // Same as current state, so don't start counting it as a run of
        // changed states
        button0Count = 0;
    }
    if ((current_states & BUTTON1_MASK) != (buttons_state & BUTTON1_MASK))
    {
        // Button stayed the same, so count up to track if this is a run of
        // consistentcy
        ++button1Count;
        if (button1Count >= 4)
        {
            // Button has stabilized for four checks
            if (current_states & BUTTON1_MASK)
            {
                // New stable state is down
                buttons_state |= BUTTON1_MASK;
                if (DIRECTMODE == g_mode)
                {
                    g_mode = FADEMODE;
                }
                else
                {
                    g_mode = DIRECTMODE;
                }
            }
            else
            {
                // New stable state is up
                buttons_state &= ~BUTTON1_MASK;
            }
        }
    }
    else
    {
        // Same as current state, so don't start counting it as a run of
        // changed states
        button1Count = 0;
    }
}

/********************************************************************************
Purpose:Debounce buttons
Precondition: None
Postcondition: Display output mode possibly changed
********************************************************************************/
ISR(TIMER1_COMPA_vect)
{
    debounce();
}

/********************************************************************************
Purpose:Handle the end of the PWM cyle for blue channel.
Precondition: None
Postcondition: Blue channel turned off.
********************************************************************************/
ISR(TIMER2_COMPA_vect)
{
    // End of cycle, set the bit for blue high to turn blue off
    PORTD |= BLUEMASK;
}

/********************************************************************************
Purpose: Handle the overflow of the blue PWM timer.
Precondition: None
Postcondition: Blue channel on if blue value > 0
********************************************************************************/
ISR(TIMER2_OVF_vect)
{
    OCR2A = blue; // Update the timer in an ISR so that we have locking
    // Turn on blue LED
    // Only turn on the LED if we are running a cycle of more than 0
    if (blue>0)
    {
        PORTD &= ~BLUEMASK;
    }
}

/********************************************************************************
Purpose: Handle overflow of the Red & Green PWM timer.
Precondition: None
Postcondition: Red channel turned on if red > 0. Green channnel turned on if
 green > 0.
********************************************************************************/
ISR(TIMER0_OVF_vect)
{
    OCR0A = red; // Update the timer in an ISR so that we have locking
    OCR0B = green; // Update the timer in an ISR so that we have locking
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

/********************************************************************************
Purpose: Handle end of red PWM cycle
Precondition: None
Postcondition: Red channel turned off.
********************************************************************************/
ISR(TIMER0_COMPA_vect)
{
    // Turn off red LED
    PORTD |= REDMASK;
}

/********************************************************************************
Purpose: Handel end of green PWM cycle
Precondition: None
Postcondition: Green chennel turned off.
********************************************************************************/
ISR(TIMER0_COMPB_vect)
{
    // Turn off green LED
    PORTD |= GREENMASK;
}

/********************************************************************************
Purpose: Convert a the value for a digit of a number to the corresponding 7
 segment display bits.
Precondition: 0 <= charVal <= 15
Postcondition: Returns 7-segment bits for character
********************************************************************************/
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

/********************************************************************************
Purpose: Break a byte down into decimal representation and return the 7-segment
 display pattern to represent it.
Precondition: None
Postcondition: Returns the pattern at the LSB end of the 32 bits.
********************************************************************************/
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

/********************************************************************************
Purpose: Convert a byte into 7-segment display pattern for it's hex value.
Precondition: None
Postcondition:Returns the pattern in the LSB end of the 32 bits.
********************************************************************************/
uint32_t byteToHexPattern(uint8_t bite)
{
    uint32_t pattern = 0;
    // Skip or'ing anything in. the default 0's will be a blank digit
    pattern <<=7;
    // Grab just the top 4 bits
    pattern |= getCharBits(bite>>4);
    pattern <<=7;
    // Mask out the top 4 bits to get the last 4
    pattern |= getCharBits(bite&0x0F);
    return pattern;
}

/********************************************************************************
Purpose: Convert a color to the 7-segment display pattern
Precondition: None
Postcondition: Pattern returned in the MSB end of the 64 bits
********************************************************************************/
uint64_t colorToPattern(uint8_t red, uint8_t green, uint8_t blue)
{
    uint64_t pattern = 0;
    if (HEX == g_displayFormat)
    {
        pattern |= byteToHexPattern(red);
        pattern <<=21;
        pattern |= byteToHexPattern(green);
        pattern <<=21;
        pattern |= byteToHexPattern(blue);
    }
    else
    {
        pattern |= byteToDecPattern(red);
        pattern <<=21;
        pattern |= byteToDecPattern(green);
        pattern <<=21;
        pattern |= byteToDecPattern(blue);
    }
    pattern <<=1;
    return pattern;
}

/********************************************************************************
Purpose: Shift a 7-segment display pattern into the shift registers
Precondition: None
Postcondition: Registers updated and displaying new pattern.
********************************************************************************/
void shiftInPattern(uint64_t pattern)
{
    uint8_t i = 0; 
    for (; i<64; ++i)
    {
        // Set shift line low
        PORTB &= ~SRCLKMASK;
        // If the last bit in the pattern is 1
        if ((pattern>>i)&(0x00000001))
        {
            // Set the data line low (remember we are active low)
            PORTB &= ~SERDATAMASK;
        }
        else
        {
            // Set the data line high
            PORTB |= SERDATAMASK;
        }
        // Set shift line High
        PORTB |= SRCLKMASK;
    }
    // Set display refresh line low
    PORTB &= ~REFRESHMASK;
    // Set display refresh line high
    PORTB |= REFRESHMASK;
}

/********************************************************************************
Purpose: Read the ADC value and update PWM values, rotate to next ADC channel
Precondition:ADC ready to be read
Postcondition: red, green, or blue global variables updated; PWM timers updated,
 ADC switched to next channel.
********************************************************************************/
ISR(ADC_vect)
{
    // Figure out what we were reading by what admux is set to
    // Mask as bits except the source selection bits
    // First potentiometer
    if (0x01 == (ADMUX & 0x07))
    {
        g_adc1 = ADCH;
        // Mask out the current source selection bits, and then OR in the
        // correct ones
        ADMUX = (ADMUX & 0xF8) | (1<<MUX1);
    }
    // second potentiometer
    else if (0x02 == (ADMUX & 0x07))
    {
        g_adc2 = ADCH;
        // Mask out the current source selection bits to set to 0
        ADMUX = (ADMUX & 0xF8);
    }
    // third potentiometer
    else if (0x00 == (ADMUX & 0x07))
    {
        g_adc3 = ADCH;
        // Mask out the current source selection bits, and then OR in the
        // correct ones
        ADMUX = (ADMUX & 0xF8) | (1<<MUX0);
    }
    // Update the PWM timers
    //updateTimers(red, green, blue); // Timers now update themselves on reset
}


/********************************************************************************
Purpose: Set up board to do RGB PWM while reading values from 3 input
 potentiometers; update 7-segment displays.
Precondition: None
Postcondition: None (does not exit)
********************************************************************************/
int main(void)
{
    // Set up shift register control pins
    DDRB |= (SERDATAMASK|SRCLKMASK|REFRESHMASK);
    // Set up RGB PWM pins
    DDRD |= (REDMASK|GREENMASK|BLUEMASK);
    
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

    // Set up Timer1 to do button debounce
    // Prescale of 1024
    TCCR1B |= ((1<<CS11));
    OCR1A = 156;
    TIMSK1 |= (1 << OCIE1A);
    TCNT1 = 0;

    // Set buttons to inputs
    DDRB &= ~(BUTTONS_MASK);
    // Turn on pull up resistors for buttons
    PORTB |= BUTTONS_MASK;
    


    // Or tECH branding :)
    shiftInPattern(0xDF40078F9B2D8000);
    _delay_ms(2000);

    while (1)
    {
        if (DIRECTMODE == g_mode)
        {
            red = g_adc1;
            green = g_adc2;
            blue = g_adc3;
            shiftInPattern(colorToPattern(red, green, blue));
            _delay_ms(5);
        }
        else
        {
            float scaler = 0;
            g_autoCycleMult = g_adc1;
            if (g_adc3 > 2)
            {
                g_autoCycleMaxValue = g_adc3;
            }
            else
            {
                g_autoCycleMaxValue = 3;
            }
            scaler = g_adc2;
            scaler /=255;
            scaler *= g_autoCycleMaxValue;
            g_autoCyclePureFadeDown = scaler;
            uint32_t colors = fadeNextStep();
            red = colors>>16;
            green = (colors>>8)&0x000000FF;
            blue = colors&0x000000FF;
            //updateTimers(red, green, blue); // Timers now update themselves on reset
            shiftInPattern(colorToPattern(red, green, blue));
            uint8_t fadePause = 0;
            for (fadePause=0; fadePause < g_autoCycleMult; ++fadePause)
            {
                _delay_us(FADEMULT);
            }
        }
    }
    // Should never reach here
    return 0;
}
