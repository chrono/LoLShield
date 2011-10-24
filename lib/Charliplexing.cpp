/*
 * @author: Joe Stauttener
 * - Fixes for SoOnCon's 2011 Badge (8*15 matrix)
 */

/*
  Charliplexing.cpp - Using timer2 with 1ms resolution
  
  Alex Wenger <a.wenger@gmx.de> http://arduinobuch.wordpress.com/
  Matt Mets <mahto@cibomahto.com> http://cibomahto.com/
  
  Timer init code from MsTimer2 - Javier Valencia <javiervalencia80@gmail.com>
  Misc functions from Benjamin Sonnatg <benjamin@sonntag.fr>
  
  History:
    2009-12-30 - V0.0 wrote the first version at 26C3/Berlin
    2010-01-01 - V0.1 adding misc utility functions 
      (Clear, Vertical,  Horizontal) comment are Doxygen complaints now
    2010-05-27 - V0.2 add double-buffer mode

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "WProgram.h"
#include <inttypes.h>
#include <avr/interrupt.h>
#include "Charliplexing.h"

volatile unsigned int LedSign::tcnt2;

/* -----------------------------------------------------------------  */
/** Table for the LED multiplexing cycles, containing 12 cycles made out of two bytes
 */
uint8_t leds[3][54];

/// Determines whether the display is in single or double buffer mode
uint8_t displayMode;

/// Flag indicating that the display page should be flipped as soon as the
/// current frame is displayed
boolean videoFlipPage;

/// Pointer to the buffer that is currently being displayed
uint8_t* displayBuffer;

/// Pointer to the buffer that should currently be drawn to
uint8_t* workBuffer;


/* -----------------------------------------------------------------  */
/** Table for LED Position in leds[] ram table 
 */
/*const uint16_t ledMap[270] = { 
    13, 5,13, 6,13, 7,13, 8,13, 9,13,10,13,11,13,12,13, 4, 4,13,13, 3, 3,13,13, 2, 2,13,0,0,
    12, 5,12, 6,12, 7,12, 8,12, 9,12,10,12,11,12,13,12, 4, 4,12,12, 3, 3,12,12, 2, 2,12,0,0,
    11, 5,11, 6,11, 7,11, 8,11, 9,11,10,11,12,11,13,11, 4, 4,11,11, 3, 3,11,11, 2, 2,11,0,0,
    10, 5,10, 6,10, 7,10, 8,10, 9,10,11,10,12,10,13,10, 4, 4,10,10, 3, 3,10,10, 2, 2,10,0,0,
     9, 5, 9, 6, 9, 7, 9, 8, 9,10, 9,11, 9,12, 9,13, 9, 4, 4, 9, 9, 3, 3, 9, 9, 2, 2, 9,0,0,
     8, 5, 8, 6, 8, 7, 8, 9, 8,10, 8,11, 8,12, 8,13, 8, 4, 4, 8, 8, 3, 3, 8, 8, 2, 2, 8,0,0,
     7, 5, 7, 6, 7, 8, 7, 9, 7,10, 7,11, 7,12, 7,13, 7, 4, 4, 7, 7, 3, 3, 7, 7, 2, 2, 7,0,0,
     6, 5, 6, 7, 6, 8, 6, 9, 6,10, 6,11, 6,12, 6,13, 6, 4, 4, 6, 6, 3, 3, 6, 6, 2, 2, 6,0,0,
     5, 6, 5, 7, 5, 8, 5, 9, 5,10, 5,11, 5,12, 5,13, 5, 4, 4, 5, 5, 3, 3, 5, 5, 2, 2, 5,0,0,
     0,
    };*/
    
const uint16_t ledMap[270] = { 
    17, 2,17, 3,17, 4,17, 5,17, 6,17, 7,17, 8,17, 9,17,10,17,11,17,12,17,13,17,19,17,18,17,16,
    16, 2,16, 3,16, 4,16, 5,16, 6,16, 7,16, 8,16, 9,16,10,16,11,16,12,16,13,16,19,16,18, 3, 2,
    18, 2,18, 3,18, 4,18, 5,18, 6,18, 7,18, 8,18, 9,18,10,18,11,18,12,18,13,18,19, 4, 3, 4, 2,
    19, 2,19, 3,19, 4,19, 5,19, 6,19, 7,19, 8,19, 9,19,10,19,11,19,12,19,13, 5, 4, 5, 3, 5, 2,
    13, 2,13, 3,13, 4,13, 5,13, 6,13, 7,13, 8,13, 9,13,10,13,11,13,12, 6, 5, 6, 4, 6, 3, 6, 2,//12, 6,12, 5,12, 4,12, 3,
    12, 2,12, 3,12, 4,12, 5,12, 6,12, 7,12, 8,12, 9,12,10,12,11, 7, 6, 7, 5, 7, 4, 7, 3, 7, 2,
    11, 2,11, 3,11, 4,11, 5,11, 6,11, 7,11, 8,11, 9,11,10, 8, 7, 8, 6, 8, 5, 8, 4, 8, 3, 8, 2,
    10, 2,10, 3,10, 4,10, 5,10, 6,10, 7,10, 8,10, 9, 9, 8, 9, 7, 9, 6, 9, 5, 9, 4, 9, 3, 9, 2
};
     


/* -----------------------------------------------------------------  */
/** Constructor : Initialize the interrupt code. 
 * should be called in setup();
 */
void LedSign::Init(uint8_t mode)
{
	float prescaler = 0.0;
	
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)
	TIMSK2 &= ~(1<<TOIE2);
	TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
	TCCR2B &= ~(1<<WGM22);
	ASSR &= ~(1<<AS2);
	TIMSK2 &= ~(1<<OCIE2A);
	
	if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL)) {	// prescaler set to 64
		TCCR2B |= (1<<CS22);
		TCCR2B &= ~((1<<CS21) | (1<<CS20));
		prescaler = 64.0;
	} else if (F_CPU < 1000000UL) {	// prescaler set to 8
		TCCR2B |= (1<<CS21);
		TCCR2B &= ~((1<<CS22) | (1<<CS20));
		prescaler = 8.0;
	} else { // F_CPU > 16Mhz, prescaler set to 128
		TCCR2B |= ((1<<CS22) | (1<<CS20));
		TCCR2B &= ~(1<<CS21);
		prescaler = 128.0;
	}
#elif defined (__AVR_ATmega8__)
	TIMSK &= ~(1<<TOIE2);
	TCCR2 &= ~((1<<WGM21) | (1<<WGM20));
	TIMSK &= ~(1<<OCIE2);
	ASSR &= ~(1<<AS2);
	
	if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL)) {	// prescaler set to 64
		TCCR2 |= (1<<CS22);
		TCCR2 &= ~((1<<CS21) | (1<<CS20));
		prescaler = 64.0;
	} else if (F_CPU < 1000000UL) {	// prescaler set to 8
		TCCR2 |= (1<<CS21);
		TCCR2 &= ~((1<<CS22) | (1<<CS20));
		prescaler = 8.0;
	} else { // F_CPU > 16Mhz, prescaler set to 128
		TCCR2 |= ((1<<CS22) && (1<<CS20));
		TCCR2 &= ~(1<<CS21);
		prescaler = 128.0;
	}
#elif defined (__AVR_ATmega128__)
	TIMSK &= ~(1<<TOIE2);
	TCCR2 &= ~((1<<WGM21) | (1<<WGM20));
	TIMSK &= ~(1<<OCIE2);
	
	if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL)) {	// prescaler set to 64
		TCCR2 |= ((1<<CS21) | (1<<CS20));
		TCCR2 &= ~(1<<CS22);
		prescaler = 64.0;
	} else if (F_CPU < 1000000UL) {	// prescaler set to 8
		TCCR2 |= (1<<CS21);
		TCCR2 &= ~((1<<CS22) | (1<<CS20));
		prescaler = 8.0;
	} else { // F_CPU > 16Mhz, prescaler set to 256
		TCCR2 |= (1<<CS22);
		TCCR2 &= ~((1<<CS21) | (1<<CS20));
		prescaler = 256.0;
	}
#endif
	
	tcnt2 = 256 - (int)((float)F_CPU * 0.001 / prescaler);
	
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)
	TCNT2 = tcnt2;
	TIMSK2 |= (1<<TOIE2);
#elif defined (__AVR_ATmega128__)
	TCNT2 = tcnt2;
	TIMSK |= (1<<TOIE2);
#elif defined (__AVR_ATmega8__)
	TCNT2 = tcnt2;
	TIMSK |= (1<<TOIE2);
#endif

    // Record whether we are in single or double buffer mode
    displayMode = mode;

    // Point the display buffer to the first physical buffer
    displayBuffer = leds[0];

    // If we are in single buffered mode, point the work buffer
    // at the same physical buffer as the display buffer.  Otherwise,
    // point it at the second physical buffer.
    if( displayMode == SINGLE_BUFFER ) {
        workBuffer = displayBuffer;
    }
    else {
        workBuffer = leds[1];
    }

    // Clear the buffer and display it
    LedSign::Clear(0);
    LedSign::Flip(false);
}


/* -----------------------------------------------------------------  */
/** Clear the screen completely
 * @param blocking if true : wait for flip before returning, if false :
 *                 return immediately.
 */
void LedSign::Flip(bool blocking)
{
    if (displayMode == DOUBLE_BUFFER)
    {
        // Just set the flip flag, the buffer will flip between redraws
        videoFlipPage = true;

        // If we are blocking, sit here until the page flips.
        while (blocking && videoFlipPage) {
            delay(1);
        }
    }
}


/* -----------------------------------------------------------------  */
/** Clear the screen completely
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Clear(int set) {
    for(int x=0;x<15;x++)  
        for(int y=0;y<8;y++) 
            Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Clear an horizontal line completely
 * @param y is the y coordinate of the line to clear/light [0-8]
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Horizontal(int y, int set) {
    for(int x=0;x<15;x++)  
        Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Clear a vertical line completely
 * @param x is the x coordinate of the line to clear/light [0-13]
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Vertical(int x, int set) {
    for(int y=0;y<8;y++)  
        Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Set : switch on and off the leds. All the position 
 * calculations are done here, so we don't need to do in the
 * interrupt code
 */
void LedSign::Set(uint8_t x, uint8_t y, uint8_t c)
{
    uint8_t red_low  = ledMap[x*2+y*30+1];
    uint8_t red_high = ledMap[x*2+y*30+0];
    uint8_t green_low = red_high;
    uint8_t green_high = red_low;
    // pin_low is directly the address in the led array (minus 2 because the 
    // first two bytes are used for RS232 communication), but
    // as it is a two byte array we need to check pin_high also.
    // If pin_high is bigger than 8 address has to be increased by one
    
    if (c == 3) { // ORANGE
        workBuffer[(red_low-2)*3 + (red_high / 8)] |=  _BV(red_high & 0x07);   // RED ON
        workBuffer[(green_low-2)*3 + (green_high / 8)] |=  _BV(green_high & 0x07);   // GREEN ON
    } else if(c == 2) { // GREEN
        workBuffer[(red_low-2)*3 + (red_high / 8)] &= ~_BV(red_high & 0x07);   // RED OFF
        workBuffer[(green_low-2)*3 + (green_high / 8)] |=  _BV(green_high & 0x07);   // GREEN ON    
    } else if(c == 1) { // RED
        workBuffer[(red_low-2)*3 + (red_high / 8)] |=  _BV(red_high & 0x07);   // RED ON
        workBuffer[(green_low-2)*3 + (green_high / 8)] &= ~_BV(green_high & 0x07);   // GREEN OFF
    }
    else {
        workBuffer[(red_low-2)*3 + (red_high / 8)] &= ~_BV(red_high & 0x07);   // RED OFF
        workBuffer[(green_low-2)*3 + (green_high / 8)] &= ~_BV(green_high & 0x07);   // GREEN OFF
    }
}


/* -----------------------------------------------------------------  */
/** The Interrupt code goes here !  
 */
ISR(TIMER2_OVF_vect) {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)
    TCNT2 = LedSign::tcnt2;
#elif defined (__AVR_ATmega128__)
    TCNT2 = LedSign::tcnt2;
#elif defined (__AVR_ATmega8__)
    TCNT2 = LedSign::tcnt2;
#endif

    // 12 Cycles of Matrix
    static uint8_t i = 0;

    if (i < 6) {
        DDRD  = _BV(i+2) | displayBuffer[i*3];
        PORTD =            displayBuffer[i*3];

        DDRB  =            displayBuffer[i*3+1];
        PORTB =            displayBuffer[i*3+1];

        DDRC  =            displayBuffer[i*3+2];
        PORTC =            displayBuffer[i*3+2];
    } else if (i < 12) {
        DDRD  =            displayBuffer[i*3];
        PORTD =            displayBuffer[i*3];

        DDRB  = _BV(i-6) | displayBuffer[i*3+1];
        PORTB =            displayBuffer[i*3+1];

        DDRC  =            displayBuffer[i*3+2];
        PORTC =            displayBuffer[i*3+2];
    } else if (i < 14) {
        DDRB  =            displayBuffer[(i+2)*3+1];
        PORTB =            displayBuffer[(i+2)*3+1];

        DDRC  = _BV(i-12) | displayBuffer[(i+2)*3+2];
        PORTC =            displayBuffer[(i+2)*3+2];
    } else {
        DDRD =             displayBuffer[i*3];
        PORTD =            displayBuffer[i*3];

        DDRC  = _BV(i-14) | displayBuffer[i*3+2];
        PORTC =            displayBuffer[i*3+2];
    } 
    /*
       PORTB = 0xff;
       PORTD = i;
       DDRB  = 0xff;
       DDRD  = 0xff;
     */

    i++;
    if (i > 18) {
        i = 0;

        // If the page should be flipped, do it here.
        if (videoFlipPage && displayMode == DOUBLE_BUFFER)
        {
            // TODO: is this an atomic operation?
            videoFlipPage = false;

            uint8_t* temp = displayBuffer;
            displayBuffer = workBuffer;
            workBuffer = temp;
        }
    }
}

