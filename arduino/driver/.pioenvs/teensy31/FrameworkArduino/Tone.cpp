/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows 
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "core_pins.h"
#include "pins_arduino.h"
#include "HardwareSerial.h"
#include "IntervalTimer.h"

// IntervalTimer based tone.  This allows tone() to share the timers with other
// libraries, rather than permanently hogging one PIT timer even for projects
// which never use tone().  Someday this single-tone implementation might be
// changed to allow multiple simultaneous tones.

static uint32_t tone_toggle_count;
static volatile uint8_t *tone_reg;
static uint8_t tone_pin=255;
static float tone_usec=0.0;
static uint32_t tone_new_count=0;
IntervalTimer tone_timer;

void tone_interrupt(void);

#if defined(KINETISK)
#define TONE_CLEAR_PIN   tone_reg[0] = 1
#define TONE_TOGGLE_PIN  tone_reg[128] = 1
#define TONE_OUTPUT_PIN  tone_reg[384] = 1

#elif defined(KINETISL)
static uint8_t tone_mask;
#define TONE_CLEAR_PIN   tone_reg[0] = tone_mask
#define TONE_TOGGLE_PIN  tone_reg[4] = tone_mask
#define TONE_OUTPUT_PIN  __disable_irq(); tone_reg[12] |= tone_mask; __enable_irq()

#endif

void tone(uint8_t pin, uint16_t frequency, uint32_t duration)
{
	uint32_t count;
	volatile uint32_t *config;
	float usec;

	if (pin >= CORE_NUM_DIGITAL) return;
	if (duration) {
		count = (frequency * duration / 1000) * 2;
		if (!(count & 1)) count++; // always full waveform cycles
	} else {
		count = 0xFFFFFFFD;
	}
	usec = (float)500000.0 / (float)frequency;
	config = portConfigRegister(pin);

	// TODO: IntervalTimer really needs an API to disable and enable
	// the interrupt on a single timer.
	__disable_irq();
	if (pin == tone_pin) {
		// changing a pin which is already playing a tone
		if (usec == tone_usec) {
			// same frequency, so just change the duration
			tone_toggle_count = (tone_toggle_count & 1) + count - 1;
		} else {
			// different frequency, reduce duration to only the
			// remainder of its current cycle, and configure for
			// the transition to the new frequency when the
			// current cycle completes
			tone_usec = usec;
			tone_new_count = count;
			tone_toggle_count = (tone_toggle_count & 1);
		}
	} else {
		// if playing on a different pin, immediately stop, even mid-cycle :-(
		if (tone_pin < CORE_NUM_DIGITAL) {
			TONE_CLEAR_PIN; // clear pin
		}
		// configure the new tone to play
		tone_pin = pin;
		tone_reg = portClearRegister(pin);
		#if defined(KINETISL)
		tone_mask = digitalPinToBitMask(pin);
		#endif
		TONE_CLEAR_PIN; // clear pin
		TONE_OUTPUT_PIN; // output mode;
		*config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		tone_toggle_count = count;
		tone_usec = usec;
		tone_timer.begin(tone_interrupt, usec);
	}
	__enable_irq();
}


void tone_interrupt(void)
{
	if (tone_toggle_count) {  // odd = rising edge, even = falling edge
		// not the end
		TONE_TOGGLE_PIN; // toggle
		tone_toggle_count--;
		if (tone_toggle_count == 0xFFFFFFFB) tone_toggle_count = 0xFFFFFFFD;
	} else {
		// this transition completes the tone
		TONE_CLEAR_PIN; // clear
		if (tone_new_count > 0) {
			// begin playing a new tone
			tone_timer.begin(tone_interrupt, tone_usec);
			tone_toggle_count = tone_new_count;
			tone_new_count = 0;
		} else {
			// finished playing
			tone_timer.end();
			tone_pin = 255;
		}
	}
}

void noTone(uint8_t pin)
{
	if (pin >= CORE_NUM_DIGITAL) return;
	__disable_irq();
	if (pin == tone_pin) {
		tone_timer.end();
		TONE_CLEAR_PIN; // clear
		tone_pin = 255;
	}
	__enable_irq();
}



