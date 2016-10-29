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

#ifndef USBmidi_h_
#define USBmidi_h_

#include "usb_desc.h"

#if defined(MIDI_INTERFACE)

#include <inttypes.h>

/*
These were originally meant to allow programs written for
Francois Best's MIDI library to be easily used with
Teensy's usbMIDI which implements the same API.  However,
the MIDI library definitions have changed, so these names
now conflict.  They've never been documented (the PJRC web
page documents usbMIDI.getType() in numbers) so they are
now commented out so usbMIDI and the MIDI library can be
used together without conflict.
#define NoteOff 0
#define NoteOn 1
#define AfterTouchPoly 2
#define ControlChange 3
#define ProgramChange 4
#define AfterTouchChannel 5
#define PitchBend 6
#define SystemExclusive 7
*/

#define USB_MIDI_SYSEX_MAX 60  // maximum sysex length we can receive

// C language implementation
#ifdef __cplusplus
extern "C" {
#endif
void usb_midi_write_packed(uint32_t n);
void usb_midi_send_sysex(const uint8_t *data, uint32_t length);
void usb_midi_flush_output(void);
int usb_midi_read(uint32_t channel);
extern uint8_t usb_midi_msg_channel;
extern uint8_t usb_midi_msg_type;
extern uint8_t usb_midi_msg_data1;
extern uint8_t usb_midi_msg_data2;
extern uint8_t usb_midi_msg_sysex[USB_MIDI_SYSEX_MAX];
extern uint8_t usb_midi_msg_sysex_len;
extern void (*usb_midi_handleNoteOff)(uint8_t ch, uint8_t note, uint8_t vel);
extern void (*usb_midi_handleNoteOn)(uint8_t ch, uint8_t note, uint8_t vel);
extern void (*usb_midi_handleVelocityChange)(uint8_t ch, uint8_t note, uint8_t vel);
extern void (*usb_midi_handleControlChange)(uint8_t ch, uint8_t control, uint8_t value);
extern void (*usb_midi_handleProgramChange)(uint8_t ch, uint8_t program);
extern void (*usb_midi_handleAfterTouch)(uint8_t ch, uint8_t pressure);
extern void (*usb_midi_handlePitchChange)(uint8_t ch, int pitch);
extern void (*usb_midi_handleSysEx)(const uint8_t *data, uint16_t length, uint8_t complete);
extern void (*usb_midi_handleRealTimeSystem)(uint8_t rtb);
extern void (*usb_midi_handleTimeCodeQuarterFrame)(uint16_t data);

#ifdef __cplusplus
}
#endif

// C++ interface
#ifdef __cplusplus
class usb_midi_class
{
        public:
        void begin(void) { }
        void end(void) { }
        void sendNoteOff(uint32_t note, uint32_t velocity, uint32_t channel) __attribute__((always_inline)) {
		usb_midi_write_packed(0x8008 | (((channel - 1) & 0x0F) << 8)
		  | ((note & 0x7F) << 16) | ((velocity & 0x7F) << 24));
	}
        void sendNoteOn(uint32_t note, uint32_t velocity, uint32_t channel) __attribute__((always_inline)) {
		usb_midi_write_packed(0x9009 | (((channel - 1) & 0x0F) << 8)
		  | ((note & 0x7F) << 16) | ((velocity & 0x7F) << 24));
	}
        void sendPolyPressure(uint32_t note, uint32_t pressure, uint32_t channel) __attribute__((always_inline)) {
		usb_midi_write_packed(0xA00A | (((channel - 1) & 0x0F) << 8)
		  | ((note & 0x7F) << 16) | ((pressure & 0x7F) << 24));
	}
        void sendControlChange(uint32_t control, uint32_t value, uint32_t channel) __attribute__((always_inline)) {
		usb_midi_write_packed(0xB00B | (((channel - 1) & 0x0F) << 8)
		  | ((control & 0x7F) << 16) | ((value & 0x7F) << 24));
	}
        void sendProgramChange(uint32_t program, uint32_t channel) __attribute__((always_inline)) {
		usb_midi_write_packed(0xC00C | (((channel - 1) & 0x0F) << 8)
		  | ((program & 0x7F) << 16));
	}
        void sendAfterTouch(uint32_t pressure, uint32_t channel) __attribute__((always_inline)) {
		usb_midi_write_packed(0xD00D | (((channel - 1) & 0x0F) << 8)
		  | ((pressure & 0x7F) << 16));
	}
        void sendPitchBend(uint32_t value, uint32_t channel) __attribute__((always_inline)) {
		usb_midi_write_packed(0xE00E | (((channel - 1) & 0x0F) << 8)
		  | ((value & 0x7F) << 16) | ((value & 0x3F80) << 17));
	}
        void sendSysEx(uint32_t length, const uint8_t *data) {
		usb_midi_send_sysex(data, length);
	}
	void sendRealTime(uint32_t type) __attribute__((always_inline)) {
		uint32_t data = ( (type & 0xFF) | ((type << 8) & 0xFF00) );
		switch (type) {
			case 0xF8: // Clock
			case 0xFA: // Start
			case 0xFC: // Stop
			case 0xFB: // Continue
			case 0xFE: // ActiveSensing
			case 0xFF: // SystemReset
				usb_midi_write_packed(data);
				break;
			default: // Invalid Real Time marker
				break;
		}
	}
	void sendTimeCodeQuarterFrame(uint32_t type, uint32_t value) __attribute__((always_inline)) {
		uint32_t data = ( ((type & 0x07) << 4) | (value & 0x0F) );
		sendTimeCodeQuarterFrame(data);	
	}
        void sendTimeCodeQuarterFrame(uint32_t data) __attribute__((always_inline)) {
		usb_midi_write_packed(0xF108 | ((data & 0x7F) << 16));
	}
        void send_now(void) {
		usb_midi_flush_output();
	}
        uint8_t analog2velocity(uint16_t val, uint8_t range);
        bool read(uint8_t channel=0) {
		return usb_midi_read(channel);
	}
        inline uint8_t getType(void) {
                return usb_midi_msg_type;
        };
        inline uint8_t getChannel(void) {
                return usb_midi_msg_channel;
        };
        inline uint8_t getData1(void) {
                return usb_midi_msg_data1;
        };
        inline uint8_t getData2(void) {
                return usb_midi_msg_data2;
        };
        inline uint8_t * getSysExArray(void) {
                return usb_midi_msg_sysex;
        };
        inline void setHandleNoteOff(void (*fptr)(uint8_t channel, uint8_t note, uint8_t velocity)) {
                usb_midi_handleNoteOff = fptr;
        };
        inline void setHandleNoteOn(void (*fptr)(uint8_t channel, uint8_t note, uint8_t velocity)) {
                usb_midi_handleNoteOn = fptr;
        };
        inline void setHandleVelocityChange(void (*fptr)(uint8_t channel, uint8_t note, uint8_t velocity)) {
                usb_midi_handleVelocityChange = fptr;
        };
        inline void setHandleControlChange(void (*fptr)(uint8_t channel, uint8_t control, uint8_t value)) {
                usb_midi_handleControlChange = fptr;
        };
        inline void setHandleProgramChange(void (*fptr)(uint8_t channel, uint8_t program)) {
                usb_midi_handleProgramChange = fptr;
        };
        inline void setHandleAfterTouch(void (*fptr)(uint8_t channel, uint8_t pressure)) {
                usb_midi_handleAfterTouch = fptr;
        };
        inline void setHandlePitchChange(void (*fptr)(uint8_t channel, int pitch)) {
                usb_midi_handlePitchChange = fptr;
        };
        inline void setHandleSysEx(void (*fptr)(const uint8_t *data, uint16_t length, bool complete)) {
                usb_midi_handleSysEx = (void (*)(const uint8_t *, uint16_t, uint8_t))fptr;
        }
        inline void setHandleRealTimeSystem(void (*fptr)(uint8_t realtimebyte)) {
                usb_midi_handleRealTimeSystem = fptr;
        };
        inline void setHandleTimeCodeQuarterFrame(void (*fptr)(uint16_t data)) {
                usb_midi_handleTimeCodeQuarterFrame = fptr;
        };
	private:
};

extern usb_midi_class usbMIDI;


#endif // __cplusplus

#endif // MIDI_INTERFACE

#endif // USBmidi_h_

