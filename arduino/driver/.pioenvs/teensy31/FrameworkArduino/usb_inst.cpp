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

#include "WProgram.h"
#include "usb_desc.h"

#if F_CPU >= 20000000

#ifdef CDC_DATA_INTERFACE
#ifdef CDC_STATUS_INTERFACE
usb_serial_class Serial;
#endif
#endif

#ifdef MIDI_INTERFACE
usb_midi_class usbMIDI;
#endif

#ifdef KEYBOARD_INTERFACE
usb_keyboard_class Keyboard;
#endif

#ifdef MOUSE_INTERFACE
usb_mouse_class Mouse;
#endif

#ifdef RAWHID_INTERFACE
usb_rawhid_class RawHID;
#endif

#ifdef FLIGHTSIM_INTERFACE
FlightSimClass FlightSim;
#endif

#ifdef SEREMU_INTERFACE
usb_seremu_class Serial;
#endif

#ifdef JOYSTICK_INTERFACE
usb_joystick_class Joystick;
uint8_t usb_joystick_class::manual_mode = 0;
#endif

#ifdef USB_DISABLED
usb_serial_class Serial;
#endif


#else // F_CPU < 20 MHz

#if defined(USB_SERIAL) || defined(USB_SERIAL_HID)
usb_serial_class Serial;
#elif (USB_DISABLED)
usb_serial_class Serial;
#else
usb_seremu_class Serial;
#endif

#endif // F_CPU

void serialEvent() __attribute__((weak));
void serialEvent() {}
