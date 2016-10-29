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

#ifndef USBmouse_h_
#define USBmouse_h_

#include "usb_desc.h"

#if defined(MOUSE_INTERFACE)

#include <inttypes.h>

// C language implementation
#ifdef __cplusplus
extern "C" {
#endif
int usb_mouse_buttons(uint8_t left, uint8_t middle, uint8_t right);
int usb_mouse_move(int8_t x, int8_t y, int8_t wheel);
int usb_mouse_position(uint16_t x, uint16_t y);
void usb_mouse_screen_size(uint16_t width, uint16_t height, uint8_t mac);
extern uint8_t usb_mouse_buttons_state;
#ifdef __cplusplus
}
#endif


#define MOUSE_LEFT 1
#define MOUSE_MIDDLE 4
#define MOUSE_RIGHT 2
#define MOUSE_ALL (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE)

// C++ interface
#ifdef __cplusplus
class usb_mouse_class
{
        public:
        void begin(void) { }
        void end(void) { }
        void move(int8_t x, int8_t y, int8_t wheel=0) { usb_mouse_move(x, y, wheel); }
	void moveTo(uint16_t x, uint16_t y) { usb_mouse_position(x, y); }
	void screenSize(uint16_t width, uint16_t height, bool isMacintosh = false) {
		usb_mouse_screen_size(width, height, isMacintosh ? 1 : 0);
	}
        void click(uint8_t b = MOUSE_LEFT) {
		usb_mouse_buttons_state = b;
		usb_mouse_move(0, 0, 0);
		usb_mouse_buttons_state = 0;
		usb_mouse_move(0, 0, 0);
	}
        void scroll(int8_t wheel) { usb_mouse_move(0, 0, wheel); }
        void set_buttons(uint8_t left, uint8_t middle=0, uint8_t right=0) {
		usb_mouse_buttons(left, middle, right);
	}
        void press(uint8_t b = MOUSE_LEFT) {
		uint8_t buttons = usb_mouse_buttons_state | (b & MOUSE_ALL);
		if (buttons != usb_mouse_buttons_state) {
			usb_mouse_buttons_state = buttons;
			usb_mouse_move(0, 0, 0);
		}
	}
        void release(uint8_t b = MOUSE_LEFT) {
		uint8_t buttons = usb_mouse_buttons_state & ~(b & MOUSE_ALL);
		if (buttons != usb_mouse_buttons_state) {
			usb_mouse_buttons_state = buttons;
			usb_mouse_move(0, 0, 0);
		}
	}
        bool isPressed(uint8_t b = MOUSE_ALL) {
		return ((usb_mouse_buttons_state & (b & MOUSE_ALL)) != 0);
	}
};
extern usb_mouse_class Mouse;

#endif // __cplusplus

#endif // MOUSE_INTERFACE

#endif // USBmouse_h_
