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

#include "usb_dev.h"
#include "usb_mouse.h"
#include "core_pins.h" // for yield()
#include "HardwareSerial.h"
#include <string.h> // for memcpy()

#ifdef MOUSE_INTERFACE // defined by usb_dev.h -> usb_desc.h
#if F_CPU >= 20000000

// which buttons are currently pressed
uint8_t usb_mouse_buttons_state=0;

//#define DEFAULT_XRES 640
//#define DEFAULT_YRES 480

//#define DEFAULT_XRES 800
//#define DEFAULT_YRES 600

//#define DEFAULT_XRES 1024
//#define DEFAULT_YRES 768

//#define DEFAULT_XRES 1280
//#define DEFAULT_YRES 720

//#define DEFAULT_XRES 1280
//#define DEFAULT_YRES 800

#define DEFAULT_XRES 1366
#define DEFAULT_YRES 768

//#define DEFAULT_XRES 1440
//#define DEFAULT_YRES 900

//#define DEFAULT_XRES 1920
//#define DEFAULT_YRES 1080

//#define DEFAULT_XRES 2560
//#define DEFAULT_YRES 1440

//#define DEFAULT_XRES 2560
//#define DEFAULT_YRES 1600

//#define DEFAULT_XRES 2880
//#define DEFAULT_YRES 1800

//#define DEFAULT_XRES 3840
//#define DEFAULT_YRES 2160

//#define DEFAULT_XRES 7680
//#define DEFAULT_YRES 4320


#define DEFAULT_XSCALE ((0x80000000ul+DEFAULT_XRES/2)/DEFAULT_XRES)
#define DEFAULT_YSCALE ((0x80000000ul+DEFAULT_YRES/2)/DEFAULT_YRES)

static uint16_t usb_mouse_resolution_x=DEFAULT_XRES;
static uint16_t usb_mouse_resolution_y=DEFAULT_YRES;
static uint16_t usb_mouse_position_x=DEFAULT_XRES/2;
static uint16_t usb_mouse_position_y=DEFAULT_YRES/2;
static uint32_t usb_mouse_scale_x=DEFAULT_XSCALE;
static uint32_t usb_mouse_scale_y=DEFAULT_YSCALE;
static uint32_t usb_mouse_offset_x=DEFAULT_XSCALE/2-1;
static uint32_t usb_mouse_offset_y=DEFAULT_YSCALE/2-1;


// Set the mouse buttons.  To create a "click", 2 calls are needed,
// one to push the button down and the second to release it
int usb_mouse_buttons(uint8_t left, uint8_t middle, uint8_t right)
{
        uint8_t mask=0;

        if (left) mask |= 1;
        if (middle) mask |= 4;
        if (right) mask |= 2;
        usb_mouse_buttons_state = mask;
        return usb_mouse_move(0, 0, 0);
}


// Maximum number of transmit packets to queue so we don't starve other endpoints for memory
#define TX_PACKET_LIMIT 3

static uint8_t transmit_previous_timeout=0;

// When the PC isn't listening, how long do we wait before discarding data?
#define TX_TIMEOUT_MSEC 30

#if F_CPU == 168000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1100)
#elif F_CPU == 144000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 932)
#elif F_CPU == 120000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 764)
#elif F_CPU == 96000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 596)
#elif F_CPU == 72000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 512)
#elif F_CPU == 48000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 428)
#elif F_CPU == 24000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 262)
#endif


// Move the mouse.  x, y and wheel are -127 to 127.  Use 0 for no movement.
int usb_mouse_move(int8_t x, int8_t y, int8_t wheel)
{
        uint32_t wait_count=0;
        usb_packet_t *tx_packet;

        //serial_print("move");
        //serial_print("\n");
        if (x == -128) x = -127;
        if (y == -128) y = -127;
        if (wheel == -128) wheel = -127;

        while (1) {
                if (!usb_configuration) {
                        return -1;
                }
                if (usb_tx_packet_count(MOUSE_ENDPOINT) < TX_PACKET_LIMIT) {
                        tx_packet = usb_malloc();
                        if (tx_packet) break;
                }
                if (++wait_count > TX_TIMEOUT || transmit_previous_timeout) {
                        transmit_previous_timeout = 1;
                        return -1;
                }
                yield();
        }
        transmit_previous_timeout = 0;
        *(tx_packet->buf + 0) = 1;
        *(tx_packet->buf + 1) = usb_mouse_buttons_state;
        *(tx_packet->buf + 2) = x;
        *(tx_packet->buf + 3) = y;
        *(tx_packet->buf + 4) = wheel;
        tx_packet->len = 5;
        usb_tx(MOUSE_ENDPOINT, tx_packet);
        return 0;
}

int usb_mouse_position(uint16_t x, uint16_t y)
{
        uint32_t wait_count=0, val32;
        usb_packet_t *tx_packet;

	if (x >= usb_mouse_resolution_x) x = usb_mouse_resolution_x - 1;
	usb_mouse_position_x = x;
	if (y >= usb_mouse_resolution_y) y = usb_mouse_resolution_y - 1;
	usb_mouse_position_y = y;

        while (1) {
                if (!usb_configuration) {
                        return -1;
                }
                if (usb_tx_packet_count(MOUSE_ENDPOINT) < TX_PACKET_LIMIT) {
                        tx_packet = usb_malloc();
                        if (tx_packet) break;
                }
                if (++wait_count > TX_TIMEOUT || transmit_previous_timeout) {
                        transmit_previous_timeout = 1;
                        return -1;
                }
                yield();
        }
	transmit_previous_timeout = 0;
	*(tx_packet->buf + 0) = 2;
	val32 = usb_mouse_position_x * usb_mouse_scale_x + usb_mouse_offset_x;
	 //serial_print("position:");
	 //serial_phex16(usb_mouse_position_x);
	 //serial_print("->");
	 //serial_phex32(val32);
	*(tx_packet->buf + 1) = val32 >> 16;
	*(tx_packet->buf + 2) = val32 >> 24;
	val32 = usb_mouse_position_y * usb_mouse_scale_y + usb_mouse_offset_y;
	 //serial_print(",");
	 //serial_phex16(usb_mouse_position_y);
	 //serial_print("->");
	 //serial_phex32(val32);
	 //serial_print("\n");
	*(tx_packet->buf + 3) = val32 >> 16;
	*(tx_packet->buf + 4) = val32 >> 24;
	tx_packet->len = 5;
	usb_tx(MOUSE_ENDPOINT, tx_packet);
        return 0;
}

void usb_mouse_screen_size(uint16_t width, uint16_t height, uint8_t mac)
{
	if (width < 128) width = 128;
	else if (width > 7680) width = 7680;
	if (height < 128) height = 128;
	else if (height > 7680) height = 7680;
	usb_mouse_resolution_x = width;
	usb_mouse_resolution_y = height;
	usb_mouse_position_x = width / 2;
	usb_mouse_position_y = height / 2;
	usb_mouse_scale_x = (0x80000000ul + (width >> 1)) / width;
	usb_mouse_scale_y = (0x80000000ul + (height >> 1)) / height;
	usb_mouse_offset_x = (usb_mouse_scale_x >> 1) - 1;
	usb_mouse_offset_y = (usb_mouse_scale_y >> 1) - 1;
	if (mac) {
		// ugly workaround for Mac's HID coordinate scaling:
		// http://lists.apple.com/archives/usb/2011/Jun/msg00032.html
		usb_mouse_offset_x += 161061273ul;
		usb_mouse_offset_y += 161061273ul;
		usb_mouse_scale_x = (1825361101ul + (width >> 1)) / width;
		usb_mouse_scale_y = (1825361101ul + (height >> 1)) / height;
	}
}


#endif // F_CPU
#endif // MOUSE_INTERFACE
