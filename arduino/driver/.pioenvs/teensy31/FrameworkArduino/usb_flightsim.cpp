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
#include "usb_flightsim.h"
#include "core_pins.h" // for yield(), millis()
#include <string.h>    // for memcpy()

#ifdef FLIGHTSIM_INTERFACE // defined by usb_dev.h -> usb_desc.h
#if F_CPU >= 20000000

FlightSimCommand * FlightSimCommand::first = NULL;
FlightSimCommand * FlightSimCommand::last = NULL;
FlightSimInteger * FlightSimInteger::first = NULL;
FlightSimInteger * FlightSimInteger::last = NULL;
FlightSimFloat * FlightSimFloat::first = NULL;
FlightSimFloat * FlightSimFloat::last = NULL;

uint8_t FlightSimClass::enabled = 0;
uint8_t FlightSimClass::request_id_messages = 0;
unsigned long FlightSimClass::frameCount = 0;
elapsedMillis FlightSimClass::enableTimeout;

static unsigned int unassigned_id = 1;  // TODO: move into FlightSimClass


FlightSimCommand::FlightSimCommand()
{
	id = unassigned_id++;
	if (!first) {
		first = this;
	} else {
		last->next = this;
	}
	last = this;
	name = NULL;
	next = NULL;
	FlightSimClass::request_id_messages = 1;
}

void FlightSimCommand::identify(void)
{
	uint8_t len, buf[6];

	if (!FlightSim.enabled || !name) return;
	len = strlen((const char *)name);
	buf[0] = len + 6;
	buf[1] = 1;
	buf[2] = id;
	buf[3] = id >> 8;
	buf[4] = 0;
	buf[5] = 0;
	FlightSimClass::xmit(buf, 6, name, len);
}

void FlightSimCommand::sendcmd(uint8_t n)
{
	uint8_t buf[4];

	if (!FlightSim.enabled || !name) return;
	buf[0] = 4;
	buf[1] = n;
	buf[2] = id;
	buf[3] = id >> 8;
	FlightSimClass::xmit(buf, 4, NULL, 0);
}


FlightSimInteger::FlightSimInteger()
{
	id = unassigned_id++;
	if (!first) {
		first = this;
	} else {
		last->next = this;
	}
	last = this;
	name = NULL;
	next = NULL;
	value = 0;
	change_callback = NULL;
	callbackInfo = NULL;
	hasCallbackInfo = false;
	FlightSimClass::request_id_messages = 1;
}

void FlightSimInteger::identify(void)
{
	uint8_t len, buf[6];

	if (!FlightSim.enabled || !name) return;
	len = strlen((const char *)name);
	buf[0] = len + 6;
	buf[1] = 1;
	buf[2] = id;
	buf[3] = id >> 8;
	buf[4] = 1;
	buf[5] = 0;
	FlightSimClass::xmit(buf, 6, name, len);
}

void FlightSimInteger::write(long val)
{
	uint8_t buf[6];

	value = val;
	if (!FlightSim.enabled || !name) return; // TODO: mark as dirty
	buf[0] = 10;
	buf[1] = 2;
	buf[2] = id;
	buf[3] = id >> 8;
	buf[4] = 1;
	buf[5] = 0;
	FlightSimClass::xmit(buf, 6, (uint8_t *)&value, 4);
}

void FlightSimInteger::update(long val)
{
	value = val;
	if (change_callback) {
		if (!hasCallbackInfo) {
			(*change_callback)(val);
		} else {
			(*(void(*)(long,void*))change_callback)(val,callbackInfo);
		}
	}
}

FlightSimInteger * FlightSimInteger::find(unsigned int n)
{
	for (FlightSimInteger *p = first; p; p = p->next) {
		if (p->id == n) return p;
	}
	return NULL;
}




FlightSimFloat::FlightSimFloat()
{
	id = unassigned_id++;
	if (!first) {
		first = this;
	} else {
		last->next = this;
	}
	last = this;
	name = NULL;
	next = NULL;
	value = 0;
	change_callback = NULL;
	hasCallbackInfo = false;
	callbackInfo = NULL;
	FlightSimClass::request_id_messages = 1;
}

void FlightSimFloat::identify(void)
{
	uint8_t len, buf[6];

	if (!FlightSim.enabled || !name) return;
	len = strlen((const char *)name);
	buf[0] = len + 6;
	buf[1] = 1;
	buf[2] = id;
	buf[3] = id >> 8;
	buf[4] = 2;
	buf[5] = 0;
	FlightSimClass::xmit(buf, 6, name, len);
}

void FlightSimFloat::write(float val)
{
	uint8_t buf[6];

	value = val;
	if (!FlightSim.enabled || !name) return; // TODO: mark as dirty
	buf[0] = 10;
	buf[1] = 2;
	buf[2] = id;
	buf[3] = id >> 8;
	buf[4] = 2;
	buf[5] = 0;
	FlightSimClass::xmit(buf, 6, (uint8_t *)&value, 4);
}

void FlightSimFloat::update(float val)
{
	value = val;
	if (change_callback) { // add: JB
		if (!hasCallbackInfo) {
			(*change_callback)(val);
		} else {
			(*(void(*)(long,void*))change_callback)(val,callbackInfo);
		}
	}
}

FlightSimFloat * FlightSimFloat::find(unsigned int n)
{
	for (FlightSimFloat *p = first; p; p = p->next) {
		if (p->id == n) return p;
	}
	return NULL;
}






FlightSimClass::FlightSimClass()
{
}

void FlightSimClass::update(void)
{
	uint8_t len, maxlen, type, *p, *end;
	union {
		uint8_t b[4];
		long l;
		float f;
	} data;
	usb_packet_t *rx_packet;
	uint16_t id;

	while (1) {
		if (!usb_configuration) break;
		rx_packet = usb_rx(FLIGHTSIM_RX_ENDPOINT);
		if (!rx_packet) break;
		p = rx_packet->buf;
		end = p + 64;
		maxlen = 64;
		do {
			len = p[0];
			if (len < 2 || len > maxlen) break;
			switch (p[1]) {
			  case 0x02: // write data
				if (len < 10) break;
				id = p[2] | (p[3] << 8);
				type = p[4];
				if (type == 1) {
					FlightSimInteger *item = FlightSimInteger::find(id);
					if (!item) break;
					#ifdef KINETISK
					data.l = *(long *)(p + 6);
					#else
					data.b[0] = p[6];
					data.b[1] = p[7];
					data.b[2] = p[8];
					data.b[3] = p[9];
					#endif
					item->update(data.l);
				} else if (type == 2) {
					FlightSimFloat *item = FlightSimFloat::find(id);
					if (!item) break;
					#ifdef KINETISK
					data.f = *(float *)(p + 6);
					#else
					data.b[0] = p[6];
					data.b[1] = p[7];
					data.b[2] = p[8];
					data.b[3] = p[9];
					#endif
					item->update(data.f);
				}
				break;
			  case 0x03: // enable/disable
				if (len < 4) break;
				switch (p[2]) {
				  case 1:
					request_id_messages = 1;
					/* no break */
				  case 2:
					enable();
					frameCount++;
					break;
				  case 3:
					disable();
				}
			}
			p += len;
			maxlen -= len;
		} while (p < end);
		usb_free(rx_packet);
	}
	if (enabled && request_id_messages) {
		request_id_messages = 0;
		for (FlightSimCommand *p = FlightSimCommand::first; p; p = p->next) {
			p->identify();
		}
		for (FlightSimInteger *p = FlightSimInteger::first; p; p = p->next) {
			p->identify();
			// TODO: send any dirty data
		}
		for (FlightSimFloat *p = FlightSimFloat::first; p; p = p->next) {
			p->identify();
			// TODO: send any dirty data
		}
	}
}


bool FlightSimClass::isEnabled(void)
{
        if (!usb_configuration) return false;
        if (!enabled) return false;
	if (enableTimeout > 1500) return false;
        return true;
}


// Maximum number of transmit packets to queue so we don't starve other endpoints for memory
#define TX_PACKET_LIMIT 8

static usb_packet_t *tx_packet=NULL;
static volatile uint8_t tx_noautoflush=0;

void FlightSimClass::xmit(const void *p1, uint8_t n1, const void *p2, uint8_t n2)
{
	uint16_t total;

	total = n1 + n2;
	if (total > FLIGHTSIM_TX_SIZE) {
		xmit_big_packet(p1, n1, p2,  n2);
		return;
	}
	if (!enabled || !usb_configuration) return;
	tx_noautoflush = 1;
	if (tx_packet) {
		if (total <= FLIGHTSIM_TX_SIZE - tx_packet->index) goto send;
		for (int i = tx_packet->index; i < FLIGHTSIM_TX_SIZE; i++) {
			tx_packet->buf[i] = 0;
		}
		tx_packet->len = FLIGHTSIM_TX_SIZE;
		usb_tx(FLIGHTSIM_TX_ENDPOINT, tx_packet);
		tx_packet = NULL;
	}
	while (1) {
		if (usb_tx_packet_count(FLIGHTSIM_TX_ENDPOINT) < TX_PACKET_LIMIT) {
			tx_packet = usb_malloc();
			if (tx_packet) break;
		}
		if (!enabled || !usb_configuration) {
			tx_noautoflush = 0;
			return;
		}
		tx_noautoflush = 0;
		yield();
		tx_noautoflush = 1;
	}
send:
	memcpy(tx_packet->buf + tx_packet->index, p1, n1);
	tx_packet->index += n1;
	if (n2 > 0) {
		memcpy(tx_packet->buf + tx_packet->index, p2, n2);
		tx_packet->index += n2;
	}
	if (tx_packet->index >= FLIGHTSIM_TX_SIZE) {
		tx_packet->len = FLIGHTSIM_TX_SIZE;
		usb_tx(FLIGHTSIM_TX_ENDPOINT, tx_packet);
		tx_packet = NULL;
	}
	tx_noautoflush = 0;
}

void FlightSimClass::xmit_big_packet(const void *p1, uint8_t n1, const void *p2, uint8_t n2)
{
	if (!enabled || !usb_configuration) return;

	uint16_t remaining = n1 + n2;
	if (remaining > 255) return;
	
	bool part2 = false;
	uint8_t remainingPart1 = n1;
	const uint8_t *dataPtr = (const uint8_t*)p1;
	bool writeFragmentHeader = false;
	uint8_t fragmentCounter = 1;

	tx_noautoflush =1; // don't mess with my data, I'm working on it!

	if (tx_packet) {
		// If we have a current packet, fill it with whatever fits
		uint8_t partLen = FLIGHTSIM_TX_SIZE - tx_packet->index; 
		if (partLen > n1) partLen=n1;
		// copy first part, containing total packet length
		memcpy(tx_packet->buf + tx_packet->index, dataPtr, partLen);
		remainingPart1 -= partLen;
		tx_packet->index += partLen;
		if (remainingPart1) {
			// there still is data from the first part that
			// will go to the next packet. The boolean variable
			// part2 remains false
			remaining = remainingPart1+n2;
			dataPtr += partLen;
		} else {
			// maybe we have space for some data from the second part
			part2=true;
			partLen = FLIGHTSIM_TX_SIZE - tx_packet->index;
			// there is no need here to check whether partLen is
			// bigger than n2. It's not. If it were, all the data
			// would have fit in a single packet and xmit_big_packet
			// would never have been called...
			remaining = n2;
			if (partLen) {
				memcpy(tx_packet->buf + tx_packet->index, p2, partLen);
				remaining -= partLen;
				tx_packet->index += partLen;
			}
			dataPtr = (const uint8_t*)p2 + partLen;
		}
		// Packet padding should not be necessary, as xmit_big_packet 
		// will only be called for data that doesn't fit in a single
		// packet. So, the previous code should always fill up the
		// first packet. Right?
		for (int i = tx_packet->index; i < FLIGHTSIM_TX_SIZE; i++) {
			tx_packet->buf[i] = 0;
		}

		// queue first packet for sending
		tx_packet->len = FLIGHTSIM_TX_SIZE;
		usb_tx(FLIGHTSIM_TX_ENDPOINT, tx_packet);
		tx_packet = NULL;
		writeFragmentHeader = true;
	} else {
		remaining = n1+n2;
	}
	
	while (remaining >0) {
		while (1) {
			// get memory for next packet
			if (usb_tx_packet_count(FLIGHTSIM_TX_ENDPOINT) < TX_PACKET_LIMIT) {
				tx_packet = usb_malloc();
				if (tx_packet) {
					break;
				}
			}

			if (!enabled || !usb_configuration) {
				// teensy disconnected
				tx_noautoflush = 0;
				return;
			}
			tx_noautoflush = 0;  // you can pick up my data, if you like
			yield(); 			 // do other things and wait for memory to become free
			tx_noautoflush = 1;  // wait, I'm working on the packet data
		}

		if (writeFragmentHeader) {
			tx_packet->buf[0]=(remaining+3 <= FLIGHTSIM_TX_SIZE) ? (byte) remaining+3 : FLIGHTSIM_TX_SIZE;
			tx_packet->buf[1]=0xff;
			tx_packet->buf[2]=fragmentCounter++;
			tx_packet->index=3;
		}
		if (!part2) {
			// we still need to send the first part
			uint8_t partLen = FLIGHTSIM_TX_SIZE - tx_packet->index; 
			if (partLen > remainingPart1)
				partLen=remainingPart1;
			memcpy(tx_packet->buf + tx_packet->index, dataPtr, partLen);
			dataPtr += partLen;
			remainingPart1 -= partLen;
			tx_packet->index += partLen;
			remaining -= partLen;
			if (!remainingPart1) {
				part2=true;
				dataPtr = (const uint8_t*)p2;
			}
		}

		if (part2) {
			uint8_t partLen = FLIGHTSIM_TX_SIZE - tx_packet->index; 
			if (partLen) {
				if (partLen > remaining)
					partLen=remaining;
				memcpy(tx_packet->buf + tx_packet->index, dataPtr, partLen);
				remaining -= partLen;
				tx_packet->index += partLen;
				dataPtr += partLen;
			}
		} 
		writeFragmentHeader = true;
		
		if (tx_packet->index >= FLIGHTSIM_TX_SIZE) {
			// queue packet for sending
			tx_packet->len = FLIGHTSIM_TX_SIZE;
			usb_tx(FLIGHTSIM_TX_ENDPOINT, tx_packet);
			tx_packet = NULL;
		}
	}
	tx_noautoflush = 0;  // data is ready to be transmitted on start of USB token
}

extern "C" {
// This gets called from usb_isr when a USB start token arrives.
// If we have a packet to transmit AND transmission isn't disabled 
// by tx_noautoflush, we fill it up with zeros and send it out 
// to USB
void usb_flightsim_flush_callback(void)
{
	if (tx_noautoflush || !tx_packet || tx_packet->index == 0) return;
	for (int i=tx_packet->index; i < FLIGHTSIM_TX_SIZE; i++) {
		tx_packet->buf[i] = 0;
	}
	tx_packet->len = FLIGHTSIM_TX_SIZE;
	usb_tx(FLIGHTSIM_TX_ENDPOINT, tx_packet);
	tx_packet = NULL;
}
}

#endif // F_CPU
#endif // FLIGHTSIM_INTERFACE
