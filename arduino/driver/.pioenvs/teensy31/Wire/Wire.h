/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

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

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

#ifndef TwoWire_h
#define TwoWire_h

#include <inttypes.h>
#include "Arduino.h"

#define BUFFER_LENGTH 32

extern "C" void i2c0_isr(void);

class TwoWire : public Stream
{
  private:
    static uint8_t rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static uint8_t txBufferIndex;
    static uint8_t txBufferLength;

    static uint8_t transmitting;
    static void onRequestService(void);
    static void onReceiveService(uint8_t*, int);
    static void (*user_onRequest)(void);
    static void (*user_onReceive)(int);
    static void sda_rising_isr(void);
    friend void i2c0_isr(void);
  public:
    TwoWire();
    void begin();
    void begin(uint8_t);
    void begin(int);
    void setClock(uint32_t);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
	virtual void flush(void);
    void onReceive( void (*)(int) );
    void onRequest( void (*)(void) );
  
#ifdef CORE_TEENSY
    // added by Teensyduino installer, for compatibility
    // with pre-1.0 sketches and libraries
    void send(uint8_t b)               { write(b); }
    void send(uint8_t *s, uint8_t n)   { write(s, n); }
    void send(int n)                   { write((uint8_t)n); }
    void send(char *s)                 { write(s); }
    uint8_t receive(void) {
        int c = read();
        if (c < 0) return 0;
        return c;
    }
#endif
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;
};

extern TwoWire Wire;

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__)
class TWBRemulation
{
public:
	inline TWBRemulation & operator = (int val) __attribute__((always_inline)) {
		if (val == 12 || val == ((F_CPU / 400000) - 16) / 2) { // 22, 52, 112
			I2C0_C1 = 0;
			#if F_BUS == 60000000
			I2C0_F = 0x1C; // 416 kHz
			#elif F_BUS == 56000000
			I2C0_F = 0x1C; // 389 kHz
			#elif F_BUS == 48000000
			I2C0_F = 0x1A; // 400 kHz
			#elif F_BUS == 40000000
			I2C0_F = 0x19; // 416 kHz
			#elif F_BUS == 36000000
			I2C0_F = 0x19; // 375 kHz
			#elif F_BUS == 24000000
			I2C0_F = 0x12; // 375 kHz
			#elif F_BUS == 16000000
			I2C0_F = 0x07; // 400 kHz
			#elif F_BUS == 8000000
			I2C0_F = 0x00; // 400 kHz
			#elif F_BUS == 4000000
			I2C0_F = 0x00; // 200 kHz
			#endif
			I2C0_C1 = I2C_C1_IICEN;
		} else if (val == 72 || val == ((F_CPU / 100000) - 16) / 2) { // 112, 232, 472
			I2C0_C1 = 0;
			#if F_BUS == 60000000
			I2C0_F = 0x2C; // 104 kHz
			#elif F_BUS == 56000000
			I2C0_F = 0x2B; // 109 kHz
			#elif F_BUS == 48000000
			I2C0_F = 0x27; // 100 kHz
			#elif F_BUS == 40000000
			I2C0_F = 0x29; // 104 kHz
			#elif F_BUS == 36000000
			I2C0_F = 0x28; // 113 kHz
			#elif F_BUS == 24000000
			I2C0_F = 0x1F; // 100 kHz
			#elif F_BUS == 16000000
			I2C0_F = 0x20; // 100 kHz
			#elif F_BUS == 8000000
			I2C0_F = 0x14; // 100 kHz
			#elif F_BUS == 4000000
			I2C0_F = 0x07; // 100 kHz
			#elif F_BUS == 2000000
			I2C0_F = 0x00; // 100 kHz
			#endif
			I2C0_C1 = I2C_C1_IICEN;
		}
		return *this;
	}
	inline operator int () const __attribute__((always_inline)) {
		#if F_BUS == 60000000
		if (I2C0_F == 0x1C) return 12;
		#elif F_BUS == 48000000
		if (I2C0_F == 0x1A) return 12;
		#elif F_BUS == 40000000
		if (I2C0_F == 0x19) return 12;
		#elif F_BUS == 36000000
		if (I2C0_F == 0x19) return 12;
		#elif F_BUS == 24000000
		if (I2C0_F == 0x12) return 12;
		#elif F_BUS == 16000000
		if (I2C0_F == 0x07) return 12;
		#elif F_BUS == 8000000
		if (I2C0_F == 0x00) return 12;
		#elif F_BUS == 4000000
		if (I2C0_F == 0x00) return 12;
		#endif
		return 72;
	}
};
extern TWBRemulation TWBR;
#endif

#endif

