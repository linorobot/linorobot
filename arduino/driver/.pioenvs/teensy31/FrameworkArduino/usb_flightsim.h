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

#ifndef USBflightsim_h_
#define USBflightsim_h_

#include "usb_desc.h"

#if defined(FLIGHTSIM_INTERFACE)

#ifdef __cplusplus

#include <inttypes.h>
#include "elapsedMillis.h"

// workaround for elapsedMillis.h bringing in WProgram.h which brings usb_undef.h
#undef USB_DESC_LIST_DEFINE
#include "usb_desc.h"

class FlightSimClass;
class FlightSimCommand;
class FlightSimInteger;

class _XpRefStr_;
#define XPlaneRef(str) ((const _XpRefStr_ *)(str))

class FlightSimClass
{
public:
	FlightSimClass();
	static void update(void);
	static bool isEnabled(void);
	static unsigned long getFrameCount(void) { return frameCount; }
private:
	static uint8_t request_id_messages;
	static uint8_t enabled;
	static elapsedMillis enableTimeout;
	static unsigned long frameCount;
	static void enable(void) { enabled = 1; enableTimeout = 0; }
	static void disable(void) { enabled = 0; }
	static void xmit(const void *p1, uint8_t n1, const void *p2, uint8_t n2);
	static void xmit_big_packet(const void *p1, uint8_t n1, const void *p2, uint8_t n2);
	friend class FlightSimCommand;
	friend class FlightSimInteger;
	friend class FlightSimFloat;
};


class FlightSimCommand
{
public:
	FlightSimCommand();
	void assign(const _XpRefStr_ *s) { name = s; if (FlightSimClass::enabled) identify(); }
	FlightSimCommand & operator = (const _XpRefStr_ *s) { assign(s); return *this; }
	void begin(void) { sendcmd(4); }
	void end(void) { sendcmd(5); }
	FlightSimCommand & operator = (int n) { sendcmd((n) ? 4 : 5); return *this; }
	void once(void) { sendcmd(6); }
	void identify(void);
private:
	unsigned int id;
	const _XpRefStr_ *name;
	void sendcmd(uint8_t n);
	FlightSimCommand *next;
	static FlightSimCommand *first;
	static FlightSimCommand *last;
	friend class FlightSimClass;
};


class FlightSimInteger
{
public:
	FlightSimInteger();
	void assign(const _XpRefStr_ *s) { name = s; if (FlightSimClass::enabled) identify(); }
	FlightSimInteger & operator = (const _XpRefStr_ *s) { assign(s); return *this; }
	void write(long val);
	FlightSimInteger & operator = (char n) { write((long)n); return *this; }
	FlightSimInteger & operator = (int n) { write((long)n); return *this; }
	FlightSimInteger & operator = (long n) { write(n); return *this; }
	FlightSimInteger & operator = (unsigned char n) { write((long)n); return *this; }
	FlightSimInteger & operator = (unsigned int n) { write((long)n); return *this; }
	FlightSimInteger & operator = (unsigned long n) { write((long)n); return *this; }
	FlightSimInteger & operator = (float n) { write((long)n); return *this; }
	FlightSimInteger & operator = (double n) { write((long)n); return *this; }
	long read(void) const { return value; }
	operator long () const { return value; }
	void identify(void);
	void update(long val);
	static FlightSimInteger * find(unsigned int n);
	void onChange(void (*fptr)(long)) { 
		hasCallbackInfo=false;
		change_callback = fptr; 
	}
	void onChange(void (*fptr)(long,void*), void* info) {
		hasCallbackInfo=true;
		change_callback = (void (*)(long))fptr;
		callbackInfo = info;
	}
	// TODO: math operators....  + - * / % ++ --
private:
	unsigned int id;
	const _XpRefStr_ *name;
	long value;
	void (*change_callback)(long);
	void* callbackInfo;
	bool  hasCallbackInfo;
	FlightSimInteger *next;
	static FlightSimInteger *first;
	static FlightSimInteger *last;
	friend class FlightSimClass;
};


class FlightSimFloat
{
public:
	FlightSimFloat();
	void assign(const _XpRefStr_ *s) { name = s; if (FlightSimClass::enabled) identify(); }
	FlightSimFloat & operator = (const _XpRefStr_ *s) { assign(s); return *this; }
	void write(float val);
	FlightSimFloat & operator = (char n) { write((float)n); return *this; }
	FlightSimFloat & operator = (int n) { write((float)n); return *this; }
	FlightSimFloat & operator = (long n) { write((float)n); return *this; }
	FlightSimFloat & operator = (unsigned char n) { write((float)n); return *this; }
	FlightSimFloat & operator = (unsigned int n) { write((float)n); return *this; }
	FlightSimFloat & operator = (unsigned long n) { write((float)n); return *this; }
	FlightSimFloat & operator = (float n) { write(n); return *this; }
	FlightSimFloat & operator = (double n) { write((float)n); return *this; }
	float read(void) const { return value; }
	operator float () const { return value; }
	void identify(void);
	void update(float val);
	static FlightSimFloat * find(unsigned int n);
	void onChange(void (*fptr)(float)) {
		hasCallbackInfo=false;
		change_callback = fptr; 
	}
	void onChange(void (*fptr)(float,void*), void* info) {
		hasCallbackInfo=true;
		change_callback = (void (*)(float))fptr;
		callbackInfo = info;
	}
	// TODO: math operators....  + - * / % ++ --
private:
	unsigned int id;
	const _XpRefStr_ *name;
	float value;
	void (*change_callback)(float);
	void* callbackInfo;
	bool  hasCallbackInfo;
	FlightSimFloat *next;
	static FlightSimFloat *first;
	static FlightSimFloat *last;
	friend class FlightSimClass;
};


class FlightSimElapsedFrames
{
private:
	unsigned long count;
public:
	FlightSimElapsedFrames(void) { count = FlightSimClass::getFrameCount(); }
	FlightSimElapsedFrames(unsigned long val) { count = FlightSimClass::getFrameCount() - val; }
	FlightSimElapsedFrames(const FlightSimElapsedFrames &orig) { count = orig.count; }
	operator unsigned long () const { return FlightSimClass::getFrameCount() - count; }
	FlightSimElapsedFrames & operator = (const FlightSimElapsedFrames &rhs) { count = rhs.count; return *this; }
	FlightSimElapsedFrames & operator = (unsigned long val) { count = FlightSimClass::getFrameCount() - val; return *this; }
	FlightSimElapsedFrames & operator -= (unsigned long val)    { count += val; return *this; }
	FlightSimElapsedFrames & operator += (unsigned long val)    { count -= val; return *this; }
	FlightSimElapsedFrames operator - (int val) const           { FlightSimElapsedFrames r(*this); r.count += val; return r; }
	FlightSimElapsedFrames operator - (unsigned int val) const  { FlightSimElapsedFrames r(*this); r.count += val; return r; }
	FlightSimElapsedFrames operator - (long val) const          { FlightSimElapsedFrames r(*this); r.count += val; return r; }
	FlightSimElapsedFrames operator - (unsigned long val) const { FlightSimElapsedFrames r(*this); r.count += val; return r; }
	FlightSimElapsedFrames operator + (int val) const           { FlightSimElapsedFrames r(*this); r.count -= val; return r; }
	FlightSimElapsedFrames operator + (unsigned int val) const  { FlightSimElapsedFrames r(*this); r.count -= val; return r; }
	FlightSimElapsedFrames operator + (long val) const          { FlightSimElapsedFrames r(*this); r.count -= val; return r; }
	FlightSimElapsedFrames operator + (unsigned long val) const { FlightSimElapsedFrames r(*this); r.count -= val; return r; }
};


extern FlightSimClass FlightSim;

#endif // __cplusplus

#endif // FLIGHTSIM_INTERFACE

#endif // USBflightsim_h_
