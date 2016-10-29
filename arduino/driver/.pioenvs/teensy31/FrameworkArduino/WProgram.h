#ifndef WProgram_h
#define WProgram_h

#include <stdlib.h>
#include <string.h>
#include <math.h>

// some libraries and sketches depend on this
// AVR stuff, assuming Arduino.h or WProgram.h
// automatically includes it...
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "avr_functions.h"
#include "wiring.h"
#include "HardwareSerial.h"

#define DMAMEM __attribute__ ((section(".dmabuffers"), used))
#define FASTRUN __attribute__ ((section(".fastrun"), noinline, noclone ))

#ifdef __cplusplus

#include "avr_emulation.h"
#include "usb_serial.h"
#include "usb_seremu.h"
#include "usb_keyboard.h"
#include "usb_mouse.h"
#include "usb_joystick.h"
#include "usb_midi.h"
#include "usb_rawhid.h"
#include "usb_flightsim.h"
#include "usb_mtp.h"
#include "usb_undef.h" // do not allow usb_desc.h stuff to leak to user programs

//#include "WCharacter.h"
#include "WString.h"
#include "elapsedMillis.h"
#include "IntervalTimer.h"

uint16_t makeWord(uint16_t w);
uint16_t makeWord(byte h, byte l);

#define word(...) makeWord(__VA_ARGS__)

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);

void tone(uint8_t pin, uint16_t frequency, uint32_t duration = 0);
void noTone(uint8_t pin);

// WMath prototypes
uint32_t random(void);
uint32_t random(uint32_t howbig);
int32_t random(int32_t howsmall, int32_t howbig);
void randomSeed(uint32_t newseed);
void srandom(uint32_t newseed);
long map(long, long, long, long, long);

#include "pins_arduino.h"

#endif // __cplusplus


// Fast memcpy
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK66FX1M0__)
#ifdef __cplusplus
extern "C" {
extern void *memcpy (void *dst, const void *src, size_t count);
}
#else
extern void *memcpy (void *dst, const void *src, size_t count);
#endif
#endif


#endif // WProgram_h
