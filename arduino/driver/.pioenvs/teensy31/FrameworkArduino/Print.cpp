/*
 Print.cpp - Base class that provides print() and println()
 Copyright (c) 2008 David A. Mellis.  All right reserved.
 many modifications, by Paul Stoffregen <paul@pjrc.com>
 
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
 
 Modified 23 November 2006 by David A. Mellis
 */

//#include <stdio.h>
//#include <string.h>
#include <inttypes.h>
//#include <math.h>
//#include <avr/pgmspace.h>
//#include "wiring.h"

#include "Print.h"



size_t Print::write(const uint8_t *buffer, size_t size)
{
	size_t count = 0;
	while (size--) count += write(*buffer++);
	return count;
}


size_t Print::print(const String &s)
{
	uint8_t buffer[33];
	size_t count = 0;
	unsigned int index = 0;
	unsigned int len = s.length();
	while (len > 0) {
		s.getBytes(buffer, sizeof(buffer), index);
		unsigned int nbytes = len;
		if (nbytes > sizeof(buffer)-1) nbytes = sizeof(buffer)-1;
		index += nbytes;
		len -= nbytes;
		count += write(buffer, nbytes);
	}
	return count;
}


size_t Print::print(long n)
{
	uint8_t sign=0;

	if (n < 0) {
		sign = '-';
		n = -n;
	}
	return printNumber(n, 10, sign);
}


size_t Print::println(void)
{
	uint8_t buf[2]={'\r', '\n'};
	return write(buf, 2);
}

extern "C" {
__attribute__((weak))
int _write(int file, char *ptr, int len)
{
	((class Print *)file)->write((uint8_t *)ptr, len);
	return 0;
}
}

int Print::printf(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	return vdprintf((int)this, format, ap);
}

int Print::printf(const __FlashStringHelper *format, ...)
{
	va_list ap;
	va_start(ap, format);
	return vdprintf((int)this, (const char *)format, ap);
}

#ifdef __MKL26Z64__

// optimized code inspired by Stimmer's optimization
// obviously a dit different, adapted to 32 bit Cortex-M0+
// http://forum.arduino.cc/index.php?topic=167414.msg1293679#msg1293679
// http://forum.arduino.cc/index.php?topic=167414.msg1309482#msg1309482
//  equivelant code:
//    mod = div % 10;
//    div = div / 10;
//    tmp1 = {random};
//    tmp2 = 10;
#if 1
// https://forum.pjrc.com/threads/28932-LC-is-10-9-times-slower-than-T3-1?p=76072&viewfull=1#post76072
void inline divmod10_v2(uint32_t n,uint32_t *div,uint32_t *mod) {
  uint32_t p,q;
  /* Using 32.16 fixed point representation p.q */
  /* p.q = (n+1)/512 */
  q = (n&0xFFFF) + 1;
  p = (n>>16);
  /* p.q = 51*(n+1)/512 */
  q = 13107*q;
  p = 13107*p;
  /* p.q = (1+1/2^8+1/2^16+1/2^24)*51*(n+1)/512 */
  q = q + (q>>16) + (p&0xFFFF);
  p = p + (p>>16) + (q>>16);
  /* divide by 2 */
  p = p>>1;
  *div = p;
  *mod = n-10*p;
}
#define divmod10_asm(div, mod, tmp1, tmp2, const3333) \
  divmod10_v2(div, &div, &mod);
/*
#define divmod10_asm(div, mod, tmp1, tmp2, const3333) \
asm (                              \
  " lsr   %2, %0, #16"     "\n\t"  \
  " mul   %2, %4"          "\n\t"  \
  " uxth  %1, %0"          "\n\t"  \
  " mul   %1, %4"          "\n\t"  \
  " add   %1, #1"          "\n\t"  \
  " lsr   %0, %2, #16"     "\n\t"  \
  " lsl   %2, %2, #16"     "\n\t"  \
  " add   %1, %2"          "\n\t"  \
  " mov   %3, #0"          "\n\t"  \
  " adc   %0, %3"          "\n\t"  \
  " lsl   %0, %0, #15"     "\n\t"  \
  " lsr   %2, %1, #17"     "\n\t"  \
  " orr   %0, %2"          "\n\t"  \
  " lsl   %1, %1, #15"     "\n\t"  \
  " lsr   %2, %1, #16"     "\n\t"  \
  " lsl   %3, %0, #16"     "\n\t"  \
  " orr   %2, %3"          "\n\t"  \
  " lsr   %3, %0, #16"     "\n\t"  \
  " add   %1, %0"          "\n\t"  \
  " adc   %0, %1"          "\n\t"  \
  " sub   %0, %1"          "\n\t"  \
  " add   %1, %2"          "\n\t"  \
  " adc   %0, %3"          "\n\t"  \
  " lsr   %1, %1, #4"      "\n\t"  \
  " mov   %3, #10"         "\n\t"  \
  " mul   %1, %3"          "\n\t"  \
  " lsr   %1, %1, #28"     "\n\t"  \
  : "+l" (div),                    \
    "=&l" (mod),                   \
    "=&l" (tmp1),                  \
    "=&l" (tmp2)                   \
  : "l" (const3333)                \
  :                                \
)
*/
#else
#define divmod10_asm(_div, _mod, _tmp1, _tmp2, _const3333)    \
  ({ _tmp1 = _div; _div = _div / 10; _mod = _tmp1 - _div * 10; })
  // ({_mod = _div % 10, _div = _div / 10; })
#endif


size_t Print::printNumberDec(unsigned long n, uint8_t sign)
{
        uint8_t buf[11], *p;
        uint32_t digit;
	//uint32_t t1, t2, c3333=0x3333;

        p = buf + (sizeof(buf));
        do {
		uint32_t div;
		divmod10_v2(n, &div, &digit);
		n = div;
		//divmod10_asm(n, digit, t1, t2, c3333);
                *--p = digit + '0';
        } while (n);
        if (sign) *--p = '-';
        return write(p, sizeof(buf) - (p - buf));
}

size_t Print::printNumberHex(unsigned long n)
{
        uint8_t digit, buf[8], *p;

        p = buf + (sizeof(buf));
        do {
                digit = n & 15;
                *--p = (digit < 10) ? '0' + digit : 'A' + digit - 10;
                n >>= 4;
        } while (n);
        return write(p, sizeof(buf) - (p - buf));
}

size_t Print::printNumberBin(unsigned long n)
{
        uint8_t buf[32], *p;

        p = buf + (sizeof(buf));
        do {
                *--p = '0' + ((uint8_t)n & 1);
                n >>= 1;
        } while (n);
        return write(p, sizeof(buf) - (p - buf));
}

size_t Print::printNumberAny(unsigned long n, uint8_t base)
{
        uint8_t digit, buf[21], *p;
        uint32_t tmp;

        p = buf + sizeof(buf);
        do {
                tmp = n;
                n = n / base;
                digit = tmp - n * base;
                *--p = (digit < 10) ? '0' + digit : 'A' + digit - 10;
        } while (n);
        return write(p, sizeof(buf) - (p - buf));
}

#else

size_t Print::printNumber(unsigned long n, uint8_t base, uint8_t sign)
{
	uint8_t buf[34];
	uint8_t digit, i;

	// TODO: make these checks as inline, since base is
	// almost always a constant.  base = 0 (BYTE) should
	// inline as a call directly to write()
	if (base == 0) {
		return write((uint8_t)n);
	} else if (base == 1) {
		base = 10;
	}


	if (n == 0) {
		buf[sizeof(buf) - 1] = '0';
		i = sizeof(buf) - 1;
	} else {
		i = sizeof(buf) - 1;
		while (1) {
			digit = n % base;
			buf[i] = ((digit < 10) ? '0' + digit : 'A' + digit - 10);
			n /= base;
			if (n == 0) break;
			i--;
		}
	}
	if (sign) {
		i--;
		buf[i] = '-';
	}
	return write(buf + i, sizeof(buf) - i);
}

#endif

size_t Print::printFloat(double number, uint8_t digits) 
{
	uint8_t sign=0;
	size_t count=0;

	// Handle negative numbers
	if (number < 0.0) {
		sign = 1;
		number = -number;
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	double rounding = 0.5;
	for (uint8_t i=0; i<digits; ++i) {
		rounding *= 0.1;
	}
	number += rounding;

	// Extract the integer part of the number and print it
	unsigned long int_part = (unsigned long)number;
	double remainder = number - (double)int_part;
	count += printNumber(int_part, 10, sign);

	// Print the decimal point, but only if there are digits beyond
	if (digits > 0) {
		uint8_t n, buf[16], count=1;
		buf[0] = '.';

		// Extract digits from the remainder one at a time
		if (digits > sizeof(buf) - 1) digits = sizeof(buf) - 1;

		while (digits-- > 0) {
			remainder *= 10.0;
			n = (uint8_t)(remainder);
			buf[count++] = '0' + n;
			remainder -= n; 
		}
		count += write(buf, count);
	}
	return count;
}


