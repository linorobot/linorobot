#include "kinetis.h"
#include "ser_print.h"

#ifdef KINETISL

// Simple polling-only Serial1 support for Teensy-LC

// this is really only useful for extremely low-level troubleshooting

void ser_write(uint8_t c)
{
        while ((UART0_S1 & UART_S1_TDRE) == 0) /* wait */ ;
        UART0_D = c;
}

void ser_print(const char *p)
{
        while (*p) {
                char c = *p++;
                if (c == '\n') ser_write('\r');
                ser_write(c);
        }
}

static void ser_print_hex1(unsigned int n)
{
        n &= 15;
        if (n < 10) {
                ser_write('0' + n);
        } else {
                ser_write('A' - 10 + n);
        }
}

void ser_print_hex(unsigned int n)
{
	ser_print_hex1(n >> 4);
	ser_print_hex1(n);
}

void ser_print_hex32(unsigned int n)
{
	ser_print_hex(n >> 24);
	ser_print_hex(n >> 16);
	ser_print_hex(n >> 8);
	ser_print_hex(n);
}

#endif
