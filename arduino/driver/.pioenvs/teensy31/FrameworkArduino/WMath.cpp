#include <stdint.h>

static uint32_t seed;

void randomSeed(uint32_t newseed)
{
	if (newseed > 0) seed = newseed;
}

void srandom(uint32_t newseed)
{
	seed = newseed;
}

uint32_t random(void)
{
	int32_t hi, lo, x;

	// the algorithm used in avr-libc 1.6.4
	x = seed;
	if (x == 0) x = 123459876;
	hi = x / 127773;
	lo = x % 127773;
	x = 16807 * lo - 2836 * hi;
	if (x < 0) x += 0x7FFFFFFF;
	seed = x;
	return x;
}

uint32_t random(uint32_t howbig)
{
	if (howbig == 0) return 0;
	return random() % howbig;
}

int32_t random(int32_t howsmall, int32_t howbig)
{
	if (howsmall >= howbig) return howsmall;
	int32_t diff = howbig - howsmall;
	return random(diff) + howsmall;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned int makeWord(unsigned int w) { return w; }
unsigned int makeWord(unsigned char h, unsigned char l) { return (h << 8) | l; }

