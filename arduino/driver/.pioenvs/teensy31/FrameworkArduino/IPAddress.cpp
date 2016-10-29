#include "Arduino.h"
#include "IPAddress.h"

size_t IPAddress::printTo(Print& p) const
{
	int i=0;
	while (1) {
		p.print(_address.bytes[i], DEC);
		if (++i >= 4) return 4;
		p.write('.');
	}
}

