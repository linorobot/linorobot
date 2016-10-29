/*
 *
 * MIT License:
 * Copyright (c) 2011 Adrian McEwen
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * adrianm@mcqn.com 1/1/2011
 */

#ifndef IPAddress_h
#define IPAddress_h

#include <Printable.h>

// A class to make it easier to handle and pass around IP addresses

class IPAddress : public Printable {
private:
	union {
		uint8_t bytes[4]; // IPv4 address
		uint32_t dword;
	} _address;

	// Access the raw byte array containing the address.  Because this returns a pointer
	// to the internal structure rather than a copy of the address this function should only
	// be used when you know that the usage of the returned uint8_t* will be transient and not
	// stored.
	uint8_t * raw_address() { return _address.bytes; };

public:
	// Constructors
	IPAddress() {
		_address.dword = 0;
	}
	IPAddress(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
		_address.bytes[0] = b1;
		_address.bytes[1] = b2;
		_address.bytes[2] = b3;
		_address.bytes[3] = b4;
	}
	IPAddress(uint32_t address) {
		_address.dword = address;
	}
	IPAddress(const uint8_t *address) {
		// TODO: use unaligned read on Cortex-M4
		_address.bytes[0] = *address++;
		_address.bytes[1] = *address++;
		_address.bytes[2] = *address++;
		_address.bytes[3] = *address++;
	}

	// Overloaded cast operator to allow IPAddress objects to be used where a pointer
	// to a four-byte uint8_t array is expected
	operator uint32_t () {
		return _address.dword;
	}
	bool operator==(const IPAddress& addr) {
		return _address.dword == addr._address.dword;
	}
	bool operator==(const uint8_t* addr) {
		// TODO: use unaligned read on Cortex-M4
		return (_address.bytes[0] == addr[0]
			&& _address.bytes[1] == addr[1]
			&& _address.bytes[2] == addr[2]
			&& _address.bytes[3] == addr[3]);
	}

	// Overloaded index operator to allow getting and setting individual octets of the address
	uint8_t operator[](int index) const {
		return _address.bytes[index];
	};
	uint8_t& operator[](int index) {
		return _address.bytes[index];
	};

	// Overloaded copy operators to allow initialisation of IPAddress objects from other types
	IPAddress& operator=(const uint8_t *address) {
		// TODO: use unaligned read on Cortex-M4
		_address.bytes[0] = *address++;
		_address.bytes[1] = *address++;
		_address.bytes[2] = *address++;
		_address.bytes[3] = *address++;
		return *this;
	}
	IPAddress& operator=(uint32_t address) {
		_address.dword = address;
		return *this;
	}

	virtual size_t printTo(Print& p) const;

	friend class EthernetClass;
	friend class UDP;
	friend class Client;
	friend class Server;
	friend class DhcpClass;
	friend class DNSClient;
};

const IPAddress INADDR_NONE((uint32_t)0);


#endif
