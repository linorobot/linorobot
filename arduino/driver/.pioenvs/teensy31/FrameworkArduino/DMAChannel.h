#ifndef DMAChannel_h_
#define DMAChannel_h_

#include "kinetis.h"

// Discussion about DMAChannel is here:
// http://forum.pjrc.com/threads/25778-Could-there-be-something-like-an-ISR-template-function/page3

#define DMACHANNEL_HAS_BEGIN
#define DMACHANNEL_HAS_BOOLEAN_CTOR


// The channel allocation bitmask is accessible from "C" namespace,
// so C-only code can reserve DMA channels
#ifdef __cplusplus
extern "C" {
#endif
extern uint16_t dma_channel_allocated_mask;
#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

// known libraries with DMA usage (in need of porting to this new scheme):
//
// https://github.com/PaulStoffregen/Audio
// https://github.com/PaulStoffregen/OctoWS2811
// https://github.com/pedvide/ADC
// https://github.com/duff2013/SerialEvent
// https://github.com/pixelmatix/SmartMatrix
// https://github.com/crteensy/DmaSpi <-- DmaSpi has adopted this scheme


/****************************************************************/
/**                     Teensy 3.0 & 3.1                       **/
/****************************************************************/
#if defined(KINETISK)


class DMABaseClass {
public:
	typedef struct __attribute__((packed)) {
		volatile const void * volatile SADDR;
		int16_t SOFF;
		union { uint16_t ATTR;
			struct { uint8_t ATTR_DST; uint8_t ATTR_SRC; }; };
		union { uint32_t NBYTES; uint32_t NBYTES_MLNO;
			uint32_t NBYTES_MLOFFNO; uint32_t NBYTES_MLOFFYES; };
		int32_t SLAST;
		volatile void * volatile DADDR;
		int16_t DOFF;
		union { volatile uint16_t CITER;
			volatile uint16_t CITER_ELINKYES; volatile uint16_t CITER_ELINKNO; };
		int32_t DLASTSGA;
		volatile uint16_t CSR;
		union { volatile uint16_t BITER;
			volatile uint16_t BITER_ELINKYES; volatile uint16_t BITER_ELINKNO; };
	} TCD_t;
	TCD_t *TCD;

	/***************************************/
	/**    Data Transfer                  **/
	/***************************************/

	// Use a single variable as the data source.  Typically a register
	// for receiving data from one of the hardware peripherals is used.
	void source(volatile const signed char &p) { source(*(volatile const uint8_t *)&p); }
	void source(volatile const unsigned char &p) {
		TCD->SADDR = &p;
		TCD->SOFF = 0;
		TCD->ATTR_SRC = 0;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 1;
		TCD->SLAST = 0;
	}
	void source(volatile const signed short &p) { source(*(volatile const uint16_t *)&p); }
	void source(volatile const unsigned short &p) {
		TCD->SADDR = &p;
		TCD->SOFF = 0;
		TCD->ATTR_SRC = 1;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 2;
		TCD->SLAST = 0;
	}
	void source(volatile const signed int &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const unsigned int &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const signed long &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const unsigned long &p) {
		TCD->SADDR = &p;
		TCD->SOFF = 0;
		TCD->ATTR_SRC = 2;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 4;
		TCD->SLAST = 0;
	}

	// Use a buffer (array of data) as the data source.  Typically a
	// buffer for transmitting data is used.
	void sourceBuffer(volatile const signed char p[], unsigned int len) {
		sourceBuffer((volatile const uint8_t *)p, len); }
	void sourceBuffer(volatile const unsigned char p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 1;
		TCD->ATTR_SRC = 0;
		TCD->NBYTES = 1;
		TCD->SLAST = -len;
		TCD->BITER = len;
		TCD->CITER = len;
	}
	void sourceBuffer(volatile const signed short p[], unsigned int len) {
		sourceBuffer((volatile const uint16_t *)p, len); }
	void sourceBuffer(volatile const unsigned short p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 2;
		TCD->ATTR_SRC = 1;
		TCD->NBYTES = 2;
		TCD->SLAST = -len;
		TCD->BITER = len / 2;
		TCD->CITER = len / 2;
	}
	void sourceBuffer(volatile const signed int p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const unsigned int p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const signed long p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const unsigned long p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 4;
		TCD->ATTR_SRC = 2;
		TCD->NBYTES = 4;
		TCD->SLAST = -len;
		TCD->BITER = len / 4;
		TCD->CITER = len / 4;
	}

	// Use a circular buffer as the data source
	void sourceCircular(volatile const signed char p[], unsigned int len) {
		sourceCircular((volatile const uint8_t *)p, len); }
	void sourceCircular(volatile const unsigned char p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 1;
		TCD->ATTR_SRC = ((31 - __builtin_clz(len)) << 3);
		TCD->NBYTES = 1;
		TCD->SLAST = 0;
		TCD->BITER = len;
		TCD->CITER = len;
	}
	void sourceCircular(volatile const signed short p[], unsigned int len) {
		sourceCircular((volatile const uint16_t *)p, len); }
	void sourceCircular(volatile const unsigned short p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 2;
		TCD->ATTR_SRC = ((31 - __builtin_clz(len)) << 3) | 1;
		TCD->NBYTES = 2;
		TCD->SLAST = 0;
		TCD->BITER = len / 2;
		TCD->CITER = len / 2;
	}
	void sourceCircular(volatile const signed int p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const unsigned int p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const signed long p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const unsigned long p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 4;
		TCD->ATTR_SRC = ((31 - __builtin_clz(len)) << 3) | 2;
		TCD->NBYTES = 4;
		TCD->SLAST = 0;
		TCD->BITER = len / 4;
		TCD->CITER = len / 4;
	}

	// Use a single variable as the data destination.  Typically a register
	// for transmitting data to one of the hardware peripherals is used.
	void destination(volatile signed char &p) { destination(*(volatile uint8_t *)&p); }
	void destination(volatile unsigned char &p) {
		TCD->DADDR = &p;
		TCD->DOFF = 0;
		TCD->ATTR_DST = 0;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 1;
		TCD->DLASTSGA = 0;
	}
	void destination(volatile signed short &p) { destination(*(volatile uint16_t *)&p); }
	void destination(volatile unsigned short &p) {
		TCD->DADDR = &p;
		TCD->DOFF = 0;
		TCD->ATTR_DST = 1;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 2;
		TCD->DLASTSGA = 0;
	}
	void destination(volatile signed int &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile unsigned int &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile signed long &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile unsigned long &p) {
		TCD->DADDR = &p;
		TCD->DOFF = 0;
		TCD->ATTR_DST = 2;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 4;
		TCD->DLASTSGA = 0;
	}

	// Use a buffer (array of data) as the data destination.  Typically a
	// buffer for receiving data is used.
	void destinationBuffer(volatile signed char p[], unsigned int len) {
		destinationBuffer((volatile uint8_t *)p, len); }
	void destinationBuffer(volatile unsigned char p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 1;
		TCD->ATTR_DST = 0;
		TCD->NBYTES = 1;
		TCD->DLASTSGA = -len;
		TCD->BITER = len;
		TCD->CITER = len;
	}
	void destinationBuffer(volatile signed short p[], unsigned int len) {
		destinationBuffer((volatile uint16_t *)p, len); }
	void destinationBuffer(volatile unsigned short p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 2;
		TCD->ATTR_DST = 1;
		TCD->NBYTES = 2;
		TCD->DLASTSGA = -len;
		TCD->BITER = len / 2;
		TCD->CITER = len / 2;
	}
	void destinationBuffer(volatile signed int p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile unsigned int p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile signed long p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile unsigned long p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 4;
		TCD->ATTR_DST = 2;
		TCD->NBYTES = 4;
		TCD->DLASTSGA = -len;
		TCD->BITER = len / 4;
		TCD->CITER = len / 4;
	}

	// Use a circular buffer as the data destination
	void destinationCircular(volatile signed char p[], unsigned int len) {
		destinationCircular((volatile uint8_t *)p, len); }
	void destinationCircular(volatile unsigned char p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 1;
		TCD->ATTR_DST = ((31 - __builtin_clz(len)) << 3);
		TCD->NBYTES = 1;
		TCD->DLASTSGA = 0;
		TCD->BITER = len;
		TCD->CITER = len;
	}
	void destinationCircular(volatile signed short p[], unsigned int len) {
		destinationCircular((volatile uint16_t *)p, len); }
	void destinationCircular(volatile unsigned short p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 2;
		TCD->ATTR_DST = ((31 - __builtin_clz(len)) << 3) | 1;
		TCD->NBYTES = 2;
		TCD->DLASTSGA = 0;
		TCD->BITER = len / 2;
		TCD->CITER = len / 2;
	}
	void destinationCircular(volatile signed int p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile unsigned int p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile signed long p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile unsigned long p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 4;
		TCD->ATTR_DST = ((31 - __builtin_clz(len)) << 3) | 2;
		TCD->NBYTES = 4;
		TCD->DLASTSGA = 0;
		TCD->BITER = len / 4;
		TCD->CITER = len / 4;
	}

	/*************************************************/
	/**    Quantity of Data to Transfer             **/
	/*************************************************/

	// Set the data size used for each triggered transfer
	void transferSize(unsigned int len) {
		if (len == 16) {
			TCD->NBYTES = 16;
			if (TCD->SOFF != 0) TCD->SOFF = 16;
			if (TCD->DOFF != 0) TCD->DOFF = 16;
			TCD->ATTR = (TCD->ATTR & 0xF8F8) | 0x0404;
		} else if (len == 4) {
			TCD->NBYTES = 4;
			if (TCD->SOFF != 0) TCD->SOFF = 4;
			if (TCD->DOFF != 0) TCD->DOFF = 4;
			TCD->ATTR = (TCD->ATTR & 0xF8F8) | 0x0202;
		} else if (len == 2) {
			TCD->NBYTES = 2;
			if (TCD->SOFF != 0) TCD->SOFF = 2;
			if (TCD->DOFF != 0) TCD->DOFF = 2;
			TCD->ATTR = (TCD->ATTR & 0xF8F8) | 0x0101;
		} else {
			TCD->NBYTES = 1;
			if (TCD->SOFF != 0) TCD->SOFF = 1;
			if (TCD->DOFF != 0) TCD->DOFF = 1;
			TCD->ATTR = TCD->ATTR & 0xF8F8;
		}
	}

	// Set the number of transfers (number of triggers until complete)
	void transferCount(unsigned int len) {
		if (len > 32767) return;
		if (len >= 512) {
			TCD->BITER = len;
			TCD->CITER = len;
		} else {
			TCD->BITER = (TCD->BITER & 0xFE00) | len;
			TCD->CITER = (TCD->CITER & 0xFE00) | len;
		}
	}

	/*************************************************/
	/**    Special Options / Features               **/
	/*************************************************/

	void interruptAtCompletion(void) {
		TCD->CSR |= DMA_TCD_CSR_INTMAJOR;
	}

	void interruptAtHalf(void) {
		TCD->CSR |= DMA_TCD_CSR_INTHALF;
	}

	void disableOnCompletion(void) {
		TCD->CSR |= DMA_TCD_CSR_DREQ;
	}

	void replaceSettingsOnCompletion(const DMABaseClass &settings) {
		TCD->DLASTSGA = (int32_t)(settings.TCD);
		TCD->CSR &= ~DMA_TCD_CSR_DONE;
		TCD->CSR |= DMA_TCD_CSR_ESG;
	}

protected:
	// users should not be able to create instances of DMABaseClass, which
	// require the inheriting class to initialize the TCD pointer.
	DMABaseClass() {}

	static inline void copy_tcd(TCD_t *dst, const TCD_t *src) {
		const uint32_t *p = (const uint32_t *)src;
		uint32_t *q = (uint32_t *)dst;
		uint32_t t1, t2, t3, t4;
		t1 = *p++; t2 = *p++; t3 = *p++; t4 = *p++;
		*q++ = t1; *q++ = t2; *q++ = t3; *q++ = t4;
		t1 = *p++; t2 = *p++; t3 = *p++; t4 = *p++;
		*q++ = t1; *q++ = t2; *q++ = t3; *q++ = t4;
	}
};


// DMASetting represents settings stored only in memory, which can be
// applied to any DMA channel.

class DMASetting : public DMABaseClass {
public:
	DMASetting() {
		TCD = &tcddata;
	}
	DMASetting(const DMASetting &c) {
		TCD = &tcddata;
		*this = c;
	}
	DMASetting(const DMABaseClass &c) {
		TCD = &tcddata;
		*this = c;
	}
	DMASetting & operator = (const DMABaseClass &rhs) {
		copy_tcd(TCD, rhs.TCD);
		return *this;
	}
private:
	TCD_t tcddata __attribute__((aligned(32)));
};


// DMAChannel reprents an actual DMA channel and its current settings

class DMAChannel : public DMABaseClass {
public:
	/*************************************************/
	/**    Channel Allocation                       **/
	/*************************************************/

	DMAChannel() {
		begin();
	}
	DMAChannel(const DMAChannel &c) {
		TCD = c.TCD;
		channel = c.channel;
	}
	DMAChannel(const DMASetting &c) {
		begin();
		copy_tcd(TCD, c.TCD);
	}
	DMAChannel(bool allocate) {
		if (allocate) begin();
	}
	DMAChannel & operator = (const DMAChannel &rhs) {
		if (channel != rhs.channel) {
			release();
			TCD = rhs.TCD;
			channel = rhs.channel;
		}
		return *this;
	}
	DMAChannel & operator = (const DMASetting &rhs) {
		copy_tcd(TCD, rhs.TCD);
		return *this;
	}
	~DMAChannel() {
		release();
	}
	void begin(bool force_initialization = false);
private:
	void release(void);

public:
	/***************************************/
	/**    Triggering                     **/
	/***************************************/

	// Triggers cause the DMA channel to actually move data.  Each
	// trigger moves a single data unit, which is typically 8, 16 or
	// 32 bits.  If a channel is configured for 200 transfers

	// Use a hardware trigger to make the DMA channel run
	void triggerAtHardwareEvent(uint8_t source) {
		volatile uint8_t *mux;
		mux = (volatile uint8_t *)&(DMAMUX0_CHCFG0) + channel;
		*mux = 0;
		*mux = (source & 63) | DMAMUX_ENABLE;
	}

	// Use another DMA channel as the trigger, causing this
	// channel to trigger after each transfer is makes, except
	// the its last transfer.  This effectively makes the 2
	// channels run in parallel until the last transfer
	void triggerAtTransfersOf(DMABaseClass &ch) {
		ch.TCD->BITER = (ch.TCD->BITER & ~DMA_TCD_BITER_ELINKYES_LINKCH_MASK)
		  | DMA_TCD_BITER_ELINKYES_LINKCH(channel) | DMA_TCD_BITER_ELINKYES_ELINK;
		ch.TCD->CITER = ch.TCD->BITER ;
	}

	// Use another DMA channel as the trigger, causing this
	// channel to trigger when the other channel completes.
	void triggerAtCompletionOf(DMABaseClass &ch) {
		ch.TCD->CSR = (ch.TCD->CSR & ~(DMA_TCD_CSR_MAJORLINKCH_MASK|DMA_TCD_CSR_DONE))
		  | DMA_TCD_CSR_MAJORLINKCH(channel) | DMA_TCD_CSR_MAJORELINK;
	}

	// Cause this DMA channel to be continuously triggered, so
	// it will move data as rapidly as possible, without waiting.
	// Normally this would be used with disableOnCompletion().
	void triggerContinuously(void) {
		volatile uint8_t *mux = (volatile uint8_t *)&DMAMUX0_CHCFG0;
		mux[channel] = 0;
#if DMAMUX_NUM_SOURCE_ALWAYS >= DMA_NUM_CHANNELS
		mux[channel] = DMAMUX_SOURCE_ALWAYS0 + channel;	
#else
		// search for an unused "always on" source
		unsigned int i = DMAMUX_SOURCE_ALWAYS0;
		for (i = DMAMUX_SOURCE_ALWAYS0;
		  i < DMAMUX_SOURCE_ALWAYS0 + DMAMUX_NUM_SOURCE_ALWAYS; i++) {
			unsigned int ch;
			for (ch=0; ch < DMA_NUM_CHANNELS; ch++) {
				if (mux[ch] == i) break;
			}
			if (ch >= DMA_NUM_CHANNELS) {
				mux[channel] = (i | DMAMUX_ENABLE);
				return;
			}
		}
#endif
	}

	// Manually trigger the DMA channel.
	void triggerManual(void) {
		DMA_SSRT = channel;
	}


	/***************************************/
	/**    Interrupts                     **/
	/***************************************/

	// An interrupt routine can be run when the DMA channel completes
	// the entire transfer, and also optionally when half of the
	// transfer is completed.
	void attachInterrupt(void (*isr)(void)) {
		_VectorsRam[channel + IRQ_DMA_CH0 + 16] = isr;
		NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + channel);
	}

	void detachInterrupt(void) {
		NVIC_DISABLE_IRQ(IRQ_DMA_CH0 + channel);
	}

	void clearInterrupt(void) {
		DMA_CINT = channel;
	}


	/***************************************/
	/**    Enable / Disable               **/
	/***************************************/

	void enable(void) {
		DMA_SERQ = channel;
	}
	void disable(void) {
		DMA_CERQ = channel;
	}

	/***************************************/
	/**    Status                         **/
	/***************************************/

	bool complete(void) {
		if (TCD->CSR & DMA_TCD_CSR_DONE) return true;
		return false;
	}
	void clearComplete(void) {
		DMA_CDNE = channel;
	}
	bool error(void) {
		if (DMA_ERR & (1<<channel)) return true;
		return false;
	}
	void clearError(void) {
		DMA_CERR = channel;
	}
	void * sourceAddress(void) {
		return (void *)(TCD->SADDR);
	}
	void * destinationAddress(void) {
		return (void *)(TCD->DADDR);
	}

	/***************************************/
	/**    Direct Hardware Access         **/
	/***************************************/

	// For complex and unusual configurations not possible with the above
	// functions, the Transfer Control Descriptor (TCD) and channel number
	// can be used directly.  This leads to less portable and less readable
	// code, but direct control of all parameters is possible.
	uint8_t channel;
	// TCD is accessible due to inheritance from DMABaseClass
};

// arrange the relative priority of 2 or more DMA channels
void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2);
void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2, DMAChannel &ch3);
void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2, DMAChannel &ch3, DMAChannel &ch4);

















/****************************************************************/
/**                          Teensy-LC                         **/
/****************************************************************/
#elif defined(KINETISL)


class DMABaseClass {
public:
	typedef struct __attribute__((packed)) {
		volatile const void * volatile SAR;
		volatile void * volatile       DAR;
		volatile uint32_t              DSR_BCR;
		volatile uint32_t              DCR;
	} CFG_t;
	CFG_t *CFG;

	/***************************************/
	/**    Data Transfer                  **/
	/***************************************/

	// Use a single variable as the data source.  Typically a register
	// for receiving data from one of the hardware peripherals is used.
	void source(volatile const signed char &p) { source(*(volatile const uint8_t *)&p); }
	void source(volatile const unsigned char &p) {
		CFG->SAR = &p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(1);
	}
	void source(volatile const signed short &p) { source(*(volatile const uint16_t *)&p); }
	void source(volatile const unsigned short &p) {
		CFG->SAR = &p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(2);
	}
	void source(volatile const signed int &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const unsigned int &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const signed long &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const unsigned long &p) {
		CFG->SAR = &p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(0);
	}

	// Use a buffer (array of data) as the data source.  Typically a
	// buffer for transmitting data is used.
	void sourceBuffer(volatile const signed char p[], unsigned int len) {
		sourceBuffer((volatile const uint8_t *)p, len); }
	void sourceBuffer(volatile const unsigned char p[], unsigned int len) {
		if (len > 0xFFFFF) return;
		CFG->SAR = p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(1) | DMA_DCR_SINC;
		CFG->DSR_BCR = len;
	}
	void sourceBuffer(volatile const signed short p[], unsigned int len) {
		sourceBuffer((volatile const uint16_t *)p, len); }
	void sourceBuffer(volatile const unsigned short p[], unsigned int len) {
		if (len > 0xFFFFF) return;
		CFG->SAR = p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(2) | DMA_DCR_SINC;
		CFG->DSR_BCR = len;
	}
	void sourceBuffer(volatile const signed int p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const unsigned int p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const signed long p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const unsigned long p[], unsigned int len) {
		if (len > 0xFFFFF) return;
		CFG->SAR = p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(0) | DMA_DCR_SINC;
		CFG->DSR_BCR = len;
	}

	// Use a circular buffer as the data source
	void sourceCircular(volatile const signed char p[], unsigned int len) {
		sourceCircular((volatile const uint8_t *)p, len); }
	void sourceCircular(volatile const unsigned char p[], unsigned int len) {
		uint32_t mod = len2mod(len);
		if (mod == 0) return;
		CFG->SAR = p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(1) | DMA_DCR_SINC
			| DMA_DCR_SMOD(mod);
		CFG->DSR_BCR = len;
	}
	void sourceCircular(volatile const signed short p[], unsigned int len) {
		sourceCircular((volatile const uint16_t *)p, len); }
	void sourceCircular(volatile const unsigned short p[], unsigned int len) {
		uint32_t mod = len2mod(len);
		if (mod == 0) return;
		CFG->SAR = p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(2) | DMA_DCR_SINC
			| DMA_DCR_SMOD(mod);
		CFG->DSR_BCR = len;
	}
	void sourceCircular(volatile const signed int p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const unsigned int p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const signed long p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const unsigned long p[], unsigned int len) {
		uint32_t mod = len2mod(len);
		if (mod == 0) return;
		CFG->SAR = p;
		CFG->DCR = (CFG->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(0) | DMA_DCR_SINC
			| DMA_DCR_SMOD(mod);
		CFG->DSR_BCR = len;
	}

	// Use a single variable as the data destination.  Typically a register
	// for transmitting data to one of the hardware peripherals is used.
	void destination(volatile signed char &p) { destination(*(volatile uint8_t *)&p); }
	void destination(volatile unsigned char &p) {
		CFG->DAR = &p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(1);
	}
	void destination(volatile signed short &p) { destination(*(volatile uint16_t *)&p); }
	void destination(volatile unsigned short &p) {
		CFG->DAR = &p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(2);
	}
	void destination(volatile signed int &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile unsigned int &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile signed long &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile unsigned long &p) {
		CFG->DAR = &p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(0);
	}

	// Use a buffer (array of data) as the data destination.  Typically a
	// buffer for receiving data is used.
	void destinationBuffer(volatile signed char p[], unsigned int len) {
		destinationBuffer((volatile uint8_t *)p, len); }
	void destinationBuffer(volatile unsigned char p[], unsigned int len) {
		CFG->DAR = p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(1) | DMA_DCR_DINC;
		CFG->DSR_BCR = len;
	}
	void destinationBuffer(volatile signed short p[], unsigned int len) {
		destinationBuffer((volatile uint16_t *)p, len); }
	void destinationBuffer(volatile unsigned short p[], unsigned int len) {
		CFG->DAR = p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(2) | DMA_DCR_DINC;
		CFG->DSR_BCR = len;
	}
	void destinationBuffer(volatile signed int p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile unsigned int p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile signed long p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile unsigned long p[], unsigned int len) {
		CFG->DAR = p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(0) | DMA_DCR_DINC;
		CFG->DSR_BCR = len;
	}

	// Use a circular buffer as the data destination
	void destinationCircular(volatile signed char p[], unsigned int len) {
		destinationCircular((volatile uint8_t *)p, len); }
	void destinationCircular(volatile unsigned char p[], unsigned int len) {
		uint32_t mod = len2mod(len);
		if (mod == 0) return;
		CFG->DAR = p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(1) | DMA_DCR_DINC
			| DMA_DCR_DMOD(mod);
		CFG->DSR_BCR = len;
	}
	void destinationCircular(volatile signed short p[], unsigned int len) {
		destinationCircular((volatile uint16_t *)p, len); }
	void destinationCircular(volatile unsigned short p[], unsigned int len) {
		uint32_t mod = len2mod(len);
		if (mod == 0) return;
		CFG->DAR = p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(1) | DMA_DCR_DINC
			| DMA_DCR_DMOD(mod);
		CFG->DSR_BCR = len;
	}
	void destinationCircular(volatile signed int p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile unsigned int p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile signed long p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile unsigned long p[], unsigned int len) {
		uint32_t mod = len2mod(len);
		if (mod == 0) return;
		CFG->DAR = p;
		CFG->DCR = (CFG->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(1) | DMA_DCR_DINC
			| DMA_DCR_DMOD(mod);
		CFG->DSR_BCR = len;
	}

	/*************************************************/
	/**    Quantity of Data to Transfer             **/
	/*************************************************/

	// Set the data size used for each triggered transfer
	void transferSize(unsigned int len) {
		uint32_t dcr = CFG->DCR & 0xF0C8FFFF;
		if (len == 4) {
			CFG->DCR = dcr | DMA_DCR_DSIZE(0) | DMA_DCR_DSIZE(0);
		} else if (len == 2) {
			CFG->DCR = dcr | DMA_DCR_DSIZE(2) | DMA_DCR_DSIZE(2);
		} else {
			CFG->DCR = dcr | DMA_DCR_DSIZE(1) | DMA_DCR_DSIZE(1);
		}
	}

	// Set the number of transfers (number of triggers until complete)
	void transferCount(unsigned int len) {
		uint32_t s, d, n = 0; // 0 = 8 bit, 1 = 16 bit, 2 = 32 bit
		uint32_t dcr = CFG->DCR;
		s = (dcr >> 20) & 3;
		d = (dcr >> 17) & 3;
		if (s == 0 || d == 0) n = 2;
		else if (s == 2 || d == 2) n = 1;
		CFG->DSR_BCR = len << n;
	}

	/*************************************************/
	/**    Special Options / Features               **/
	/*************************************************/

	void interruptAtCompletion(void) {
		CFG->DCR |= DMA_DCR_EINT;
	}

	void disableOnCompletion(void) {
		CFG->DCR |= DMA_DCR_D_REQ;
	}

	// Kinetis-L DMA does not have these features :-(
	//
	// void interruptAtHalf(void) {}
	// void replaceSettingsOnCompletion(const DMABaseClass &settings) {};
	// TODO: can a 2nd linked channel be used to emulate this?

protected:
	// users should not be able to create instances of DMABaseClass, which
	// require the inheriting class to initialize the TCD pointer.
	DMABaseClass() {}

	static inline void copy_cfg(CFG_t *dst, const CFG_t *src) {
		dst->SAR = src->SAR;
		dst->DAR = src->DAR;
		dst->DSR_BCR = src->DSR_BCR;
		dst->DCR = src->DCR;
	}
private:
	static inline uint32_t len2mod(uint32_t len) {
		if (len < 16) return 0;
		if (len < 32) return 1;
		if (len < 64) return 2;
		if (len < 128) return 3;
		if (len < 256) return 4;
		if (len < 512) return 5;
		if (len < 1024) return 6;
		if (len < 2048) return 7;
		if (len < 4096) return 8;
		if (len < 8192) return 9;
		if (len < 16384) return 10;
		if (len < 32768) return 11;
		if (len < 65536) return 12;
		if (len < 131072) return 13;
		if (len < 262144) return 14;
		return 15;
	}
};


// DMASetting represents settings stored only in memory, which can be
// applied to any DMA channel.

class DMASetting : public DMABaseClass {
public:
	DMASetting() {
		cfgdata.SAR = NULL;
		cfgdata.DAR = NULL;
		cfgdata.DSR_BCR = 0;
		cfgdata.DCR = DMA_DCR_CS;
		CFG = &cfgdata;
	}
	DMASetting(const DMASetting &c) {
		CFG = &cfgdata;
		*this = c;
	}
	DMASetting(const DMABaseClass &c) {
		CFG = &cfgdata;
		*this = c;
	}
	DMASetting & operator = (const DMABaseClass &rhs) {
		copy_cfg(CFG, rhs.CFG);
		return *this;
	}
private:
	CFG_t cfgdata __attribute__((aligned(4)));
};


// DMAChannel reprents an actual DMA channel and its current settings

class DMAChannel : public DMABaseClass {
public:
	/*************************************************/
	/**    Channel Allocation                       **/
	/*************************************************/

	DMAChannel() {
		begin();
	}
	DMAChannel(const DMAChannel &c) {
		CFG = c.CFG;
		channel = c.channel;
	}
	DMAChannel(const DMASetting &c) {
		begin();
		copy_cfg(CFG, c.CFG);
	}
	DMAChannel(bool allocate) {
		if (allocate) begin();
	}
	DMAChannel & operator = (const DMAChannel &rhs) {
		if (channel != rhs.channel) {
			release();
			CFG = rhs.CFG;
			channel = rhs.channel;
		}
		return *this;
	}
	DMAChannel & operator = (const DMASetting &rhs) {
		copy_cfg(CFG, rhs.CFG);
		return *this;
	}
	~DMAChannel() {
		release();
	}
	void begin(bool force_initialization = false);
private:
	void release(void);

public:
	/***************************************/
	/**    Triggering                     **/
	/***************************************/

	// Triggers cause the DMA channel to actually move data.  Each
	// trigger moves a single data unit, which is typically 8, 16 or
	// 32 bits.  If a channel is configured for 200 transfers

	// Use a hardware trigger to make the DMA channel run
	void triggerAtHardwareEvent(uint8_t source) {
		volatile uint8_t *mux;
		CFG->DCR |= DMA_DCR_CS;
		mux = (volatile uint8_t *)&(DMAMUX0_CHCFG0) + channel;
		*mux = 0;
		*mux = (source & 63) | DMAMUX_ENABLE;
	}

	// Use another DMA channel as the trigger, causing this
	// channel to trigger after each transfer is makes, including
	// the its last transfer.  This effectively makes the 2
	// channels run in parallel.  Note, on Teensy 3.0 & 3.1,
	// this feature triggers on every transfer except the last.
	// On Teensy-LC, it triggers on every one, including the last.
	void triggerAtTransfersOf(DMABaseClass &ch) {
		uint32_t dcr = ch.CFG->DCR;
		uint32_t linkcc = (dcr >> 4) & 3;
		if (linkcc == 0 || linkcc == 2) {
			ch.CFG->DCR = (dcr & ~DMA_DCR_LCH1(3)) |
				DMA_DCR_LINKCC(2) | DMA_DCR_LCH1(channel);
		} else if (linkcc == 1) {
			ch.CFG->DCR = (dcr & ~DMA_DCR_LCH1(3)) |
				DMA_DCR_LCH1(channel);
		} else {
			uint32_t lch1 = (dcr >> 2) & 3;
			ch.CFG->DCR = (dcr
				& ~(DMA_DCR_LINKCC(3) | DMA_DCR_LCH2(3) | DMA_DCR_LCH1(3)))
				| DMA_DCR_LINKCC(1) | DMA_DCR_LCH2(lch1) | DMA_DCR_LCH1(channel);
		}
	}

	// Use another DMA channel as the trigger, causing this
	// channel to trigger when the other channel completes.
	void triggerAtCompletionOf(DMABaseClass &ch) {
		uint32_t dcr = ch.CFG->DCR;
		uint32_t linkcc = (dcr >> 4) & 3;
		if (linkcc == 0 || linkcc == 3) {
			ch.CFG->DCR = (dcr & ~DMA_DCR_LCH1(3)) |
				DMA_DCR_LINKCC(3) | DMA_DCR_LCH1(channel);
		} else {
			ch.CFG->DCR = (dcr
				& ~(DMA_DCR_LINKCC(3) | DMA_DCR_LCH2(3)))
				| DMA_DCR_LINKCC(1) | DMA_DCR_LCH2(channel);
		}
	}

	// Cause this DMA channel to be continuously triggered, so
	// it will move data as rapidly as possible, without waiting.
	// Normally this would be used with disableOnCompletion().
	void triggerContinuously(void) {
		uint32_t dcr = CFG->DCR;
		dcr &= ~(DMA_DCR_ERQ | DMA_DCR_CS);
		CFG->DCR = dcr;
		CFG->DCR = dcr | DMA_DCR_START;
	}

	// Manually trigger the DMA channel.
	void triggerManual(void) {
		CFG->DCR = (CFG->DCR & ~DMA_DCR_ERQ) | (DMA_DCR_CS | DMA_DCR_START);
	}


	/***************************************/
	/**    Interrupts                     **/
	/***************************************/

	// An interrupt routine can be run when the DMA channel completes
	// the entire transfer, and also optionally when half of the
	// transfer is completed.
	void attachInterrupt(void (*isr)(void)) {
		_VectorsRam[channel + IRQ_DMA_CH0 + 16] = isr;
		NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + channel);
	}

	void detachInterrupt(void) {
		NVIC_DISABLE_IRQ(IRQ_DMA_CH0 + channel);
	}

	void clearInterrupt(void) {
		CFG->DSR_BCR = DMA_DSR_BCR_DONE;
	}


	/***************************************/
	/**    Enable / Disable               **/
	/***************************************/

	void enable(void) {
		CFG->DCR |= DMA_DCR_ERQ;
	}
	void disable(void) {
		CFG->DCR &= ~DMA_DCR_ERQ;
	}

	/***************************************/
	/**    Status                         **/
	/***************************************/

	bool complete(void) {
		if (CFG->DSR_BCR & DMA_DSR_BCR_DONE) return true;
		return false;
	}
	void clearComplete(void) {
		CFG->DSR_BCR |= DMA_DSR_BCR_DONE;
	}
	bool error(void) {
		if (CFG->DSR_BCR &
		  (DMA_DSR_BCR_CE | DMA_DSR_BCR_BES | DMA_DSR_BCR_BED)) return true;
		return false;
	}
	void clearError(void) {
		CFG->DSR_BCR |= DMA_DSR_BCR_DONE;
	}
	void * sourceAddress(void) {
		return (void *)(CFG->SAR);
	}
	void * destinationAddress(void) {
		return (void *)(CFG->DAR);
	}

	/***************************************/
	/**    Direct Hardware Access         **/
	/***************************************/

	uint8_t channel;
	// CFG is accessible due to inheritance from DMABaseClass
};

// arrange the relative priority of 2 or more DMA channels
void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2);
void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2, DMAChannel &ch3);
void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2, DMAChannel &ch3, DMAChannel &ch4);



#endif // KINETISL



#endif // __cplusplus


#endif // DMAChannel_h_
