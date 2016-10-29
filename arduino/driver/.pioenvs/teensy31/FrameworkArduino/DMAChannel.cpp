#include "DMAChannel.h"


// The channel allocation bitmask is accessible from "C" namespace,
// so C-only code can reserve DMA channels
uint16_t dma_channel_allocated_mask = 0;



/****************************************************************/
/**                     Teensy 3.0 & 3.1                       **/
/****************************************************************/
#if defined(KINETISK)

void DMAChannel::begin(bool force_initialization)
{
	uint32_t ch = 0;

	__disable_irq();
	if (!force_initialization && TCD && channel < DMA_NUM_CHANNELS
	  && (dma_channel_allocated_mask & (1 << channel))
	  && (uint32_t)TCD == (uint32_t)(0x40009000 + channel * 32)) {
		// DMA channel already allocated
		__enable_irq();
		return;
	}
	while (1) {
		if (!(dma_channel_allocated_mask & (1 << ch))) {
			dma_channel_allocated_mask |= (1 << ch);
			__enable_irq();
			break;
		}
		if (++ch >= DMA_NUM_CHANNELS) {
			__enable_irq();
			TCD = (TCD_t *)0;
			channel = DMA_NUM_CHANNELS;
			return; // no more channels available
			// attempts to use this object will hardfault
		}
	}
	channel = ch;
	SIM_SCGC7 |= SIM_SCGC7_DMA;
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
	DMA_CR = DMA_CR_EMLM | DMA_CR_EDBG ; // minor loop mapping is available
	DMA_CERQ = ch;
	DMA_CERR = ch;
	DMA_CEEI = ch;
	DMA_CINT = ch;
	TCD = (TCD_t *)(0x40009000 + ch * 32);
	uint32_t *p = (uint32_t *)TCD;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
	*p++ = 0;
}

void DMAChannel::release(void)
{
	if (channel >= DMA_NUM_CHANNELS) return;
	DMA_CERQ = channel;
	__disable_irq();
	dma_channel_allocated_mask &= ~(1 << channel);
	__enable_irq();
	channel = 16;
	TCD = (TCD_t *)0;
}

static uint32_t priority(const DMAChannel &c)
{
	uint32_t n;
	n = *(uint32_t *)((uint32_t)&DMA_DCHPRI3 + (c.channel & 0xFC));
	n = __builtin_bswap32(n);
	return (n >> ((c.channel & 0x03) << 3)) & 0x0F;
}

static void swap(DMAChannel &c1, DMAChannel &c2)
{
	uint8_t c;
	DMABaseClass::TCD_t *t;

	c = c1.channel;
	c1.channel = c2.channel;
	c2.channel = c;
	t = c1.TCD;
	c1.TCD = c2.TCD;
	c2.TCD = t;
}

/****************************************************************/
/**                          Teensy-LC                         **/
/****************************************************************/
#elif defined(KINETISL)


void DMAChannel::begin(bool force_initialization)
{
	uint32_t ch = 0;

	__disable_irq();
	if (!force_initialization && CFG && channel < DMA_NUM_CHANNELS
	  && (dma_channel_allocated_mask & (1 << channel))
	  && (uint32_t)CFG == (uint32_t)(0x40008100 + channel * 16)) {
		// DMA channel already allocated
		__enable_irq();
		return;
	}
	while (1) {
		if (!(dma_channel_allocated_mask & (1 << ch))) {
			dma_channel_allocated_mask |= (1 << ch);
			__enable_irq();
			break;
		}
		if (++ch >= DMA_NUM_CHANNELS) {
			__enable_irq();
			CFG = (CFG_t *)0;
			channel = DMA_NUM_CHANNELS;
			return; // no more channels available
			// attempts to use this object will hardfault
		}
	}
	channel = ch;
	SIM_SCGC7 |= SIM_SCGC7_DMA;
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
	CFG = (CFG_t *)(0x40008100 + ch * 16);
	CFG->DSR_BCR = DMA_DSR_BCR_DONE;
	CFG->DCR = DMA_DCR_CS;
	CFG->SAR = NULL;
	CFG->DAR = NULL;
}

void DMAChannel::release(void)
{
	if (channel >= DMA_NUM_CHANNELS) return;
	CFG->DSR_BCR = DMA_DSR_BCR_DONE;
	__disable_irq();
	dma_channel_allocated_mask &= ~(1 << channel);
	__enable_irq();
	channel = 16;
	CFG = (CFG_t *)0;
}

static uint32_t priority(const DMAChannel &c)
{
	return 3 - c.channel;
}

static void swap(DMAChannel &c1, DMAChannel &c2)
{
	uint8_t c;
	DMABaseClass::CFG_t *t;

	c = c1.channel;
	c1.channel = c2.channel;
	c2.channel = c;
	t = c1.CFG;
	c1.CFG = c2.CFG;
	c2.CFG = t;
}




#endif




void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2)
{
	if (priority(ch1) < priority(ch2)) swap(ch1, ch2);
}

void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2, DMAChannel &ch3)
{
	if (priority(ch2) < priority(ch3)) swap(ch2, ch3);
	if (priority(ch1) < priority(ch2)) swap(ch1, ch2);
	if (priority(ch2) < priority(ch3)) swap(ch2, ch3);
}

void DMAPriorityOrder(DMAChannel &ch1, DMAChannel &ch2, DMAChannel &ch3, DMAChannel &ch4)
{
	if (priority(ch3) < priority(ch4)) swap(ch3, ch4);
	if (priority(ch2) < priority(ch3)) swap(ch2, ch3);
	if (priority(ch1) < priority(ch2)) swap(ch1, ch2);
	if (priority(ch3) < priority(ch4)) swap(ch2, ch3);
	if (priority(ch2) < priority(ch3)) swap(ch1, ch2);
	if (priority(ch3) < priority(ch4)) swap(ch2, ch3);
}

