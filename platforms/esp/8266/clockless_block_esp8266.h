#pragma once

#define FASTLED_HAS_BLOCKLESS 1
#define ESP 1

FASTLED_NAMESPACE_BEGIN


#define FASTLED_HAS_CLOCKLESS 1

#define PMASK ((1<<(LANES-1)) & 0xFF)


template <uint8_t LANES, int T1, int T2, int T3, EOrder RGB_ORDER = GRB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 50>
class InlineBlockClocklessController : public CPixelLEDController<RGB_ORDER, LANES, PMASK> {
	CMinWait<WAIT_TIME> mWait;
public:
	virtual void init() {
		switch(LANES) {
			case 8:  FastPin<15>::setOutput();
			case 7:  FastPin<13>::setOutput();
			case 6:  FastPin<12>::setOutput();
			case 5:  FastPin<14>::setOutput();
			case 4:  FastPin<2>::setOutput();
			case 3:  FastPin<0>::setOutput();
			case 2:  FastPin<4>::setOutput();
			case 1:  FastPin<5>::setOutput();
		}
	}

	virtual uint16_t getMaxRefreshRate() const { return 400; }

protected:

	virtual void showPixels(PixelController<RGB_ORDER> & pixels) {
		//mWait.wait();
		showRGBInternal(pixels);
		//mWait.mark();
	}

	typedef union {
		uint8_t bytes[12];
		uint16_t shorts[6];
		uint32_t raw[3];
	} Lines;

	template<int BITS,int PX> __attribute__ ((always_inline)) inline static void writeBits(register uint32_t & next_mark, register Lines & b, PixelController<RGB_ORDER, LANES, PMASK> &pixels) { // , register uint32_t & b2)  {
		register Lines b2;
		transpose8x1(b.bytes,b2.bytes);

		register uint8_t d = pixels.template getd<PX>(pixels);
		register uint8_t scale = pixels.template getscale<PX>(pixels);

		for(register uint32_t i = 0; (i < LANES) && (i < 8); i++) {
			while(__clock_cycles() < next_mark);
			next_mark = __clock_cycles() + (T1+T2+T3)-3;
			switch(LANES) {
				case 8:  FastPin<15>::hi();
				case 7:  FastPin<13>::hi();
				case 6:  FastPin<12>::hi();
				case 5:  FastPin<14>::hi();
				case 4:  FastPin<2>::hi();
				case 3:  FastPin<0>::hi();
				case 2:  FastPin<4>::hi();
				case 1:  FastPin<5>::hi();
			}


			while((next_mark - __clock_cycles()) > (T2+T3+6));
			switch(LANES) {
					case 8:  FastPin<15>::set((~b2.bytes[7-i]) & PMASK);
					case 7:  FastPin<13>::set((~b2.bytes[7-i]) & PMASK);
					case 6:  FastPin<12>::set((~b2.bytes[7-i]) & PMASK);
					case 5:  FastPin<14>::set((~b2.bytes[7-i]) & PMASK);
					case 4:  FastPin<2>::set((~b2.bytes[7-i]) & PMASK);
					case 3:  FastPin<0>::set((~b2.bytes[7-i]) & PMASK);
					case 2:  FastPin<4>::set((~b2.bytes[7-i]) & PMASK);
					case 1:  FastPin<5>::set((~b2.bytes[7-i]) & PMASK);
				}


			while((next_mark - __clock_cycles()) > (T3));
			switch(LANES) {
				case 8:  FastPin<15>::lo();
				case 7:  FastPin<13>::lo();
				case 6:  FastPin<12>::lo();
				case 5:  FastPin<14>::lo();
				case 4:  FastPin<2>::lo();
				case 3:  FastPin<0>::lo();
				case 2:  FastPin<4>::lo();
				case 1:  FastPin<5>::lo();
			}

			b.bytes[i] = pixels.template loadAndScale<PX>(pixels,i,d,scale);
		}


	}




	// This method is made static to force making register Y available to use for data on AVR - if the method is non-static, then
	// gcc will use register Y for the this pointer.
	static uint32_t ICACHE_RAM_ATTR showRGBInternal(PixelController<RGB_ORDER, PMASK> &allpixels) {

		// Setup the pixel controller and load/scale the first byte
		allpixels.preStepFirstByteDithering();
		register Lines b0;

		allpixels.preStepFirstByteDithering();
		for(int i = 0; i < LANES; i++) {
			b0.bytes[i] = allpixels.loadAndScale0(i);
		}

		os_intr_lock();
		uint32_t next_mark = __clock_cycles() + (T1+T2+T3);

		while(allpixels.has(1)) {
			allpixels.stepDithering();
			#if 0 && (FASTLED_ALLOW_INTERRUPTS == 1)
			os_intr_lock();
			// if interrupts took longer than 45µs, punt on the current frame
			if(ARM_DWT_CYCCNT > next_mark) {
				if((__clock_cycles()-next_mark) > ((WAIT_TIME-INTERRUPT_THRESHOLD)*CLKS_PER_US)) { sei(); return __clock_cycles(); }
			}
			#endif

			// Write first byte, read next byte
			writeBits<8+XTRA0,1>(next_mark, b0, allpixels);

			// Write second byte, read 3rd byte
			writeBits<8+XTRA0,2>(next_mark, b0, allpixels);
			allpixels.advanceData();

			// Write third byte
			writeBits<8+XTRA0,0>(next_mark, b0, allpixels);

			#if 0 && (FASTLED_ALLOW_INTERRUPTS == 1)
			os_intr_unlock();
			#endif
		};

		os_intr_unlock();
		return __clock_cycles();

	}

};

FASTLED_NAMESPACE_END
