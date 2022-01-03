/* -*-Mode: C;-*- */

#include <stdio.h>
#include <stdlib.h>

#include "verbose.h"
#include "fft-1d.h"

#include "calc_fmcw_dist.h"

uint64_t calc_start;
uint64_t calc_stop;
uint64_t calc_intvl;
uint64_t fft_br_stop;
uint64_t fft_br_intvl;
uint64_t fft_cvtin_start;
uint64_t fft_cvtin_stop;
uint64_t fft_cvtin_intvl;
uint64_t fft_start;
uint64_t fft_stop;
uint64_t fft_intvl;
uint64_t fft_cvtout_start;
uint64_t fft_cvtout_stop;
uint64_t fft_cvtout_intvl;
uint64_t cdfmcw_start;
uint64_t cdfmcw_stop;
uint64_t cdfmcw_intvl;

unsigned RADAR_LOGN    = 0;   // Log2 of the number of samples
unsigned RADAR_N       = 0;   // The number of samples (2^LOGN)
float    RADAR_fs      = 0.0; // Sampling Frequency
float    RADAR_alpha   = 0.0; // Chirp rate (saw-tooth)
// CONSTANTS
#define RADAR_c          300000000.0  // Speed of Light in Meters/Sec
#define RADAR_threshold -100;

//float   RADAR_psd_threshold = 1e-10*pow(8192,2);  // ~= 0.006711 and 450 ~= 0.163635 in 16K
float   RADAR_psd_threshold = 0.0067108864;

void init_calculate_peak_dist(unsigned fft_logn_samples)
{
  switch (fft_logn_samples) {
  case 10:
    RADAR_LOGN  = 10;
    RADAR_fs    = 204800.0;
    RADAR_alpha = 30000000000.0;
    RADAR_psd_threshold = 0.000316; // 1e-10*pow(8192,2);  // 450m ~= 0.000638 so psd_thres ~= 0.000316 ?
    break;
  case 14:
    RADAR_LOGN  = 14;
    RADAR_fs    = 32768000.0;
    RADAR_alpha = 4800000000000.0;
    //RADAR_psd_threshold = 1e-10*pow(8192,2);
    RADAR_psd_threshold = 0.0067108864;
    break;
  default:
    printf("ERROR : Unsupported Log-N FFT Samples Value: %u\n", fft_logn_samples);
    exit(-1);
  }
  RADAR_N = (1 << RADAR_LOGN);
}



#ifdef HW_FFT
#include "fixed_point.h"
#include "mini-era.h"

//#define FFT_DEVNAME  "/dev/fft.0"

//extern int32_t fftHW_len;
//extern int32_t fftHW_log_len;

extern fftHW_token_t* fftHW_lmem;
extern fftHW_token_t* fftHW_li_mem;
extern fftHW_token_t* fftHW_lo_mem;

extern struct fftHW_access fftHW_desc;

unsigned int fft_rev(unsigned int v)
{
        unsigned int r = v;
        int s = sizeof(v) * CHAR_BIT - 1;

        for (v >>= 1; v; v >>= 1) {
                r <<= 1;
                r |= v & 1;
                s--;
        }
        r <<= s;
        return r;
}

void fft_bit_reverse(float *w, unsigned int n, unsigned int bits)
{
        unsigned int i, s, shift;

        s = sizeof(i) * CHAR_BIT - 1;
        shift = s - bits + 1;

        for (i = 0; i < n; i++) {
                unsigned int r;
                float t_real, t_imag;

                r = fft_rev(i);
                r >>= shift;

                if (i < r) {
                        t_real = w[2 * i];
                        t_imag = w[2 * i + 1];
                        w[2 * i] = w[2 * r];
                        w[2 * i + 1] = w[2 * r + 1];
                        w[2 * r] = t_real;
                        w[2 * r + 1] = t_imag;
                }
        }
}


static void fft_in_hw(struct fftHW_access *desc)
{
	// Configure Spandex request types
#if (FFT_SPANDEX_MODE > 1)
	spandex_config_t spandex_config;
	spandex_config.spandex_reg = 0;
#if (FFT_SPANDEX_MODE == 2)
	spandex_config.r_en = 1;
	spandex_config.r_type = 1;
#elif (FFT_SPANDEX_MODE == 3)
	spandex_config.r_en = 1;
	spandex_config.r_type = 2;
	spandex_config.w_en = 1;
	spandex_config.w_type = 1;
#elif (FFT_SPANDEX_MODE == 4)
	spandex_config.r_en = 1;
	spandex_config.r_type = 2;
	spandex_config.w_en = 1;
	spandex_config.w_op = 1;
	spandex_config.w_type = 1;
#endif
	iowrite32(fft_dev, SPANDEX_REG, spandex_config.spandex_reg);
#endif

	iowrite32(fft_dev, COHERENCE_REG, fftHW_desc.coherence);
	iowrite32(fft_dev, FFT_DO_PEAK_REG, 0);
	iowrite32(fft_dev, FFT_DO_BITREV_REG, fftHW_desc.do_bitrev);
	iowrite32(fft_dev, FFT_LOG_LEN_REG, fftHW_desc.log_len);
	iowrite32(fft_dev, SRC_OFFSET_REG, 0);
	iowrite32(fft_dev, DST_OFFSET_REG, fftHW_in_size);

	// Start accelerators
	iowrite32(fft_dev, CMD_REG, CMD_MASK_START);

  int count = 0;

	// Wait for completion
	unsigned done = 0;
	while (!done) {
		done = ioread32(fft_dev, STATUS_REG);
		done &= STATUS_MASK_DONE;
    count++;
	}

	iowrite32(fft_dev, CMD_REG, 0x0);

  MIN_DEBUG(printf("count = %d\n", count));
}
#endif // HW_FFT

float calculate_peak_dist_from_fmcw(float* data)
{
#ifdef DOUBLE_WORD
	int value_32_1;
	int value_32_2;
  int64_t value_64;
#endif

  calc_start = get_counter();

#ifdef HW_FFT
 #ifndef HW_FFT_BITREV
  // preprocess with bitreverse (fast in software anyway)
  //fft_bit_reverse(data, fftHW_len, fftHW_log_len);
  fft_bit_reverse(data, RADAR_N, RADAR_LOGN);
 #endif // HW_FFT

  fft_br_stop = get_counter();
  fft_br_intvl += fft_br_stop - calc_start;

  fft_cvtin_start = get_counter();

#ifdef DOUBLE_WORD
  // convert input to fixed point
  for (int j = 0; j < 2 * RADAR_N; j+=2) {
	  value_32_1 = float2fx(data[j], FX_IL);
	  value_32_2 = float2fx(data[j+1], FX_IL);

	  value_64 = ((int64_t) value_32_1) & 0xFFFFFFFF;
	  value_64 |= (((int64_t) value_32_2) << 32) & 0xFFFFFFFF00000000;

#if (FFT_SPANDEX_MODE == 3)
		void* dst = (void*)((int64_t)(fftHW_li_mem+j));

		asm volatile (
			"mv t0, %0;"
			"mv t1, %1;"
			".word 0x2062B02B"
			: 
			: "r" (dst), "r" (value_64)
			: "t0", "t1", "memory"
		);
#elif (FFT_SPANDEX_MODE == 4)
		void* dst = (void*)((int64_t)(fftHW_li_mem+j));

		asm volatile (
			"mv t0, %0;"
			"mv t1, %1;"
			".word 0x2262B82B"
			: 
			: "r" (dst), "r" (value_64)
			: "t0", "t1", "memory"
		);
#else
		((int64_t*) fftHW_li_mem)[j/2] = value_64;
#endif
#else
  // convert input to fixed point
  //for (int j = 0; j < 2 * fftHW_len; j++) {
  for (int j = 0; j < 2 * RADAR_N; j++) {
    //fftHW_lmem[j] = float2fx((fftHW_native_t) data[j], FX_IL);
    fftHW_li_mem[j] = float2fx(data[j], FX_IL);
#endif

    SDEBUG(
      if (j < 64) { 
	    printf("FFT_IN_DATA %u : %lx\n", j, data[j]);
      }
    );
  }

  fft_cvtin_stop = get_counter();
  fft_cvtin_intvl += fft_cvtin_stop - fft_cvtin_start;

  fft_start = get_counter();
  fft_in_hw(&fftHW_desc);
  fft_stop = get_counter();
  fft_intvl += fft_stop - fft_start;

  MIN_DEBUG(printf("HW FFT done\n"));

  fft_cvtout_start = get_counter();

#ifdef DOUBLE_WORD
  // convert fixed point to output
  for (int j = 0; j < 2 * RADAR_N; j+=2) {
#if (FFT_SPANDEX_MODE == 2)
		void* dst = (void*)((uint64_t)(fftHW_lo_mem+j));

		asm volatile (
			"mv t0, %1;"
			".word 0x2002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#elif (FFT_SPANDEX_MODE > 2)
		void* dst = (void*)((uint64_t)(fftHW_lo_mem+j));

		asm volatile (
			"mv t0, %1;"
			".word 0x4002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#else
		value_64 = ((uint64_t*) fftHW_lo_mem)[j/2];
#endif

	  value_32_1 = value_64 & 0xFFFFFFFF;
    data[j] = (float)fx2float(value_32_1, FX_IL);

	  value_32_2 = (value_64 >> 32) & 0xFFFFFFFF;
    data[j+1] = (float)fx2float(value_32_2, FX_IL);
#else
  // convert input to fixed point
  for (int j = 0; j < 2 * RADAR_N; j++) {
    data[j] = (float)fx2float(fftHW_lo_mem[j], FX_IL);
    //printf("%u,0x%08x,%f\n", j, fftHW_lmem[j], data[j]);
#endif

    SDEBUG(
      if (j < 64) { 
	    printf("FFT_OUT_DATA %u : %lx\n", j, data[j]);
      }
    );
  }

  fft_cvtout_stop = get_counter();
  fft_cvtout_intvl += fft_cvtout_stop - fft_cvtout_start;

#else // if HW_FFT

  fft_start = get_counter();

  SDEBUG(for (int tj = 0; tj < 64; tj++) {
	  printf("FFT_IN_DATA %u : %f\n", tj, data[tj]);
    });

  fft(data, RADAR_N, RADAR_LOGN, -1);

  SDEBUG(for (int tj = 0; tj < 64; tj++) {
	  printf("FFT_OUT_DATA %u : %f\n", tj, data[tj]);
    });

  /* for (int j = 0; j < 2 * RADAR_N; j++) { */
  /*   printf("%u,%f\n", j, data[j]); */
  /* } */

  fft_stop = get_counter();
  fft_intvl += fft_stop - fft_start;

#endif // if HW_FFT

  calc_stop = get_counter();
  calc_intvl += calc_stop - calc_start;

  cdfmcw_start = get_counter();

  float max_psd = 0;
  unsigned int max_index = 0;
  unsigned int i;
  float temp;

  for (i=0; i < RADAR_N; i++) {
    temp = (pow(data[2*i],2) + pow(data[2*i+1],2))/100.0;
    if (temp > max_psd) {
      max_psd = temp;
      max_index = i;
    }
  }

  // printf("max_index = %d RADAR_fs = %lx RADAR_N = %d RADAR_c = %lx RADAR_alpha = %lx\n", 
  //   max_index, RADAR_fs, RADAR_N, RADAR_c, RADAR_alpha);

  float distance = ((float)(max_index*((float)RADAR_fs)/((float)(RADAR_N))))*0.5*RADAR_c/((float)(RADAR_alpha));
  //printf("Max distance is %.3f\nMax PSD is %4E\nMax index is %d\n", distance, max_psd, max_index);

  cdfmcw_stop = get_counter();
  cdfmcw_intvl += cdfmcw_stop - cdfmcw_start;
  //printf("max_psd = %f  vs %f\n", max_psd, 1e-10*pow(8192,2));

  if (max_psd > RADAR_psd_threshold) {
    return distance;
  } else {
    return INFINITY;
  }
}

