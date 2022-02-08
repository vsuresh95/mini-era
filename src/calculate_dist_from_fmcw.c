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

	iowrite32(fft_dev, COHERENCE_REG, desc->coherence);
	iowrite32(fft_dev, FFT_DO_PEAK_REG, 0);
	iowrite32(fft_dev, FFT_DO_BITREV_REG, desc->do_bitrev);
	iowrite32(fft_dev, FFT_LOG_LEN_REG, desc->log_len);
	iowrite32(fft_dev, SRC_OFFSET_REG, 0);
	iowrite32(fft_dev, DST_OFFSET_REG, fftHW_in_size);

	// Start accelerators
	iowrite32(fft_dev, CMD_REG, CMD_MASK_START);

  fft_start = get_counter();

	load_aq();

  int count = 0;

	// Wait for completion
	unsigned done = 0;
	while (!done) {
		done = ioread32(fft_dev, STATUS_REG);
		done &= STATUS_MASK_DONE;
    count++;
	}

  fft_stop = get_counter();
  fft_intvl += fft_stop - fft_start;

	iowrite32(fft_dev, CMD_REG, 0x0);

	// printf("fft interval = %lu\n", fft_stop - fft_start);

  MIN_DEBUG(printf("count = %d\n", count));
}
#endif // HW_FFT

float calculate_peak_dist_from_fmcw(float* data)
{
#ifdef DOUBLE_WORD
  
  typedef union float_val {
    unsigned int int_val;
    float flt_val;
  } float_val;

	int value_32_1;
	int value_32_2;
  float value_32_1_f;
  float value_32_2_f;
  int64_t value_64;

  float_val new_value_32_1;
  float_val new_value_32_2;

  // unsigned shift_int = 0x3f800000 + 0x800000 * (32 - FX_IL);
  // float *shift = (float *) &shift_int;
#endif

  calc_start = get_counter();

#ifdef HW_FFT
 #ifndef HW_FFT_BITREV
  // preprocess with bitreverse (fast in software anyway)
  fft_bit_reverse(data, RADAR_N, RADAR_LOGN);
 #endif // HW_FFT

  fft_br_stop = get_counter();
  fft_br_intvl += fft_br_stop - calc_start;

  // printf("  ccccc\n");

  fft_cvtin_start = get_counter();

#ifdef DOUBLE_WORD
  // convert input to fixed point
  for (int j = 0; j < 2 * RADAR_N; j+=2) {
#if 1
#if (FFT_SPANDEX_MODE == 2)
		void* dst = (void*)(input_rad_mem+(j/2));

		asm volatile (
			"mv t0, %1;"
			".word 0x2002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#elif (FFT_SPANDEX_MODE > 2)
		void* dst = (void*)(input_rad_mem+(j/2));

		asm volatile (
			"mv t0, %1;"
			".word 0x4002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#else
		void* dst = (void*)(input_rad_mem+(j/2));

		asm volatile (
			"mv t0, %1;"
			".word 0x0002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#endif

	  new_value_32_1.int_val = (value_64 & 0xFFFFFFFF);
	  new_value_32_2.int_val = ((value_64 >> 32) & 0xFFFFFFFF);
	  // value_32_1_f = (value_64 & 0xFFFFFFFF);
	  // value_32_2_f = ((value_64 >> 32) & 0xFFFFFFFF);

	  value_32_1 = float2fx(new_value_32_1.flt_val, FX_IL);
	  value_32_2 = float2fx(new_value_32_2.flt_val, FX_IL);
#else
	  value_32_1 = float2fx(data[j], FX_IL);
	  value_32_2 = float2fx(data[j+1], FX_IL);
#endif
	  // value_32_1 = (int)(data[j] * (*shift));
	  // value_32_2 = (int)(data[j+1] * (*shift));

    // if(j < 50) printf("dst = %p, value_64 = %lx\n", dst, value_64);
    // if(j < 50) printf("1 half = %x, 2 half = %x\n", new_value_32_1.int_val, new_value_32_2.int_val);
    // if(j < 50) printf("value_32_1 = %x value_32_2 = %x\n", value_32_1, value_32_2);

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
		dst = (void*)((int64_t)(fftHW_li_mem+j));

		asm volatile (
			"mv t0, %0;"
			"mv t1, %1;"
			".word 0x2862B82B"
			: 
			: "r" (dst), "r" (value_64)
			: "t0", "t1", "memory"
		);
#else
 		dst = (void*)((int64_t)(fftHW_li_mem+j));

		asm volatile (
			"mv t0, %0;"
			"mv t1, %1;"
			".word 0x0062B02B"
			: 
			: "r" (dst), "r" (value_64)
			: "t0", "t1", "memory"
		);
#endif
#else
  // convert input to fixed point
  for (int j = 0; j < 2 * RADAR_N; j++) {
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

  // printf("  ddddd\n");

	store_rl();

  fft_in_hw(&fftHW_desc);

  MIN_DEBUG(printf("HW FFT done\n"));

  fft_cvtout_start = get_counter();

#ifdef DOUBLE_WORD
  float max_psd = 0;
  unsigned int max_index = 0;
  unsigned int i;
  float temp;

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
		void* dst = (void*)((uint64_t)(fftHW_lo_mem+j));

		asm volatile (
			"mv t0, %1;"
			".word 0x0002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#endif

	  value_32_1 = value_64 & 0xFFFFFFFF;
    value_32_1_f = (float)fx2float(value_32_1, FX_IL);
    // value_32_1_f = (float)((*shift) * (float) value_32_1);

	  value_32_2 = (value_64 >> 32) & 0xFFFFFFFF;
    value_32_2_f = (float)fx2float(value_32_2, FX_IL);
    // value_32_2_f = (float)((*shift) * (float) value_32_2);

    temp = (pow(value_32_1_f,2) + pow(value_32_2_f,2))/100.0;
    if (temp > max_psd) {
      max_psd = temp;
      max_index = j/2;
    }
  }

  fft_cvtout_stop = get_counter();
  fft_cvtout_intvl += fft_cvtout_stop - fft_cvtout_start;

  calc_stop = get_counter();
  calc_intvl += calc_stop - calc_start;

  // printf("  eeeee\n");
#else
  // convert input to fixed point
  for (int j = 0; j < 2 * RADAR_N; j++) {
    data[j] = (float)fx2float(fftHW_lo_mem[j], FX_IL);
    }

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

  cdfmcw_stop = get_counter();
  cdfmcw_intvl += cdfmcw_stop - cdfmcw_start;
#endif

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

  cdfmcw_stop = get_counter();
  cdfmcw_intvl += cdfmcw_stop - cdfmcw_start;
#endif // if HW_FFT

  float distance = ((float)(max_index*((float)RADAR_fs)/((float)(RADAR_N))))*0.5*RADAR_c/((float)(RADAR_alpha));

  if (max_psd > RADAR_psd_threshold) {
    return distance;
  } else {
    return INFINITY;
  }
}

