/* -*-Mode: C;-*- */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "verbose.h"
#include "fft-1d.h"

#include "calc_fmcw_dist.h"

static inline uint64_t get_counter() {
    uint64_t t_end = 0;
#ifndef __linux__
	__asm__ volatile (
		"li t0, 0;"
		"csrr t0, mcycle;"
		"mv %0, t0"
		: "=r" (t_end)
		:
		: "t0"
	);
#else
	__asm__ volatile (
		"li t0, 0;"
		"csrr t0, cycle;"
		"mv %0, t0"
		: "=r" (t_end)
		:
		: "t0"
	);
#endif
	return t_end;
}

typedef union
{
  struct
  {
    int value_32_1;
    int value_32_2;
  };
  int64_t value_64;
} token_union_t;

typedef union
{
  struct
  {
    float value_32_1;
    float value_32_2;
  };
  int64_t value_64;
} native_union_t;

#include "coh_func.h"

inline void write_mem (void* dst, int64_t value_64)
{
	__asm__ volatile (
		"mv t0, %0;"
		"mv t1, %1;"
		".word " QU(WRITE_CODE)
		:
		: "r" (dst), "r" (value_64)
		: "t0", "t1", "memory"
	);
}

/* Read from the memory*/
inline int64_t read_mem (void* dst)
{
	int64_t value_64;

	__asm__ volatile (
		"mv t0, %1;"
		".word " QU(READ_CODE) ";"
		"mv %0, t1"
		: "=r" (value_64)
		: "r" (dst)
		: "t0", "t1", "memory"
	);

	return value_64;
}

#ifdef INT_TIME
struct timeval calc_start, calc_stop;
uint64_t calc_sec  = 0LL;
uint64_t calc_usec = 0LL;
uint64_t calc_cycles = 0LL;

struct timeval fft_stop, fft_start;
uint64_t fft_sec  = 0LL;
uint64_t fft_usec = 0LL;
uint64_t fft_cycles = 0LL;

struct timeval fft_br_stop, fft_br_start;
uint64_t fft_br_sec  = 0LL;
uint64_t fft_br_usec = 0LL;
uint64_t fft_br_cycles = 0LL;

struct timeval fft_cvtin_stop, fft_cvtin_start;
uint64_t fft_cvtin_sec  = 0LL;
uint64_t fft_cvtin_usec = 0LL;
uint64_t fft_cvtin_cycles = 0LL;

struct timeval fft_cvtout_stop, fft_cvtout_start;
uint64_t fft_cvtout_sec  = 0LL;
uint64_t fft_cvtout_usec = 0LL;
uint64_t fft_cvtout_cycles = 0LL;

struct timeval cdfmcw_stop, cdfmcw_start;
uint64_t cdfmcw_sec  = 0LL;
uint64_t cdfmcw_usec = 0LL;
uint64_t cdfmcw_cycles = 0LL;
#endif

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
#include "contig.h"
#include "fixed_point.h"
#include "mini-era.h"

extern fftHW_token_t* fftHW_li_mem; // Pointer to input memory block
extern fftHW_token_t* fftHW_lo_mem; // Pointer to output memory block

//#define FFT_DEVNAME  "/dev/fft.0"

//extern int32_t fftHW_len;
//extern int32_t fftHW_log_len;

extern int fftHW_fd;
extern contig_handle_t fftHW_mem;
extern fftHW_token_t* fftHW_lmem;

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


static void fft_in_hw(/*unsigned char *inMemory,*/ int *fd, /*contig_handle_t *mem, size_t size, size_t out_size,*/ struct fftHW_access *desc)
{
  //contig_copy_to(*mem, 0, inMemory, size);

  if (ioctl(*fd, FFTHW_IOC_ACCESS, *desc)) {
    perror("IOCTL:");
    exit(EXIT_FAILURE);
  }

  //contig_copy_from(inMemory, *mem, 0, out_size);
}
#endif // HW_FFT

float calculate_peak_dist_from_fmcw(float* data)
{
 #ifdef INT_TIME
  int64_t temp, temp2, fft_calc_start;
  fft_calc_start = get_counter();
 #endif

	unsigned InitLength = 2 * RADAR_N;
  native_union_t SrcData;
	float* src;
	token_union_t DstData;
	int* dst;
  float max_psd = 0;
  unsigned int max_index = 0;
  unsigned int i;
  float __temp;

#ifdef HW_FFT
 #ifndef HW_FFT_BITREV
  // preprocess with bitreverse (fast in software anyway)
  //fft_bit_reverse(data, fftHW_len, fftHW_log_len);
  fft_bit_reverse(data, RADAR_N, RADAR_LOGN);
 #endif // HW_FFT
 #ifdef INT_TIME

  temp2 = get_counter();
  fft_br_cycles += temp2-fft_calc_start;
  temp = get_counter();
 #endif // INT_TIME


	src = data;
	dst = fftHW_li_mem;

  for (unsigned niSample = 0; niSample < InitLength; niSample+=2, src+=2, dst+=2)
	{
		SrcData.value_64 = read_mem((void *) src);

		DstData.value_32_1 = float2fx(SrcData.value_32_1, FX_IL);
		DstData.value_32_2 = float2fx(SrcData.value_32_2, FX_IL);

		write_mem((void *) dst, DstData.value_64);
	}

  // // convert input to fixed point
  // //for (int j = 0; j < 2 * fftHW_len; j++) {
  // for (int j = 0; j < 2 * RADAR_N; j++) {
  //   //fftHW_lmem[j] = float2fx((fftHW_native_t) data[j], FX_IL);
  //   fftHW_lmem[j] = float2fx(data[j], FX_IL);
  //   SDEBUG(if (j < 64) { 
	//     printf("FFT_IN_DATA %u : %f\n", j, data[j]);
  //     });
  // }
 #ifdef INT_TIME

  temp2 = get_counter();
  fft_cvtin_cycles += temp2-temp;
  temp = get_counter();
 #endif // INT_TIME
  fft_in_hw(&fftHW_fd, &fftHW_desc);
 #ifdef INT_TIME
  temp2 = get_counter();
  fft_cycles += temp2-temp;
  temp = get_counter();
 #endif // INT_TIME

	dst = fftHW_lo_mem;

  for (unsigned niSample = 0; niSample < InitLength; niSample+=2, dst+=2)
	{
		DstData.value_64 = read_mem((void *) dst);

		SrcData.value_32_1 = fx2float(DstData.value_32_1, FX_IL);
		SrcData.value_32_2 = fx2float(DstData.value_32_2, FX_IL);

    __temp = (pow(SrcData.value_32_1,2) + pow(SrcData.value_32_2,2))/100.0;
    if (__temp > max_psd) {
      max_psd = __temp;
      max_index = niSample/2;
    }
	}

  // //for (int j = 0; j < 2 * fftHW_len; j++) {
  // for (int j = 0; j < 2 * RADAR_N; j++) {
  //   data[j] = (float)fx2float(fftHW_lmem[j], FX_IL);
  //   //printf("%u,0x%08x,%f\n", j, fftHW_lmem[j], data[j]);
  //   SDEBUG(if (j < 64) { 
	//     printf("FFT_OUT_DATA %u : %f\n", j, data[j]);
  //     });
  // }
 #ifdef INT_TIME
  temp2 = get_counter();
  fft_cvtout_cycles += temp2-temp;
 #endif // INT_TIME
#else // if HW_FFT
 #ifdef INT_TIME
  temp = get_counter();
 #endif // INT_TIME
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
 #ifdef INT_TIME
  int64_t fft_calc_stop = get_counter();
  fft_cycles += fft_calc_stop-temp;
 #endif // INT_TIME
#endif // if HW_FFT

 #ifdef INT_TIME
  temp2 = get_counter();
  calc_cycles += temp2 - fft_calc_start;
  temp = get_counter();
 #endif // INT_TIME

#ifndef HW_FFT
	src = data;

  for (unsigned niSample = 0; niSample < InitLength; niSample+=2, src+=2)
	{
		SrcData.value_64 = read_mem((void *) src);
    __temp = (pow(SrcData.value_32_1,2) + pow(SrcData.value_32_2,2))/100.0;
    if (__temp > max_psd) {
      max_psd = __temp;
      max_index = niSample/2;
    }
	}
#endif

  // for (i=0; i < RADAR_N; i++) {
  //   __temp = (pow(data[2*i],2) + pow(data[2*i+1],2))/100.0;
  //   if (__temp > max_psd) {
  //     max_psd = __temp;
  //     max_index = i;
  //   }
  // }
  float distance = ((float)(max_index*((float)RADAR_fs)/((float)(RADAR_N))))*0.5*RADAR_c/((float)(RADAR_alpha));
  //printf("Max distance is %.3f\nMax PSD is %4E\nMax index is %d\n", distance, max_psd, max_index);
 #ifdef INT_TIME
  temp2 = get_counter();
  cdfmcw_cycles += temp2 - temp;
 #endif // INT_TIME
  //printf("max_psd = %f  vs %f\n", max_psd, 1e-10*pow(8192,2));
  
  #ifdef HW_FFT
  // printf("fft_cvtin_cycles = %llu fft_br_cycles = %llu fft_cvtout_cycles = %llu fft_cycles=%llu calc_cycles=%llu\n",fft_cvtin_cycles,fft_br_cycles,fft_cvtout_cycles, fft_cycles, calc_cycles);
  #endif

  if (max_psd > RADAR_psd_threshold) {
    return distance;
  } else {
    return INFINITY;
  }
}

