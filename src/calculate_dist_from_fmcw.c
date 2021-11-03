/* -*-Mode: C;-*- */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "verbose.h"
#include "fft-1d.h"

#include "calc_fmcw_dist.h"

#ifdef INT_TIME
struct timeval calc_start, calc_stop;
uint64_t calc_sec  = 0LL;
uint64_t calc_usec = 0LL;

struct timeval fft_stop, fft_start;
uint64_t fft_sec  = 0LL;
uint64_t fft_usec = 0LL;

struct timeval fft_br_stop, fft_br_start;
uint64_t fft_br_sec  = 0LL;
uint64_t fft_br_usec = 0LL;

struct timeval fft_cvtin_stop, fft_cvtin_start;
uint64_t fft_cvtin_sec  = 0LL;
uint64_t fft_cvtin_usec = 0LL;

struct timeval fft_cvtout_stop, fft_cvtout_start;
uint64_t fft_cvtout_sec  = 0LL;
uint64_t fft_cvtout_usec = 0LL;

struct timeval cdfmcw_stop, cdfmcw_start;
uint64_t cdfmcw_sec  = 0LL;
uint64_t cdfmcw_usec = 0LL;
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
  #if 0
	// Configure Spandex request types
#if (SPANDEX_MODE > 1)
	spandex_config_t spandex_config;
	spandex_config.spandex_reg = 0;
#if (SPANDEX_MODE == 2)
	spandex_config.r_en = 1;
	spandex_config.r_type = 1;
#elif (SPANDEX_MODE == 3)
	spandex_config.r_en = 1;
	spandex_config.r_type = 2;
	spandex_config.w_en = 1;
	spandex_config.w_type = 1;
#elif (SPANDEX_MODE == 4)
	spandex_config.r_en = 1;
	spandex_config.r_type = 2;
	spandex_config.w_en = 1;
	spandex_config.w_op = 1;
	spandex_config.w_type = 1;
#endif
	iowrite32(dev, SPANDEX_REG, spandex_config.spandex_reg);
#endif

	iowrite32(dev, COHERENCE_REG, fftHW_desc.esp.coherence);
	iowrite32(dev, FFT_DO_PEAK_REG, 0);
	iowrite32(dev, FFT_DO_BITREV_REG, fftHW_desc.do_bitrev);
	iowrite32(dev, FFT_LOG_LEN_REG, fftHW_desc.log_len);
	iowrite32(dev, SRC_OFFSET_REG, 0);
	iowrite32(dev, DST_OFFSET_REG, in_size);

	// Start accelerators
	iowrite32(dev, CMD_REG, CMD_MASK_START);

	load_aq();

	// Wait for completion
	done = 0;
	while (!done) {
		done = ioread32(dev, STATUS_REG);
		done &= STATUS_MASK_DONE;
	}

	iowrite32(dev, CMD_REG, 0x0);
  #endif
}
#endif // HW_FFT

float calculate_peak_dist_from_fmcw(float* data)
{
  calc_start = get_counter();

#ifdef HW_FFT
 #ifndef HW_FFT_BITREV
  // preprocess with bitreverse (fast in software anyway)
  //fft_bit_reverse(data, fftHW_len, fftHW_log_len);
  fft_bit_reverse(data, RADAR_N, RADAR_LOGN);
 #endif // HW_FFT

  fft_br_stop = get_counter();

  fft_cvtin_start = get_counter();

  // convert input to fixed point
  //for (int j = 0; j < 2 * fftHW_len; j++) {
  for (int j = 0; j < 2 * RADAR_N; j++) {
    //fftHW_lmem[j] = float2fx((fftHW_native_t) data[j], FX_IL);
    fftHW_lmem[j] = float2fx(data[j], FX_IL);
    SDEBUG(if (j < 64) { 
	    printf("FFT_IN_DATA %u : %f\n", j, data[j]);
      });
  }

  fft_cvtin_stop = get_counter();

  fft_start = get_counter();
  fft_in_hw(&fftHW_fd, &fftHW_desc);
  fft_stop = get_counter();

  fft_cvtout_start = get_counter();

  //for (int j = 0; j < 2 * fftHW_len; j++) {
  for (int j = 0; j < 2 * RADAR_N; j++) {
    data[j] = (float)fx2float(fftHW_lmem[j], FX_IL);
    //printf("%u,0x%08x,%f\n", j, fftHW_lmem[j], data[j]);
    SDEBUG(if (j < 64) { 
	    printf("FFT_OUT_DATA %u : %f\n", j, data[j]);
      });
  }

  fft_cvtout_stop = get_counter();

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

#endif // if HW_FFT

  calc_stop = get_counter();

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

  float distance = ((float)(max_index*((float)RADAR_fs)/((float)(RADAR_N))))*0.5*RADAR_c/((float)(RADAR_alpha));
  //printf("Max distance is %.3f\nMax PSD is %4E\nMax index is %d\n", distance, max_psd, max_index);

  cdfmcw_stop = get_counter();
  //printf("max_psd = %f  vs %f\n", max_psd, 1e-10*pow(8192,2));

  if (max_psd > RADAR_psd_threshold) {
    return distance;
  } else {
    return INFINITY;
  }
}

