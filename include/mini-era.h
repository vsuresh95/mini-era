#ifndef _MINI_ERA_H_
#define _MINI_ERA_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#ifndef __user
#define __user
#endif
#endif /* __KERNEL__ */
#define ASI_TIME
// #include <libesp.h>
#include <esp.h>
#include <esp_accelerator.h>

#define ENC_BYTES 24852

#include "coh_func.h"


struct audio_dma_stratus_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned start_offset;
	unsigned src_offset;
	unsigned dst_offset;
	unsigned spandex_conf;
};

#define AUDIO_DMA_STRATUS_IOC_ACCESS	_IOW ('S', 0, struct audio_dma_stratus_access)

struct vitdodec_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned cbps;
	unsigned ntraceback;
	unsigned data_bits;
	// unsigned src_offset;
	// unsigned dst_offset;
	unsigned in_length;
	unsigned out_length;
    unsigned input_start_offset 	;
    unsigned output_start_offset 	;
    unsigned accel_cons_vld_offset ;
    unsigned accel_prod_rdy_offset ;
    unsigned accel_cons_rdy_offset ;
    unsigned accel_prod_vld_offset ;
	unsigned spandex_reg;
};

#define VITDODEC_IOC_ACCESS	_IOW ('S', 0, struct vitdodec_access)

typedef int8_t vitHW_token_t;


// This is for the FFT Accelerator

#if (USE_FFT_FX == 64)
//typedef unsigned long long fftHW_token_t;
typedef int64_t fftHW_token_t;
typedef double fftHW_native_t;
#define fx2float fixed64_to_double
#define float2fx double_to_fixed64
#define FX_IL 42
#elif (USE_FFT_FX == 32)
typedef int fftHW_token_t;
typedef float fftHW_native_t;
#define fx2float fixed32_to_float
#define float2fx float_to_fixed32
#define FX_IL 14
#endif /* FFT_FX_WIDTH */

#if (USE_FFT_ACCEL_TYPE == 1)  // fft_stratus
/* <<--params-def-->> */
/* #define FFTHW_LOG_LEN     14 */
/* #define FFTHW_LEN         (1 << FFTHW_LOG_LEN) */
#define FFTHW_NO_BITREV    0
#define FFTHW_DO_BITREV    1

/* <<--params-->> */
//const int32_t fftHW_do_bitrev = FFTHW_DO_BITREV;
//const int32_t fftHW_len = FFTHW_LEN;
//const int32_t fftHW_log_len = FFTHW_LOG_LEN;

struct fftHW_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned log_len;
	unsigned do_bitrev;
	unsigned src_offset;
	unsigned dst_offset;
};

#elif (USE_FFT_ACCEL_TYPE == 2) // fft2_stratus

// #define LOGN_SAMPLES 6
#define LOGN_SAMPLES 14
#define NUM_FFTS     1
#define DO_INVERSE   0
#define DO_SHIFT     1
#define SCALE_FACTOR 0

#define NACC 1

// struct fftHW_access {
// 	struct esp_access esp;
// 	/* <<--regs-->> */
// 	unsigned scale_factor;
// 	unsigned do_inverse;
// 	unsigned logn_samples;
// 	unsigned do_shift;
// 	unsigned num_ffts;
// 	unsigned src_offset;
// 	unsigned dst_offset;
// 	unsigned spandex_reg;
// };


struct fftHW_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned do_inverse;
	unsigned logn_samples;
	unsigned do_shift;
	// ASI sync flag offsets
    unsigned prod_valid_offset;
    unsigned prod_ready_offset;
    unsigned cons_valid_offset;
    unsigned cons_ready_offset;
    unsigned input_offset;
    unsigned output_offset;
	unsigned src_offset;
	unsigned dst_offset;
    unsigned spandex_reg;
};
#endif

#define FFTHW_IOC_ACCESS	_IOW ('S', 0, struct fftHW_access)


#endif /* _MINI_ERA_H_ */
