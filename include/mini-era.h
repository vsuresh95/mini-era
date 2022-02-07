#ifndef _MINI_ERA_H_
#define _MINI_ERA_H_

#include <esp_accelerator.h>
#include <esp_probe.h>

typedef union
{
  struct
  {
    unsigned char r_en   : 1;
    unsigned char r_op   : 1;
    unsigned char r_type : 2;
    unsigned char r_cid  : 4;
    unsigned char w_en   : 1;
    unsigned char w_op   : 1;
    unsigned char w_type : 2;
    unsigned char w_cid  : 4;
	uint16_t reserved: 16;
  };
  uint32_t spandex_reg;
} spandex_config_t;

struct vitdodec_access {
	uint8_t run;
	uint8_t p2p_store;
	uint8_t p2p_nsrcs;
	enum accelerator_coherence coherence;
	unsigned cbps;
	unsigned ntraceback;
	unsigned data_bits;
	unsigned src_offset;
	unsigned dst_offset;
};

#define VITDODEC_IOC_ACCESS	_IOW ('S', 0, struct vitdodec_access)

#define SLD_FFT 0x059
#define FFT_DEV_NAME "sld,fft_stratus"

#define FFT_DO_PEAK_REG 0x48
#define FFT_DO_BITREV_REG 0x44
#define FFT_LOG_LEN_REG 0x40

#define SLD_VITDODEC 0x030
#define VIT_DEV_NAME "sld,vitdodec_stratus"

#define SLD_SENSOR_DMA 0x050
#define SENSE_DEV_NAME "sld,sensor_dma_stratus"

#define SENSOR_DMA_RD_SP_OFFSET_REG 0x58
#define SENSOR_DMA_RD_WR_ENABLE_REG 0x54
#define SENSOR_DMA_WR_SIZE_REG 0x50
#define SENSOR_DMA_WR_SP_OFFSET_REG 0x4c
#define SENSOR_DMA_RD_SIZE_REG 0x48
#define SENSOR_DMA_DST_OFFSET_REG 0x44
#define SENSOR_DMA_SRC_OFFSET_REG 0x40
#define ENC_BYTES 17408

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
	uint8_t run;
	uint8_t p2p_store;
	uint8_t p2p_nsrcs;
	enum accelerator_coherence coherence;
	unsigned log_len;
	unsigned do_bitrev;
	unsigned src_offset;
	unsigned dst_offset;
};

#elif (USE_FFT_ACCEL_TYPE == 2) // fft2_stratus

#define LOGN_SAMPLES 6
#define NUM_FFTS     1
#define DO_INVERSE   0
#define DO_SHIFT     1
#define SCALE_FACTOR 0

#define NACC 1

struct fftHW_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned scale_factor;
	unsigned do_inverse;
	unsigned logn_samples;
	unsigned do_shift;
	unsigned num_ffts;
	unsigned src_offset;
	unsigned dst_offset;
};
#endif

/*
//BM: Added malloc/free capabilities for baremetal
#define CACHELINE_SIZE 0x10

#ifdef OVERRIDE_DRAM_SIZE
static uintptr_t uncached_area_ptr = DRAM_BASE + (OVERRIDE_DRAM_SIZE >> 1);
#else
static uintptr_t uncached_area_ptr = 0xa0100000;
#endif

void *aligned_malloc(int size) {
#ifndef __riscv
	void *mem = malloc(size + CACHELINE_SIZE + sizeof(void*));
#else
	void *mem = (void *) uncached_area_ptr;
	uncached_area_ptr += size + CACHELINE_SIZE + sizeof(void*);
#endif

	void **ptr = (void**) ((uintptr_t) (mem + CACHELINE_SIZE + sizeof(void*)) & ~(CACHELINE_SIZE-1));
	ptr[-1] = mem;
	return ptr;
}

void aligned_free(void *ptr) {
	// On RISC-V we never free memory
	// This hack is intended for simulation only
#ifndef __riscv
	free(((void**)ptr)[-1]);
#endif
}
//BM: malloc+free changes end
*/

#define FFTHW_IOC_ACCESS	_IOW ('S', 0, struct fftHW_access)

#endif /* _MINI_ERA_H_ */
