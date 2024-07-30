/*
 * Copyright 2019 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BYPASS_KERAS_CV_CODE
#include <Python.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#ifdef TIME
#include <sys/time.h>
#endif
#include <math.h>

static unsigned DMA_WORD_PER_BEAT(unsigned _st)
{
        return (sizeof(void *) / _st);
}
#if defined(HW_VIT) || defined(HW_FFT) || defined(HW_CV)
 // These are includes from ESP to support the accelerators
 #include <fcntl.h>
 #include <pthread.h>
 #include <sys/types.h>
 #include <sys/mman.h>
 #include <sys/stat.h>
 #include <string.h>
 #include <unistd.h>


 // These are includes to support ESP FFT Accelerator
 //#include "libesp.h"
 // #include "fft_esp_cfg.h"

 #include "mini-era.h"
#include "fft-1d.h"

#endif

#include "kernels_api.h"

// #include "coh_func.h"

#ifdef USE_SIM_ENVIRON
 #include "sim_environs.h"
#else
 #include "read_trace.h"
#endif
#define SYNC_VAR_SIZE 10
#define UPDATE_VAR_SIZE 2
#define VALID_FLAG_OFFSET 0
#define END_FLAG_OFFSET 2
#define READY_FLAG_OFFSET 4
#define FLT_VALID_FLAG_OFFSET 6
#define FLT_READY_FLAG_OFFSET 8
#define VIT_SYNC_VAR_SIZE 40
#define VIT_UPDATE_VAR_SIZE 8
#define VIT_VALID_FLAG_OFFSET 0
#define VIT_READY_FLAG_OFFSET 16
// #define FLT_VALID_FLAG_OFFSET 6
// #define FLT_READY_FLAG_OFFSET 8

#define ENC_BYTES 24852

#define LOAD_STORE_FLAG_OFFSET 2
#define NUM_CFG_REG 8
#define VIT_LOAD_STORE_FLAG_OFFSET 2 * 4
#define VIT_NUM_CFG_REG 8 * 4

#define RD_SIZE SYNC_VAR_SIZE
#define RD_SP_OFFSET SYNC_VAR_SIZE + 1
#define MEM_SRC_OFFSET SYNC_VAR_SIZE + 2
#define WR_SIZE SYNC_VAR_SIZE + 3
#define WR_SP_OFFSET SYNC_VAR_SIZE + 4
#define MEM_DST_OFFSET SYNC_VAR_SIZE + 5
#define CONS_VALID_OFFSET SYNC_VAR_SIZE + 6
#define CONS_READY_OFFSET SYNC_VAR_SIZE + 7

#define VIT_RD_SIZE (SYNC_VAR_SIZE) * 4
#define VIT_RD_SP_OFFSET (SYNC_VAR_SIZE + 1) * 4
#define VIT_MEM_SRC_OFFSET (SYNC_VAR_SIZE + 2) * 4
#define VIT_WR_SIZE (SYNC_VAR_SIZE + 3) * 4
#define VIT_WR_SP_OFFSET (SYNC_VAR_SIZE + 4) * 4
#define VIT_MEM_DST_OFFSET (SYNC_VAR_SIZE + 5) * 4
#define VIT_CONS_VALID_OFFSET (SYNC_VAR_SIZE + 6) * 4
#define VIT_CONS_READY_OFFSET (SYNC_VAR_SIZE + 7) * 4

extern unsigned time_step;

unsigned use_device_number = 0; // Default to /dev/*_stratus.0

char* lane_names[NUM_LANES] = {"LHazard", "Left", "Center", "Right", "RHazard" };
char* message_names[NUM_MESSAGES] = {"Safe_L_or_R", "Safe_R_only", "Safe_L_only", "Unsafe_L_or_R" };
char* object_names[NUM_OBJECTS] = {"Nothing", "Car", "Truck", "Person", "Bike" };


#ifdef VERBOSE
bool output_viz_trace = true;
#else
bool output_viz_trace = false;
#endif
//unsigned fft_logn_samples = 14; // Defaults to 16k samples

unsigned total_obj; // Total non-'N' obstacle objects across all lanes this time step
unsigned obj_in_lane[NUM_LANES]; // Number of obstacle objects in each lane this time step (at least one, 'n')
unsigned lane_dist[NUM_LANES][MAX_OBJ_IN_LANE]; // The distance to each obstacle object in each lane
char     lane_obj[NUM_LANES][MAX_OBJ_IN_LANE]; // The type of each obstacle object in each lane

char     nearest_obj[NUM_LANES]  = { 'N', 'N', 'N', 'N', 'N' };
float    nearest_dist[NUM_LANES] = { INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE };

unsigned hist_total_objs[NUM_LANES * MAX_OBJ_IN_LANE];

unsigned rand_seed = 0; // Only used if -r <N> option set

float IMPACT_DISTANCE = 50.0; // Minimum distance at which an obstacle "impacts" MyCar (collision case)


/* These are types, functions, etc. required for VITERBI */
#include "viterbi_flat.h"

fftHW_native_t *fftHW_sense_buf;

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


#if defined(HW_FFT) || defined(HW_VIT) 
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

#endif



#ifndef BYPASS_KERAS_CV_CODE
PyObject *pName, *pModule, *pFunc, *pFunc_load;
PyObject *pArgs, *pValue, *pretValue;
#define PY_SSIZE_T_CLEAN

char *python_module = "mio";
char *python_func = "predict";	  
char *python_func_load = "loadmodel";	  
#endif

#ifdef INT_TIME
struct timeval cv_call_stop, cv_call_start;
uint64_t cv_call_sec  = 0LL;
uint64_t cv_call_usec = 0LL;

struct timeval nvdla_stop, nvdla_start;
uint64_t nvdla_sec  = 0LL;
uint64_t nvdla_usec = 0LL;

struct timeval parse_stop, parse_start;
uint64_t parse_sec  = 0LL;
uint64_t parse_usec = 0LL;
#endif

// esp_thread_info_t cfg_000[] = {
// 	{
// 		.run = true,
// 		.devname = "audio_fft_stratus.0",
// 		.ioctl_req = FFTHW_IOC_ACCESS,
// 		// .esp_desc = &(tiled_app_cfg_000[0].esp),
// 	}
// };


//COH

#if (IS_ESP == 1)
// ESP COHERENCE PROTOCOLS
spandex_config_t spandex_config;
#if (COH_MODE == 3)
unsigned coherence = ACC_COH_NONE;
const char print_coh[] = "Non-Coherent DMA";
#elif (COH_MODE == 2)
unsigned coherence = ACC_COH_LLC;
const char print_coh[] = "LLC-Coherent DMA";
#elif (COH_MODE == 1)
unsigned coherence = ACC_COH_RECALL;
const char print_coh[] = "Coherent DMA";
#else
unsigned coherence = ACC_COH_FULL;
const char print_coh[] = "Baseline MESI";
#endif

#else
//SPANDEX COHERENCE PROTOCOLS
unsigned coherence = ACC_COH_FULL;
#if (COH_MODE == 3)
// Owner Prediction
spandex_config_t spandex_config = {.spandex_reg = 0, .r_en = 1, .r_type = 2, .w_en = 1, .w_op = 1, .w_type = 1};
const char print_coh[] = "Owner Prediction";
#elif (COH_MODE == 2)
// Write-through forwarding
spandex_config_t spandex_config = {.spandex_reg = 0, .r_en = 1, .r_type = 2, .w_en = 1, .w_type = 1};
const char print_coh[] = "Write-through forwarding";
#elif (COH_MODE == 1)
// Baseline Spandex
spandex_config_t spandex_config = {.spandex_reg = 0, .r_en = 1, .r_type = 1};
const char print_coh[] = "Baseline Spandex (ReqV)";
#else
// Fully Coherent MESI
spandex_config_t spandex_config= {.spandex_reg = 0};
const char print_coh[] = "Baseline Spandex";
#endif
#endif

typedef union
{
  struct
  {
    fftHW_token_t value_32_1;
    fftHW_token_t value_32_2;
  };
  int64_t value_64;
} token_union_t;

typedef union
{
  struct
  {
    fftHW_native_t value_32_1;
    fftHW_native_t value_32_2;
  };
  int64_t value_64;
} native_union_t;

//END COH

/* These are some top-level defines needed for CV kernel */
unsigned label_match[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0};  // Times CNN matched dictionary
unsigned label_lookup[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0}; // Times we used CNN for object classification
unsigned label_mismatch[NUM_OBJECTS][NUM_OBJECTS] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
  
cv_dictionary_t the_cv_image_dict;

/* These are some top-level defines needed for RADAR */
/* typedef struct { */
/*   unsigned int index; */
/*   unsigned int return_id; */
/*   float distance; */
/*   float return_data[2 * RADAR_N]; */
/* } radar_dict_entry_t; */

#define MAX_RDICT_ENTRIES      12   // This should be updated eventually...
unsigned int         crit_fft_samples_set = 0; // The FFT set used for radara returns.
unsigned int         num_radar_samples_sets = 0;
unsigned int         radar_dict_items_per_set = 0;
radar_dict_entry_t** the_radar_return_dict;
unsigned int         radar_log_nsamples_per_dict_set[MAX_RDICT_SAMPLE_SETS];

unsigned radar_total_calc = 0;
unsigned hist_pct_errs[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES][5];// = {0, 0, 0, 0, 0}; // One per distance, plus global?
unsigned hist_distances[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES];
char*    hist_pct_err_label[5] = {"   0%", "<  1%", "< 10%", "<100%", ">100%"};
unsigned radar_inputs_histogram[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES];

/* These are some top-level defines needed for VITERBI */
/* typedef struct { */
/*   unsigned int msg_num; */
/*   unsigned int msg_id; */
/*   ofdm_param   ofdm_p; */
/*   frame_param  frame_p; */
/*   uint8_t      in_bits[MAX_ENCODED_BITS]; */
/* } vit_dict_entry_t; */

uint8_t descramble[1600]; // I think this covers our max use cases
uint8_t actual_msg[1600];

unsigned int      num_viterbi_dictionary_items = 0;
vit_dict_entry_t* the_viterbi_trace_dict;

unsigned vit_msgs_size = 0;
unsigned vit_msgs_per_step;
const char* vit_msgs_size_str[VITERBI_MSG_LENGTHS] = {"SHORT", "MEDIUM", "LONG", "MAXIMUM"};
const char* vit_msgs_per_step_str[VITERBI_MSGS_PER_STEP] = {"One message per time step",
							    "One message per obstacle per time step",
							    "One msg per obstacle + 1 per time step" };
unsigned viterbi_messages_histogram[VITERBI_MSG_LENGTHS][NUM_MESSAGES];

unsigned total_msgs = 0; // Total messages decoded during the full run
unsigned bad_decode_msgs = 0; // Total messages decoded incorrectly during the full run

#ifdef HW_VIT
// These are Viterbi Harware Accelerator Variales, etc.
char VIT_DEVNAME[128];
char VITDMA_DEVNAME[128];

int vitHW_fd;
int vitDMA_fd;
contig_handle_t vitHW_mem;
vitHW_token_t *vitHW_lmem;   // Pointer to local view of contig memory
vitHW_token_t *vitHW_li_mem; // Pointer to input memory block
vitHW_token_t *vitHW_lo_mem; // Pointer to output memory block
size_t vitHW_in_words_adj;
size_t vitHW_out_words_adj;
size_t vitHW_in_len;
size_t vitHW_out_len;
size_t vitHW_in_size;
size_t vitHW_out_size;
size_t vitHW_out_offset;
size_t vitHW_size;

//BM
// unsigned vit_input_start_offset 	;
// unsigned vit_output_start_offset 	;
// unsigned vit_cons_vld_offset ;
// unsigned vit_prod_rdy_offset ;
// unsigned vit_cons_rdy_offset ;
// unsigned vit_prod_vld_offset ;
unsigned VitProdRdyFlag;
unsigned VitProdVldFlag;
unsigned VitConsRdyFlag;
unsigned VitConsVldFlag;

unsigned  VitEndFlag;
struct vitdodec_access vitHW_desc;

size_t vit_dma_len;
size_t vit_dma_offset;

struct audio_dma_stratus_access fftDMA_desc;
struct audio_dma_stratus_access vitDMA_desc;

fftHW_native_t *input_rad_mem;
vitHW_token_t *input_vit_mem;

#ifdef INT_TIME
uint64_t decode_total_cycles = 0LL;
uint64_t descram_cycles = 0LL;
#endif

void reset_vit_sync(){
	int n;
		write_mem(((void*)(vitHW_lmem + VitConsVldFlag)), 0);
		write_mem(((void*)(vitHW_lmem + VitEndFlag)), 0);
		write_mem(((void*)(vitHW_lmem + VitConsRdyFlag)), 1);
		write_mem(((void*)(vitHW_lmem + VitProdRdyFlag)), 1);
		write_mem(((void*)(vitHW_lmem + VitProdVldFlag)), 0); 
		// vitHW_lmem[VitConsVldFlag] = 0;
		// vitHW_lmem[VitConsRdyFlag] = 1;
		// vitHW_lmem[VitProdRdyFlag] = 1;
		// vitHW_lmem[VitProdVldFlag] = 0; 
    // printf("vitHW_lmem[VitConsVldFlag(%x)] = %d\nvitHW_lmem[VitConsRdyFlag(%x)] = %d\nvitHW_lmem[VitProdRdyFlag(%x)] =%d\nvitHW_lmem[VitProdVldFlag(%x)] = %d\n", VitConsVldFlag,vitHW_lmem[VitConsVldFlag],
    // VitConsRdyFlag, vitHW_lmem[VitConsRdyFlag], VitProdRdyFlag, vitHW_lmem[VitProdRdyFlag], VitProdVldFlag, vitHW_lmem[VitProdVldFlag]);

	__asm__ volatile ("fence w, w");	
}






static void init_vit_parameters()
{
	//printf("Doing init_vit_parameters\n");
	if (DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)) == 0) {
		vitHW_in_words_adj  = 24852; //24852
		vitHW_out_words_adj = 18585;
	} else {
		vitHW_in_words_adj  = round_up(24852, DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)));//24852
		vitHW_out_words_adj = round_up(18585, DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)));
	}
	vitHW_in_len = vitHW_in_words_adj;
	vitHW_out_len =  vitHW_out_words_adj;
	vitHW_in_size = vitHW_in_len * sizeof(vitHW_token_t);
	vitHW_out_size = vitHW_out_len * sizeof(vitHW_token_t);
	vitHW_out_offset = vitHW_in_len + 2*VIT_SYNC_VAR_SIZE ;
	vitHW_size = (vitHW_out_offset * sizeof(vitHW_token_t)) + vitHW_out_size; 
  //  + 2*VIT_SYNC_VAR_SIZE

  vit_dma_len = vitHW_in_len + 2*VIT_SYNC_VAR_SIZE;
  vit_dma_offset = vitHW_size;
#ifdef USE_VIT_SENSOR
	vitHW_size += 2 * vit_dma_len;
#endif

	VitProdRdyFlag = VIT_SYNC_VAR_SIZE + vitHW_in_words_adj + VIT_READY_FLAG_OFFSET;
	VitProdVldFlag = VIT_SYNC_VAR_SIZE + vitHW_in_words_adj + VIT_VALID_FLAG_OFFSET;
  VitEndFlag = VitConsVldFlag + VIT_UPDATE_VAR_SIZE;
	VitConsRdyFlag =  VIT_READY_FLAG_OFFSET;
	VitConsVldFlag =  VIT_VALID_FLAG_OFFSET;
}

#endif


#ifdef HW_FFT

char FFT_DEVNAME[128];
char FFTDMA_DEVNAME[128];



int fftHW_fd;
int fftDMA_fd;
contig_handle_t fftHW_mem;

fftHW_token_t* fftHW_lmem;  // Pointer to local version (mapping) of fftHW_mem
fftHW_token_t* fftHW_li_mem; // Pointer to input memory block
fftHW_token_t* fftHW_lo_mem; // Pointer to output memory block
size_t fftHW_in_words_adj;
size_t fftHW_out_words_adj;
size_t fftHW_in_len;
size_t fftHW_out_len;
size_t fftHW_in_size;
size_t fftHW_out_size;
size_t fftHW_out_offset;
size_t fftHW_size;
size_t  acc_len;
size_t  fft_dma_len;
size_t  ConsRdyFlag;
size_t	ConsVldFlag;
size_t  EndFlag;
size_t	ProdRdyFlag;
size_t	ProdVldFlag;
size_t	DMARdyFlag ;
size_t	DMAVldFlag ;

size_t fft_dma_offset;

struct fftHW_access fftHW_desc;

const float FFT_ERR_TH = 0.05;

/* User-defined code */
static void init_fft_parameters()
{
	// int len = 0x1<<14;
	// int len = 0x1<<LOGN_SAMPLES;
  int len = 0x1<<fft_logn_samples;
	if (DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)) == 0) {
		fftHW_in_words_adj  = 2 * len;
		fftHW_out_words_adj = 2 * len;
	} else {
		fftHW_in_words_adj = round_up(2 * len, DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)));
		fftHW_out_words_adj = round_up(2 * len, DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)));
	}
	fftHW_in_len = fftHW_in_words_adj;
	fftHW_out_len =  fftHW_out_words_adj;
	fftHW_in_size = fftHW_in_len * sizeof(fftHW_token_t);
	fftHW_out_size = fftHW_out_len * sizeof(fftHW_token_t);
	//fftHW_out_offset = 0;
	// fftHW_size = (fftHW_out_offset * sizeof(fftHW_token_t)) + fftHW_out_size;

  // uint64_t num_samples = 1 << LOGN_SAMPLES; //131072 //275
  // acc_len = SYNC_VAR_SIZE + 2*len;
  acc_len = SYNC_VAR_SIZE + fftHW_in_len;
  fft_dma_len = 2 * SYNC_VAR_SIZE + fftHW_in_len;
	// fftHW_size = 2*acc_len;
  fftHW_out_offset = SYNC_VAR_SIZE + acc_len;
	fftHW_size = (fftHW_out_offset * sizeof(fftHW_token_t)) + fftHW_out_size;

  fft_dma_offset = fftHW_size;
#ifdef USE_FFT_SENSOR
	fftHW_size += 2 * fft_dma_len;
#endif

  ConsRdyFlag = 0*acc_len + READY_FLAG_OFFSET;
	ConsVldFlag = 0*acc_len + VALID_FLAG_OFFSET;
  EndFlag     = 0*acc_len + END_FLAG_OFFSET;
	ProdRdyFlag = 1*acc_len + READY_FLAG_OFFSET;
	ProdVldFlag = 1*acc_len + VALID_FLAG_OFFSET;
  #ifdef VERBOSE
  printf("ConsRdyFlag = %d\n",ConsRdyFlag );
  printf("ConsVldFlag = %d\n",ConsVldFlag );
  printf("EndFlag     = %d\n",EndFlag     );
  printf("ProdRdyFlag = %d\n",ProdRdyFlag );
  printf("ProdVldFlag = %d\n",ProdVldFlag );
  #endif
	DMARdyFlag = 8*acc_len + READY_FLAG_OFFSET;
	DMAVldFlag = 8*acc_len + VALID_FLAG_OFFSET;
}
#endif




#ifdef HW_FFT
inline void reset_sync(){
	int n;
		write_mem(((void*)(fftHW_lmem + ConsVldFlag)), 0);
		write_mem(((void*)(fftHW_lmem + EndFlag)), 0);
		write_mem(((void*)(fftHW_lmem + ConsRdyFlag)), 1);
		write_mem(((void*)(fftHW_lmem + ProdRdyFlag)), 1);
		write_mem(((void*)(fftHW_lmem + ProdVldFlag)), 0); 
	__asm__ volatile ("fence w, w");	
}



inline uint32_t poll_fft_cons_rdy(){
	int64_t value_64 = 0;
	void* dst = (void*)(fftHW_lmem + (ConsRdyFlag));
	value_64 = read_mem(dst);
	return (value_64 == 1);
}

inline uint32_t poll_fft_prod_valid(){
	void* dst = (void*)(fftHW_lmem+(ProdVldFlag));
	int64_t value_64 = 0;
	value_64 = read_mem(dst);
	return (value_64 == 1);
}


inline void update_fft_cons_valid(){
	// #if (IS_ESP == 1)
	// __asm__ volatile ("fence w, w");	//release semantics
	// #endif
	// void* dst = (void*)(buf+(*cpu_cons_valid_offset)+1);
	// write_mem(dst, last);

	// #if (IS_ESP == 1)
	__asm__ volatile ("fence w, w");	//release semantics
	// #endif

	void* dst = (void*)(fftHW_lmem+(ConsVldFlag));
	int64_t value_64 = 1;
	write_mem(dst, value_64);


	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline void update_fft_cons_rdy(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(fftHW_lmem+(ConsRdyFlag));
	int64_t value_64 = 0;
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline void update_fft_prod_rdy(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(fftHW_lmem+(ProdRdyFlag));
	int64_t value_64 = 1; 
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline void update_fft_prod_valid(){
	__asm__ volatile ("fence w, w");	//release semantics
	void* dst = (void*)(fftHW_lmem+(ProdVldFlag));
	int64_t value_64 = 0; 
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	//acquire semantics
}

#ifdef USE_FFT_SENSOR
inline void reset_fftdma_sync(){
	// Zero out sync region.
	for (unsigned sync_index = 0; sync_index < 2 * SYNC_VAR_SIZE; sync_index+=2) {
		write_mem(((void*)(fftHW_lmem + fft_dma_offset + sync_index)), 0);
	}

	for (unsigned sync_index = 0; sync_index < 2 * SYNC_VAR_SIZE; sync_index+=2) {
		write_mem(((void*)(fftHW_lmem + fft_dma_offset + fft_dma_len + sync_index)), 0);
	}

	// Reset all sync variables to default values.
	write_mem(((void*)(fftHW_lmem + fft_dma_offset + ConsVldFlag)), 0);
	write_mem(((void*)(fftHW_lmem + fft_dma_offset + ConsRdyFlag)), 1);
	write_mem(((void*)(fftHW_lmem + fft_dma_offset + LOAD_STORE_FLAG_OFFSET)), 0);
	write_mem(((void*)(fftHW_lmem + fft_dma_offset + fft_dma_len + ConsVldFlag)), 0);
	write_mem(((void*)(fftHW_lmem + fft_dma_offset + fft_dma_len + ConsRdyFlag)), 1);

	__asm__ volatile ("fence w, w");	
}

inline uint32_t poll_fftdma_cons_rdy(){
	int64_t value_64 = 0;
	void* dst = (void*)(fftHW_lmem + fft_dma_offset + (ConsRdyFlag));
	value_64 = read_mem(dst);
	return (value_64 == 1);
}

inline void update_fftdma_cons_valid(){
	// #if (IS_ESP == 1)
	__asm__ volatile ("fence w, w");	//release semantics
	// #endif

	void* dst = (void*)(fftHW_lmem + fft_dma_offset + (ConsVldFlag));
	int64_t value_64 = 1;
	write_mem(dst, value_64);


	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}


void update_fftdma_end(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(fftHW_lmem + fft_dma_offset + LOAD_STORE_FLAG_OFFSET);
	int64_t value_64 = 2;
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline void update_fftdma_cons_rdy(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(fftHW_lmem + fft_dma_offset + (ConsRdyFlag));
	int64_t value_64 = 0;
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline uint32_t poll_fftdma_prod_valid(){
	void* dst = (void*)(fftHW_lmem + fft_dma_offset + fft_dma_len + (ConsVldFlag));
	int64_t value_64 = 0;
	value_64 = read_mem(dst);
	return (value_64 == 1);
}

inline void update_fftdma_prod_rdy(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(fftHW_lmem + fft_dma_offset + fft_dma_len + (ConsRdyFlag));
	int64_t value_64 = 1; 
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline void update_fftdma_prod_valid(){
	__asm__ volatile ("fence w, w");	//release semantics
	void* dst = (void*)(fftHW_lmem + fft_dma_offset + fft_dma_len + (ConsVldFlag));
	int64_t value_64 = 0; 
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	//acquire semantics
}
#endif

#endif

#ifdef USE_VIT_SENSOR
inline void reset_vitdma_sync(){
	// Zero out sync region.
	for (unsigned sync_index = 0; sync_index < 2 * VIT_SYNC_VAR_SIZE; sync_index+=8) {
		write_mem(((void*)(vitHW_lmem + vit_dma_offset + sync_index)), 0);
	}

	for (unsigned sync_index = 0; sync_index < 2 * VIT_SYNC_VAR_SIZE; sync_index+=8) {
		write_mem(((void*)(vitHW_lmem + vit_dma_offset + vit_dma_len + sync_index)), 0);
	}

	// Reset all sync variables to default values.
	write_mem(((void*)(vitHW_lmem + vit_dma_offset + VitConsVldFlag)), 0);
	write_mem(((void*)(vitHW_lmem + vit_dma_offset + VitConsRdyFlag)), 1);
	write_mem(((void*)(vitHW_lmem + vit_dma_offset + VIT_LOAD_STORE_FLAG_OFFSET)), 0);
	write_mem(((void*)(vitHW_lmem + vit_dma_offset + vit_dma_len + VitConsVldFlag)), 0);
	write_mem(((void*)(vitHW_lmem + vit_dma_offset + vit_dma_len + VitConsRdyFlag)), 1);

	__asm__ volatile ("fence w, w");	
}

inline uint32_t poll_vitdma_cons_rdy(){
	int64_t value_64 = 0;
	void* dst = (void*)(vitHW_lmem + vit_dma_offset + (VitConsRdyFlag));
	value_64 = read_mem(dst);
	return (value_64 == 1);
}

inline void update_vitdma_cons_valid(){
	// #if (IS_ESP == 1)
	__asm__ volatile ("fence w, w");	//release semantics
	// #endif

	void* dst = (void*)(vitHW_lmem + vit_dma_offset + (VitConsVldFlag));
	int64_t value_64 = 1;
	write_mem(dst, value_64);


	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}


void update_vitdma_end(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(vitHW_lmem + vit_dma_offset + VIT_LOAD_STORE_FLAG_OFFSET);
	int64_t value_64 = 2;
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline void update_vitdma_cons_rdy(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(vitHW_lmem + vit_dma_offset + (VitConsRdyFlag));
	int64_t value_64 = 0;
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline uint32_t poll_vitdma_prod_valid(){
	void* dst = (void*)(vitHW_lmem + vit_dma_offset + vit_dma_len + (VitConsVldFlag));
	int64_t value_64 = 0;
	value_64 = read_mem(dst);
	return (value_64 == 1);
}

inline void update_vitdma_prod_rdy(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(vitHW_lmem + vit_dma_offset + vit_dma_len + (VitConsRdyFlag));
	int64_t value_64 = 1; 
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	
}

inline void update_vitdma_prod_valid(){
	__asm__ volatile ("fence w, w");	//release semantics
	void* dst = (void*)(vitHW_lmem + vit_dma_offset + vit_dma_len + (VitConsVldFlag));
	int64_t value_64 = 0; 
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	//acquire semantics
}
#endif


void update_fft_end(){
	__asm__ volatile ("fence w, w");	//acquire semantics
	void* dst = (void*)(fftHW_lmem+(EndFlag));
	int64_t value_64 = 1;
	write_mem(dst, value_64);
	// int time_var = 0;
	// while(time_var<100) time_var++;
	__asm__ volatile ("fence w, w");	

  while(!poll_fft_cons_rdy());
  update_fft_cons_rdy();

  update_fft_cons_valid();

  while(!poll_fft_prod_valid());
  update_fft_prod_valid();

  update_fft_prod_rdy();
}

void update_fftdma_end_oneiter(){
	__asm__ volatile ("fence w, w");
	void* dst = (void*)(fftHW_lmem + fft_dma_offset + LOAD_STORE_FLAG_OFFSET);
	int64_t value_64 = 2;
	write_mem(dst, value_64);
	__asm__ volatile ("fence w, w");	

  // Wait for DMA (consumer) to be ready.
  while(!poll_fftdma_cons_rdy());
  // Reset flag for next iteration.
  update_fftdma_cons_rdy();
  update_fftdma_prod_rdy();

  // Inform DMA (consumer) to start.
  update_fftdma_cons_valid();
}

void update_vitdma_end_oneiter(){
	__asm__ volatile ("fence w, w");
	void* dst = (void*)(vitHW_lmem+vit_dma_offset + VIT_LOAD_STORE_FLAG_OFFSET);
	int64_t value_64 = 2;
	write_mem(dst, value_64);
	__asm__ volatile ("fence w, w");	

  // Wait for DMA (consumer) to be ready.
  while(!poll_vitdma_cons_rdy());
  // Reset flag for next iteration.
  update_vitdma_cons_rdy();
  update_vitdma_prod_rdy();

  // Inform DMA (consumer) to start.
  update_vitdma_cons_valid();
}


extern void descrambler(uint8_t* in, int psdusize, char* out_msg, uint8_t* ref, uint8_t *msg);




status_t init_rad_kernel(char* dict_fn)
{
  DEBUG(printf("In init_rad_kernel...\n"));

  init_calculate_peak_dist(fft_logn_samples);

  // Read in the radar distances dictionary file
  FILE *dictF = fopen(dict_fn,"r");
  if (!dictF)
  {
    printf("Error: unable to open dictionary file %s\n", dict_fn);
    fclose(dictF);
    return error;
  }
  // Read the number of definitions
  if (fscanf(dictF, "%u %u\n", &num_radar_samples_sets, &radar_dict_items_per_set) != 2) {
    printf("ERROR reading the number of Radar Dictionary sets and items per set\n");
    exit(-2);
  }
  DEBUG(printf("  There are %u dictionary sets of %u entries each\n", num_radar_samples_sets, radar_dict_items_per_set));
  the_radar_return_dict = (radar_dict_entry_t**)calloc(num_radar_samples_sets, sizeof(radar_dict_entry_t*));
  if (the_radar_return_dict == NULL) {
    printf("ERROR : Cannot allocate Radar Trace Dictionary memory space\n");
    fclose(dictF);
    return error;
  }
  for (int si = 0; si < num_radar_samples_sets; si++) {
    the_radar_return_dict[si] = (radar_dict_entry_t*)calloc(radar_dict_items_per_set, sizeof(radar_dict_entry_t));
    if (the_radar_return_dict[si] == NULL) {
      printf("ERROR : Cannot allocate Radar Trace Dictionary memory space for set %u\n", si);
    fclose(dictF);
      return error;
    }
  }
  unsigned tot_dict_values = 0;
  unsigned tot_index = 0;
  for (int si = 0; si < num_radar_samples_sets; si++) {
    if (fscanf(dictF, "%u\n", &(radar_log_nsamples_per_dict_set[si])) != 1) {
      printf("ERROR reading the number of Radar Dictionary samples for set %u\n", si);
      exit(-2);
    }
    DEBUG(printf("  Dictionary set %u entries should all have %u log_nsamples\n", si, radar_log_nsamples_per_dict_set[si]));
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      unsigned entry_id;
      unsigned entry_log_nsamples;
      float entry_dist;
      unsigned entry_dict_values = 0;
      if (fscanf(dictF, "%u %u %f", &entry_id, &entry_log_nsamples, &entry_dist) != 3) {
	printf("ERROR reading Radar Dictionary set %u entry %u header\n", si, di);
	exit(-2);
      }
      if (radar_log_nsamples_per_dict_set[si] != entry_log_nsamples) {
	printf("ERROR reading Radar Dictionary set %u entry %u header : Mismatch in log2 samples : %u vs %u\n", si, di, entry_log_nsamples, radar_log_nsamples_per_dict_set[si]);
	exit(-2);
      }
	
      DEBUG(printf("  Reading rad dictionary set %u entry %u : %u %u %f\n", si, di, entry_id, entry_log_nsamples, entry_dist));
      the_radar_return_dict[si][di].index = tot_index++;  // Set, and increment total index
      the_radar_return_dict[si][di].set = si;
      the_radar_return_dict[si][di].index_in_set = di;
      the_radar_return_dict[si][di].return_id = entry_id;
      the_radar_return_dict[si][di].log_nsamples = entry_log_nsamples;
      the_radar_return_dict[si][di].distance =  entry_dist;
      for (int i = 0; i < 2*(1<<entry_log_nsamples); i++) {
	float fin;
	if (fscanf(dictF, "%f", &fin) != 1) {
	  printf("ERROR reading Radar Dictionary set %u entry %u data entries\n", si, di);
	  exit(-2);
	}
	the_radar_return_dict[si][di].return_data[i] = fin;
	tot_dict_values++;
	entry_dict_values++;
      }
      DEBUG(printf("    Read in dict set %u entry %u with %u total values\n", si, di, entry_dict_values));
    } // for (int di across radar dictionary entries per set
    DEBUG(printf("   Done reading in Radar dictionary set %u\n", si));
  } // for (si across radar dictionary sets)
  DEBUG(printf("  Read %u sets with %u entries totalling %u values across them all\n", num_radar_samples_sets, radar_dict_items_per_set, tot_dict_values));
  if (!feof(dictF)) {
    printf("NOTE: Did not hit eof on the radar dictionary file %s\n", dict_fn);
    while(!feof(dictF)) {
      char c;
      if (fscanf(dictF, "%c", &c) != 1) {
	printf("Couldn't read final character\n");
      } else {
	printf("Next char is %c = %u = 0x%x\n", c, c, c);
      }
    }
    //if (!feof(dictF)) { printf("and still no EOF\n"); } 
  }
  fclose(dictF);

  // Initialize hist_pct_errs values
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      hist_distances[si][di] = 0;
      for (int i = 0; i < 5; i++) {
	hist_pct_errs[si][di][i] = 0;
      }
    }
  }

  //Clear the inputs (injected) histogram
  for (int i = 0; i < MAX_RDICT_SAMPLE_SETS; i++) {
    for (int j = 0; j < MAX_RDICT_ENTRIES; j++) {
      radar_inputs_histogram[i][j] = 0;
    }
  }

 #ifdef HW_FFT
  init_fft_parameters();
 #if (USE_FFT_ACCEL_TYPE == 1)
  snprintf(FFT_DEVNAME, 128, "/dev/fft_stratus.%u", use_device_number);
 #elif (USE_FFT_ACCEL_TYPE == 2)
  snprintf(FFT_DEVNAME, 128, "/dev/audio_fft_stratus.%u", use_device_number);
 #endif /* USE_FFT_ACCEL_TYPE */
  DEBUG(printf("Open device %s\n", FFT_DEVNAME));
  #if (USE_FFT_FX == 64)
   DEBUG(printf(" typedef unsigned long long token_t\n"));
   DEBUG(printf(" typedef double native_t\n"));
   DEBUG(printf(" #define fx2float fixed64_to_double\n"));
   DEBUG(printf(" #define float2fx double_to_fixed64\n"));
  #elif (USE_FFT_FX == 32)
   DEBUG(printf(" typedef int token_t\n"));
   DEBUG(printf(" typedef float native_t\n"));
   DEBUG(printf(" #define fx2float fixed32_to_float\n"));
   DEBUG(printf(" #define float2fx float_to_fixed32\n"));
  #endif /* FFT_FX_WIDTH */
  DEBUG(printf(" #define FX_IL %u\n", FX_IL));

  fftHW_fd = open(FFT_DEVNAME, O_RDWR, 0);
  if (fftHW_fd < 0) {
    fprintf(stderr, "Error: cannot open %s", FFT_DEVNAME);
    exit(EXIT_FAILURE);
  }

  DEBUG(printf("Allocate hardware buffer of size %zu\n", fftHW_size));
  fftHW_lmem = contig_alloc(fftHW_size, &fftHW_mem);
  if (fftHW_lmem == NULL) {
    fprintf(stderr, "Error: cannot allocate %zu contig bytes", fftHW_size);
    exit(EXIT_FAILURE);
  }

  fftHW_li_mem = &(fftHW_lmem[SYNC_VAR_SIZE]);
  fftHW_lo_mem = &(fftHW_lmem[fftHW_out_offset]);
  DEBUG(printf("Set fftHW_li_mem = %p  AND fftHW_lo_mem = %p\n", fftHW_li_mem, fftHW_lo_mem));

  fftHW_desc.esp.run = true;
  //BM
  // fftHW_desc.esp.coherence = ACC_COH_NONE;
  fftHW_desc.esp.coherence = coherence; //coherence;
  fftHW_desc.esp.p2p_store = 0;
  fftHW_desc.esp.p2p_nsrcs = 0;
  //fftHW_desc.esp.p2p_srcs = {"", "", "", ""};
  //BM
  fftHW_desc.spandex_reg = spandex_config.spandex_reg;
  fftHW_desc.esp.start_stop = 1;

  fftHW_desc.esp.contig = contig_to_khandle(fftHW_mem);

 #if (USE_FFT_ACCEL_TYPE == 1) // fft_stratus
  #ifdef HW_FFT_BITREV
  fftHW_desc.do_bitrev  = FFTHW_DO_BITREV;
  #else
  fftHW_desc.do_bitrev  = FFTHW_NO_BITREV;
  #endif /* BITREV */
  //fftHW_desc.len      = fftHW_len;
  fftHW_desc.log_len    = fft_logn_samples; // fftHW_log_len;
 #elif (USE_FFT_ACCEL_TYPE == 2) // fft2_stratus
  // fftHW_desc.scale_factor = 0; //BM
  fftHW_desc.logn_samples = fft_logn_samples;
  // fftHW_desc.num_ffts     = 1; //BM
  fftHW_desc.do_inverse   = 0;
  fftHW_desc.do_shift     = 0;
  // fftHW_desc.do_inverse   = 0;
	fftHW_desc.prod_valid_offset = VALID_FLAG_OFFSET;
	fftHW_desc.prod_ready_offset = READY_FLAG_OFFSET;
	fftHW_desc.cons_valid_offset = acc_len + VALID_FLAG_OFFSET;
	fftHW_desc.cons_ready_offset = acc_len + READY_FLAG_OFFSET;
	fftHW_desc.input_offset = SYNC_VAR_SIZE;
	fftHW_desc.output_offset = acc_len + SYNC_VAR_SIZE;
 #endif /* ACCEL_TYPE*/
  // fftHW_desc.src_offset = 0;
  // fftHW_desc.dst_offset = 0;
// #endif

  //BM
  reset_sync();

// cfg_000.run = true;
// cfg_000.devname = "audio_fft_stratus.0",
// cfg_000.ioctl_req = FFTHW_IOC_ACCESS,
// cfg_000[0].esp_desc = &(fftHW_desc.esp),
// cfg_000[0].hw_buf = fftHW_lmem;

// esp_run(cfg_000+0, NACC);
  //BM : start app
  // #ifdef HW_FFT
  // printf("Using IOCTL and not esp_run\n");
  if (ioctl(fftHW_fd, FFTHW_IOC_ACCESS, fftHW_desc)) {
    perror("IOCTL:");
    exit(EXIT_FAILURE);
  }

#ifdef USE_FFT_SENSOR
  snprintf(FFTDMA_DEVNAME, 128, "/dev/audio_dma_stratus.%u", use_device_number);
  fftDMA_fd = open(FFTDMA_DEVNAME, O_RDWR, 0);
  if (fftDMA_fd < 0) {
    fprintf(stderr, "Error: cannot open %s", FFTDMA_DEVNAME);
    exit(EXIT_FAILURE);
  }

  fftDMA_desc.esp.run = true;
  fftDMA_desc.esp.coherence = coherence; //coherence;
  fftDMA_desc.esp.p2p_store = 0;
  fftDMA_desc.esp.p2p_nsrcs = 0;
  fftDMA_desc.esp.start_stop = 1;
  fftDMA_desc.esp.contig = contig_to_khandle(fftHW_mem);
  fftDMA_desc.spandex_conf = 0;

  fftDMA_desc.start_offset = fft_dma_offset;
  fftDMA_desc.src_offset = 0;
  fftDMA_desc.dst_offset = 0;

  reset_fftdma_sync();

  DEBUG(printf("Starting FFT DMA\n"));

  if (ioctl(fftDMA_fd, AUDIO_DMA_STRATUS_IOC_ACCESS, fftDMA_desc)) {
    perror("IOCTL:");
    exit(EXIT_FAILURE);
  }

  // copy radar data to FFT sensor scratchpad
  for (int i = 0; i < radar_dict_items_per_set; i++)
  {
    input_rad_mem = &(the_radar_return_dict[0][i].return_data[0]);

    // We're writing the input data to the same location.
	  // write_mem(((void*)(fftHW_lmem + fft_dma_offset + MEM_SRC_OFFSET)), fft_dma_offset + (2 * SYNC_VAR_SIZE));
	  fftHW_lmem[fft_dma_offset + MEM_SRC_OFFSET] = fft_dma_offset + (2 * SYNC_VAR_SIZE);
    // The total amount of data for each radar dict
	  // write_mem(((void*)(fftHW_lmem + fft_dma_offset + RD_SIZE)), 2*(1<<fft_logn_samples));
	  fftHW_lmem[fft_dma_offset + RD_SIZE] = 2*(1<<fft_logn_samples);

    // Wait for DMA (consumer) to be ready.
    while(!poll_fftdma_cons_rdy());
    // Reset flag for next iteration.
    update_fftdma_cons_rdy();

    fftHW_native_t *fftHW_lmem_temp = (fftHW_native_t *) &(fftHW_lmem[fft_dma_offset + (2 * SYNC_VAR_SIZE)]);

    for(unsigned niSample = 0; niSample < 2*(1<<fft_logn_samples); niSample++) {
      fftHW_lmem_temp[niSample] = input_rad_mem[niSample];
    }

    // We increment the scratchpad offset every dict.
	  // write_mem(((void*)(fftHW_lmem + fft_dma_offset + RD_SP_OFFSET)), i*2*(1<<fft_logn_samples));
	  fftHW_lmem[fft_dma_offset + RD_SP_OFFSET] = i*2*(1<<fft_logn_samples);

	  // Inform DMA (consumer) to start.
	  update_fftdma_cons_valid();
  }

	// Wait for DMA (consumer) to be ready.
	while(!poll_fftdma_cons_rdy());
	// Reset flag for next iteration.
	update_fftdma_cons_rdy();
	// End the DMA operation.
	update_fftdma_end();
	// Inform DMA (consumer) to start.
	update_fftdma_cons_valid();

  DEBUG(printf("Ending FFT DMA\n"));

  fftDMA_desc.spandex_conf = spandex_config.spandex_reg;

  reset_fftdma_sync();

  if (ioctl(fftDMA_fd, AUDIO_DMA_STRATUS_IOC_ACCESS, fftDMA_desc)) {
    perror("IOCTL:");
    exit(EXIT_FAILURE);
  }

  DEBUG(printf("Starting FFT DMA\n"));

  // address to be used for all FFT input data streaming in from the sensor
  // input_rad_mem = &fftHW_lmem[fft_dma_offset + fft_dma_len + 2 * SYNC_VAR_SIZE];
#endif // if USE_FFT_SENSOR

  return success;
  #endif /* FFT IN HW*/
}


/* This is the initialization of the Viterbi dictionary data, etc.
 * The format is:
 *  <n> = number of dictionary entries (message types)
 * For each dictionary entry:
 *  n1 n2 n3 n4 n5 : OFDM parms: 
 *  m1 m2 m3 m4 m5 : FRAME parms:
 *  x1 x2 x3 ...   : The message bits (input to decode routine)
 */

status_t init_vit_kernel(char* dict_fn)
{
  DEBUG(printf("In init_vit_kernel...\n"));
  if (vit_msgs_size >= VITERBI_MSG_LENGTHS) {
    printf("ERROR: Specified too large a vit_msgs_size (-v option): %u but max is %u\n", vit_msgs_size, VITERBI_MSG_LENGTHS);
    exit(-1);
  }
  // Read in the viterbi messages dictionary file
  FILE *dictF = fopen(dict_fn,"r");
  if (!dictF)
  {
    printf("Error: unable to open viterbi dictionary definition file %s\n", dict_fn);
    return error;
  }

  // Read in the trace message dictionary from the trace file
  // Read the number of messages
  if (fscanf(dictF, "%u\n", &num_viterbi_dictionary_items) != 1) {
    printf("ERROR reading the number of Viterbi Dictionary items\n");
    exit(-2);
  }    
  DEBUG(printf("  There are %u dictionary entries\n", num_viterbi_dictionary_items));
  the_viterbi_trace_dict = (vit_dict_entry_t*)calloc(num_viterbi_dictionary_items, sizeof(vit_dict_entry_t));
  if (the_viterbi_trace_dict == NULL) 
  {
    printf("ERROR : Cannot allocate Viterbi Trace Dictionary memory space\n");
    fclose(dictF);
    return error;
  }

 int in_cbps;
  // Read in each dictionary item
  for (int i = 0; i < num_viterbi_dictionary_items; i++) 
  {
    DEBUG(printf("  Reading vit dictionary entry %u\n", i));

    int mnum, mid;
    if (fscanf(dictF, "%d %d\n", &mnum, &mid) != 2) {
      printf("Error reading viterbi kernel dictionary enry %u header: Message_number and Message_id\n", i);
      fclose(dictF);
      exit(-6);
    }
    DEBUG(printf(" V_MSG: num %d Id %d\n", mnum, mid));
    if (mnum != i) {
      printf("ERROR : Check Viterbi Dictionary : i = %d but Mnum = %d  (Mid = %d)\n", i, mnum, mid);
      fclose(dictF);
      exit(-5);
    }
    the_viterbi_trace_dict[i].msg_num = mnum;
    the_viterbi_trace_dict[i].msg_id = mid;

    int in_bpsc, in_dbps, in_encoding, in_rate; // OFDM PARMS
    if (fscanf(dictF, "%d %d %d %d %d\n", &in_bpsc, &in_cbps, &in_dbps, &in_encoding, &in_rate) != 5) {
      printf("Error reading viterbi kernel dictionary entry %u bpsc, cbps, dbps, encoding and rate info\n", i);
      fclose(dictF);
      exit(-2);
    }

    DEBUG(printf("  OFDM: %d %d %d %d %d\n", in_bpsc, in_cbps, in_dbps, in_encoding, in_rate));
    the_viterbi_trace_dict[i].ofdm_p.encoding   = in_encoding;
    the_viterbi_trace_dict[i].ofdm_p.n_bpsc     = in_bpsc;
    the_viterbi_trace_dict[i].ofdm_p.n_cbps     = in_cbps;
    the_viterbi_trace_dict[i].ofdm_p.n_dbps     = in_dbps;
    the_viterbi_trace_dict[i].ofdm_p.rate_field = in_rate;

    int in_pdsu_size, in_sym, in_pad, in_encoded_bits, in_data_bits;
    if (fscanf(dictF, "%d %d %d %d %d\n", &in_pdsu_size, &in_sym, &in_pad, &in_encoded_bits, &in_data_bits) != 5) {
      printf("Error reading viterbi kernel dictionary entry %u psdu num_sym, pad, n_encoded_bits and n_data_bits\n", i);
      fclose(dictF);
      exit(-2);
    }
    DEBUG(printf("  FRAME: %d %d %d %d %d\n", in_pdsu_size, in_sym, in_pad, in_encoded_bits, in_data_bits));
    the_viterbi_trace_dict[i].frame_p.psdu_size      = in_pdsu_size;
    the_viterbi_trace_dict[i].frame_p.n_sym          = in_sym;
    the_viterbi_trace_dict[i].frame_p.n_pad          = in_pad;
    the_viterbi_trace_dict[i].frame_p.n_encoded_bits = in_encoded_bits;
    the_viterbi_trace_dict[i].frame_p.n_data_bits    = in_data_bits;

    int num_in_bits = in_encoded_bits + 10; // strlen(str3)+10; //additional 10 values
    DEBUG(printf("  Reading %u in_bits\n", num_in_bits));
    for (int ci = 0; ci < num_in_bits; ci++) { 
      unsigned c;
      if (fscanf(dictF, "%u ", &c) != 1) {
        printf("Error reading viterbi kernel dictionary entry %u data\n", i);
        fclose(dictF);
        exit(-6);
      }
      #ifdef SUPER_VERBOSE
      printf("%u ", c);
      #endif
      the_viterbi_trace_dict[i].in_bits[ci] = (uint8_t)c;
    }
    DEBUG(printf("\n"));
  }
  fclose(dictF);

  //Clear the messages (injected) histogram
  for (int i = 0; i < VITERBI_MSG_LENGTHS; i++) {
    for (int j = 0; j < NUM_MESSAGES; j++) {
      viterbi_messages_histogram[i][j] = 0;
    }
  }

  for (int i = 0; i < NUM_LANES * MAX_OBJ_IN_LANE; i++) {
    hist_total_objs[i] = 0;
  }

#ifdef HW_VIT
  init_vit_parameters();
  snprintf(VIT_DEVNAME, 128, "/dev/asi_vitdodec_stratus.%u", use_device_number);
  DEBUG(printf("Open Vit-Do-Decode device %s\n", VIT_DEVNAME));
  vitHW_fd = open(VIT_DEVNAME, O_RDWR, 0);
  if(vitHW_fd < 0) {
	  fprintf(stderr, "Error: cannot open %s", VIT_DEVNAME);
	  exit(EXIT_FAILURE);
  }

  vitHW_lmem = contig_alloc(vitHW_size, &vitHW_mem);
  if (vitHW_lmem == NULL) {
    fprintf(stderr, "Error: cannot allocate %zu contig bytes", vitHW_size);
    exit(EXIT_FAILURE);
  }
  // vitHW_li_mem = &(vitHW_lmem[0]);
  // vitHW_lo_mem = &(vitHW_lmem[vitHW_out_offset]);
  vitHW_li_mem = &(vitHW_lmem[VIT_SYNC_VAR_SIZE]);
  vitHW_lo_mem = &(vitHW_lmem[ vitHW_out_offset]);
  DEBUG(printf("Set vitHW_li_mem = %p  AND vitHW_lo_mem = %p\n", vitHW_li_mem, vitHW_lo_mem));

  // printf("Reset Sync 1\n");
  // reset_vit_sync();
  vitHW_desc.esp.run = true;
  
  //BM
  // vitHW_desc.esp.coherence = ACC_COH_NONE;
  vitHW_desc.cbps = in_cbps;
  vitHW_desc.ntraceback = 5; //d_ntraceback;
  vitHW_desc.data_bits = 288; //24852;
  //BM
  vitHW_desc.in_length = 24852; //ENC_BYTES;
  vitHW_desc.out_length = 18585; //240;

  vitHW_desc.input_start_offset = VIT_SYNC_VAR_SIZE	;
  vitHW_desc.output_start_offset =  vitHW_out_offset	;//2*VIT_SYNC_VAR_SIZE +
  // vitHW_desc.accel_cons_vld_offset = VitConsVldFlag ;
  // vitHW_desc.accel_prod_rdy_offset = VitProdRdyFlag;
  // vitHW_desc.accel_cons_rdy_offset = VitConsRdyFlag;
  // vitHW_desc.accel_prod_vld_offset = VitProdVldFlag;
  vitHW_desc.accel_cons_vld_offset = VitProdVldFlag ;
  vitHW_desc.accel_prod_rdy_offset = VitConsRdyFlag;
  vitHW_desc.accel_cons_rdy_offset = VitProdRdyFlag;
  vitHW_desc.accel_prod_vld_offset = VitConsVldFlag;

  vitHW_desc.esp.coherence = coherence;// coherence;//coherence;
  vitHW_desc.spandex_reg = spandex_config.spandex_reg;
  vitHW_desc.esp.start_stop = 1; //TODO: BM

  vitHW_desc.esp.p2p_store = 0;
  vitHW_desc.esp.p2p_nsrcs = 0;
  vitHW_desc.esp.contig = contig_to_khandle(vitHW_mem);

  DEBUG(printf("Reset Sync\n"));
  reset_vit_sync();

  DEBUG(printf("ioctl call to vitdodec to initialize accel\n"));
  if (ioctl(vitHW_fd, VITDODEC_IOC_ACCESS, vitHW_desc)) {
    perror("IOCTL:");
    exit(EXIT_FAILURE);
  }

#ifdef USE_VIT_SENSOR
  snprintf(VITDMA_DEVNAME, 128, "/dev/audio_dma_stratus.%u", use_device_number+1);
  vitDMA_fd = open(VITDMA_DEVNAME, O_RDWR, 0);
  if (vitDMA_fd < 0) {
    fprintf(stderr, "Error: cannot open %s", VITDMA_DEVNAME);
    exit(EXIT_FAILURE);
  }

  vitDMA_desc.esp.run = true;
  vitDMA_desc.esp.coherence = coherence; //coherence;
  vitDMA_desc.esp.p2p_store = 0;
  vitDMA_desc.esp.p2p_nsrcs = 0;
  vitDMA_desc.esp.start_stop = 1;
  vitDMA_desc.esp.contig = contig_to_khandle(vitHW_mem);
  vitDMA_desc.spandex_conf = 0;

  vitDMA_desc.start_offset = vit_dma_offset/4;
  vitDMA_desc.src_offset = 0;
  vitDMA_desc.dst_offset = 0;
  
  reset_vitdma_sync();

  DEBUG(printf("Starting VIT DMA\n"));

  if (ioctl(vitDMA_fd, AUDIO_DMA_STRATUS_IOC_ACCESS, vitDMA_desc)) {
    perror("IOCTL:");
    exit(EXIT_FAILURE);
  }

  // copy radar data to VIT sensor scratchpad
  for (int i = 0; i < 12; i++)
  {
    input_vit_mem = &(the_viterbi_trace_dict[i].in_bits[0]);

    fftHW_token_t *vitdmaHW_lmem = (fftHW_token_t *) vitHW_lmem;

    // We're writing the input data to the same location.
	  // write_mem(((void*)(vitHW_lmem + vit_dma_offset + VIT_MEM_SRC_OFFSET)), vit_dma_offset + 2 * SYNC_VAR_SIZE);
    vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_MEM_SRC_OFFSET]);
	  *vitdmaHW_lmem = (vit_dma_offset/4) + (2 * SYNC_VAR_SIZE);

    // The total amount of data for each radar dict
	  // write_mem(((void*)(vitHW_lmem + vit_dma_offset + VIT_RD_SIZE)), ENC_BYTES/sizeof(int64_t));
    vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_RD_SIZE]);
	  *vitdmaHW_lmem = round_up(ENC_BYTES/4, 4);

    // Wait for DMA (consumer) to be ready.
    while(!poll_vitdma_cons_rdy());
    // Reset flag for next iteration.
    update_vitdma_cons_rdy();

    for(unsigned niSample = 0; niSample < ENC_BYTES; niSample++) {
      vitHW_lmem[vit_dma_offset + (2 * VIT_SYNC_VAR_SIZE) + niSample] = input_vit_mem[niSample];
    }

    // We increment the scratchpad offset every dict.
	  // write_mem(((void*)(vitHW_lmem + vit_dma_offset + VIT_RD_SP_OFFSET)), i*ENC_BYTES/sizeof(int64_t));
    vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_RD_SP_OFFSET]);
	  *vitdmaHW_lmem = i*round_up(ENC_BYTES/4, 4);

	  // Inform DMA (consumer) to start.
	  update_vitdma_cons_valid();
  }

	// Wait for DMA (consumer) to be ready.
	while(!poll_vitdma_cons_rdy());
	// Reset flag for next iteration.
	update_vitdma_cons_rdy();
	// End the DMA operation.
	update_vitdma_end();
	// Inform DMA (consumer) to start.
	update_vitdma_cons_valid();

  DEBUG(printf("Ending VIT DMA\n"));

  vitDMA_desc.spandex_conf = spandex_config.spandex_reg;

  reset_vitdma_sync();

  if (ioctl(vitDMA_fd, AUDIO_DMA_STRATUS_IOC_ACCESS, vitDMA_desc)) {
    perror("IOCTL:");
    exit(EXIT_FAILURE);
  }

  DEBUG(printf("Starting VIT DMA\n"));
#endif // if USE_VIT_SENSOR

#endif

  DEBUG(printf("DONE with init_vit_kernel -- returning success\n"));
  return success;
}


#ifdef HW_CV
 extern void initNVDLA();
 extern void runImageonNVDLAWrapper(char *Image);
#endif

status_t init_cv_kernel(char* py_file, char* dict_cv)
{
  DEBUG(printf("In the init_cv_kernel routine\n"));
  // Generate the object image paths from the cnn_dict path

  for (unsigned i = 0; i < IMAGES_PER_OBJECT_TYPE; i++) {
    snprintf(the_cv_image_dict[no_label][i], 128, "%s/empty_%02u.jpg", dict_cv, i);
    snprintf(the_cv_image_dict[bicycle][i], 128, "%s/bike_%02u.jpg", dict_cv, i);
    snprintf(the_cv_image_dict[car][i], 128, "%s/car_%02u.jpg", dict_cv, i);
    snprintf(the_cv_image_dict[pedestrian][i], 128, "%s/person_%02u.jpg", dict_cv, i);
    snprintf(the_cv_image_dict[truck][i], 128, "%s/truck_%02u.jpg", dict_cv, i);
  }

  #ifdef SUPER_VERBOSE
  for (int i = 0; i < num_label_t; i++) {
    int j = 0;
    printf("the_cv_image_dict[%2u][%2u] = %s\n", i, j, the_cv_image_dict[i][j]);
    DEBUG(for (j = 1; j < IMAGES_PER_OBJECT_TYPE; j++) {
      printf("the_cv_image_dict[%2u][%2u] = %s\n", i, j, the_cv_image_dict[i][j]);
    });
  }
  #endif

  // Initialization to run Keras CNN code 
#ifndef BYPASS_KERAS_CV_CODE
  Py_Initialize();
  pName = PyUnicode_DecodeFSDefault(python_module);
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);

  if (pModule == NULL) {
     PyErr_Print();
     printf("Failed to load Python program, perhaps pythonpath needs to be set; export PYTHONPATH=your_mini_era_dir/cv/CNN_MIO_KERAS");
     return 1;
  } else {
    pFunc_load = PyObject_GetAttrString(pModule, python_func_load);

    if (pFunc_load && PyCallable_Check(pFunc_load)) {
       PyObject_CallObject(pFunc_load, NULL);
    }
    else {
        if (PyErr_Occurred())
        PyErr_Print();
        printf("Cannot find python function - loadmodel");
    }
    Py_XDECREF(pFunc_load);
  }
  DEBUG(printf("CV Kernel Init done\n"));
#endif
  
#ifdef HW_CV
  // Initialize NVDLA
  printf("  Calling the initNVDLA routine\n");
  initNVDLA();
  printf("  Back from the initNVDLA routine\n");
#endif
  return success;
}




label_t run_object_classification_syscall(unsigned tr_val) 
{
  DEBUG(printf("Entered run_object_classification...\n"));
  label_t object;	
#ifdef BYPASS_KERAS_CV_CODE
  object = (label_t)tr_val;
#else
  char shell_cmd[100];
  snprintf(shell_cmd, sizeof(shell_cmd), "sh utils/cnn_shell.sh %u", tr_val);
  DEBUG(printf("  Invoking CV CNN using `%s`\n", shell_cmd));
  FILE *testing = popen(shell_cmd, "r");
  if (testing == NULL)
  {
    printf("FAIL to open CV kernel !\n");
    return 1;
  }
  char pbuffer[100];
  while (fgets(pbuffer, 100, testing) != NULL)
  {
    //printf(pbuffer);
  }
  DEBUG(printf("Label Prediction done \n"));
  DEBUG(printf("pbuffer : %s\n", pbuffer));
  int val = atoi(pbuffer);   //the last thing printed by the Keras code is the predicted label 
  object = (label_t)val;
  pclose(testing);
  DEBUG(printf("run_object_classification returning %u = %u\n", val, object));
#endif
  return object;  
}

label_t run_object_classification(unsigned tr_val) 
{
  DEBUG(printf("Entered run_object_classification... tr_val = %u\n", tr_val));
  label_t object = (label_t)tr_val;
#ifndef BYPASS_KERAS_CV_CODE
  if (pModule != NULL) {
    DEBUG(printf("  Starting call to pModule...\n"));
    pFunc = PyObject_GetAttrString(pModule, python_func);
  
    if (pFunc && PyCallable_Check(pFunc)) {
      pArgs = PyTuple_New(1);
      pValue = PyLong_FromLong(tr_val);
      if (!pValue) {
	Py_DECREF(pArgs);
	Py_DECREF(pFunc);
	Py_DECREF(pModule);
	fprintf(stderr, "Trying to run CNN kernel: Cannot convert C argument into python\n");
	return 1;
      }
      PyTuple_SetItem(pArgs, 0, pValue);
      pretValue = PyObject_CallObject(pFunc, pArgs);
      Py_DECREF(pArgs);
      if (pretValue != NULL) {
	DEBUG(printf("Predicted label from Python program: %ld\n", PyLong_AsLong(pretValue)));
	int val = PyLong_AsLong(pretValue);    
	object = (label_t)val;
	DEBUG(printf("run_object_classification returning %u = %u\n", val, object));
	Py_DECREF(pretValue);
      }
      else {
	Py_DECREF(pFunc);
	Py_DECREF(pModule);
	PyErr_Print();
	printf("Trying to run CNN kernel : Python function call failed\n");
	return 1;
      }
    }
    else {
      if (PyErr_Occurred())
	PyErr_Print();
      printf("Cannot find python function");
    }
    Py_XDECREF(pFunc);
    //Py_DECREF(pModule);
  }
#endif
  return object;  
}


label_t iterate_cv_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_cv_kernel\n"));

  unsigned tr_val = 0; // Default nothing
  switch(nearest_obj[vs.lane]) {
    case 'N' : tr_val = no_label; break;
    case 'B' : tr_val = bicycle; break;
    case 'C' : tr_val = car; break;
    case 'P' : tr_val = pedestrian; break;
    case 'T' : tr_val = truck; break;
    default: printf("ERROR : Unknown object type in cv trace: '%c'\n", nearest_obj[vs.lane]); exit(-2);
  }
  label_t d_object = (label_t)tr_val;

  return d_object;
}


unsigned image_index = 0;

inline label_t parse_output_dimg() {
  FILE *file_p = fopen("./output.dimg", "r");
  const size_t n_classes = 5;
  float probs[n_classes];
  for (size_t i = 0; i < n_classes; i++) {
    if (fscanf(file_p, "%f", &probs[i]) != 1) {
      printf("Didn't parse the probs[%ld] from output.dimg\n", i);
    }
  }
  float max_val = 0.0f;
  size_t max_idx = -1;
  for (size_t i = 0; i < n_classes; i++) {
    if (probs[i] > max_val) {
      max_val = probs[i], max_idx = i;
    }
  }
  fclose(file_p);
  return (label_t)max_idx;
}


label_t execute_cv_kernel(label_t in_tr_val)
{
  /* 2) Conduct object detection on the image frame */
  
  DEBUG(printf("  Calling run_object_detection with in_tr_val tr_val %u %s\n", in_tr_val, object_names[in_tr_val]));
 #ifdef HW_CV
  // Add the call to the NVDLA stuff here.
 #ifdef INT_TIME
  gettimeofday(&(cv_call_start), NULL);
 #endif
  label_t tr_label = in_tr_val;
  DEBUG(printf("Calling NVDLA for idx %u image %s\n", image_index, the_cv_image_dict[in_tr_val][image_index % IMAGES_PER_OBJECT_TYPE]));
 #ifdef INT_TIME
  gettimeofday(&(nvdla_start), NULL);
 #endif
  runImageonNVDLAWrapper(the_cv_image_dict[in_tr_val][image_index % IMAGES_PER_OBJECT_TYPE]);
  DEBUG(printf("   DONE with NVDLA call...\n"));
  image_index++;
 #ifdef INT_TIME
  gettimeofday(&(parse_start), NULL);
  nvdla_sec  += parse_start.tv_sec  - nvdla_start.tv_sec;
  nvdla_usec += parse_start.tv_usec - nvdla_start.tv_usec;
  DEBUG(printf("REAL_HW_CV: Set Call_Sec[%u] to %llu %llu\n", cv_call_sec, cv_call_usec));
 #endif
  DEBUG(printf("Setting object from parse_output_dimg call...\n"));
  label_t object = parse_output_dimg();
 #ifdef INT_TIME
  gettimeofday(&(parse_stop), NULL);
  parse_sec  += parse_stop.tv_sec  - parse_start.tv_sec;
  parse_usec += parse_stop.tv_usec - parse_start.tv_usec;
  cv_call_sec  += parse_stop.tv_sec  - cv_call_start.tv_sec;
  cv_call_usec += parse_stop.tv_usec - cv_call_start.tv_usec;
 #endif
  DEBUG(printf("---> Predicted label = %d\n", object));
#else
  // Call Keras Code
  label_t object = run_object_classification((unsigned)in_tr_val); 
  //label_t object = the_cv_object_dict[tr_val].object;
#endif
  DEBUG(printf("  Returning object %u %s : tr_val %u %s\n", object, object_names[object], in_tr_val, object_names[in_tr_val]));
  return object;
}

void post_execute_cv_kernel(label_t tr_val, label_t cv_object)
{
  if (cv_object == tr_val) {
    label_match[cv_object]++;
    label_match[NUM_OBJECTS]++;
  } else {
    label_mismatch[tr_val][cv_object]++;
  }
  label_lookup[NUM_OBJECTS]++;
  label_lookup[cv_object]++;
}



radar_dict_entry_t* iterate_rad_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_rad_kernel\n"));
  unsigned tr_val = nearest_dist[vs.lane] / RADAR_BUCKET_DISTANCE;  // The proper message for this time step and car-lane
  radar_inputs_histogram[crit_fft_samples_set][tr_val]++;
  //printf("Incrementing radar_inputs_histogram[%u][%u] to %u\n", crit_fft_samples_set, tr_val, radar_inputs_histogram[crit_fft_samples_set][tr_val]);

#if USE_FFT_SENSOR
  // Wait for DMA (consumer) to be ready.
  while(!poll_fftdma_cons_rdy());
  // Reset flag for next iteration.
  update_fftdma_cons_rdy();
  update_fftdma_prod_rdy();

  // We're now storing data from the DMA's scratchpad.
  fftHW_lmem[fft_dma_offset + LOAD_STORE_FLAG_OFFSET] = 1;

  // DMA will write the input data to the same location for the CPU to read.
  fftHW_lmem[fft_dma_offset + MEM_DST_OFFSET] = fft_dma_offset + fft_dma_len + (2 * SYNC_VAR_SIZE);

	// Size for each transfer is the same.
  fftHW_lmem[fft_dma_offset + WR_SIZE] = 2*(1<<fft_logn_samples);

	// Offsets for sync variables to FFT.
  fftHW_lmem[fft_dma_offset + CONS_VALID_OFFSET] = fft_dma_offset + fft_dma_len + VALID_FLAG_OFFSET;
  fftHW_lmem[fft_dma_offset + CONS_READY_OFFSET] = fft_dma_offset + fft_dma_len + READY_FLAG_OFFSET;

  // We increment the scratchpad offset every iteration.
  fftHW_lmem[fft_dma_offset + WR_SP_OFFSET] = tr_val * 2*(1<<fft_logn_samples);

  // Inform DMA (consumer) to start.
  update_fftdma_cons_valid();

  while(!poll_fftdma_prod_valid());
  // Reset flag for next iteration.
  update_fftdma_prod_valid();

  fftHW_sense_buf = (fftHW_native_t *) &(fftHW_lmem[fft_dma_offset + fft_dma_len + (2 * SYNC_VAR_SIZE)]);

  // for (unsigned i = 0; i < 2*(1<<fft_logn_samples); i++) {
  //   printf("F E = %f A = %f\n",
  //     the_radar_return_dict[crit_fft_samples_set][tr_val].return_data[i],
  //     fftHW_lmem_temp[i]
  //     );
  // }
#endif // USE_FFT_SENSOR

  return &(the_radar_return_dict[crit_fft_samples_set][tr_val]);
}
  


distance_t execute_rad_kernel(float * inputs)
{
  DEBUG(printf("In execute_rad_kernel\n"));

  /* 2) Conduct distance estimation on the waveform */
  DEBUG(printf("  Calling calculate_peak_dist_from_fmcw\n"));
  distance_t dist = calculate_peak_dist_from_fmcw(inputs);
  DEBUG(printf("  Returning distance = %.1f\n", dist));
  return dist;
}


void post_execute_rad_kernel(unsigned set, unsigned index, distance_t tr_dist, distance_t dist)
{
  // Get an error estimate (Root-Squared?)
  float error;
  radar_total_calc++;
  hist_distances[set][index]++;
  //printf("Setting hist_distances[%u][%u] to %u\n", set, index, hist_distances[set][index]);
  if ((tr_dist >= 500.0) && (dist > 10000.0)) {
    error = 0.0;
  } else {
    error = (tr_dist - dist);
  }
  float abs_err = fabs(error);
  float pct_err;
  if (tr_dist != 0.0) {
    pct_err = abs_err/tr_dist;
  } else {
    pct_err = abs_err;
  }
  
  DEBUG(printf("%f vs %f : ERROR : %f   ABS_ERR : %f PCT_ERR : %f\n", tr_dist, dist, error, abs_err, pct_err));
  //printf("IDX: %u :: %f vs %f : ERROR : %f   ABS_ERR : %f PCT_ERR : %f\n", index, tr_dist, dist, error, abs_err, pct_err);
  if (pct_err == 0.0) {
    hist_pct_errs[set][index][0]++;
  } else if (pct_err < 0.01) {
    hist_pct_errs[set][index][1]++;
  } else if (pct_err < 0.1) {
    DEBUG(printf("RADAR_LT010_ERR : %f vs %f : ERROR : %f   PCT_ERR : %f\n", tr_dist, dist, error, pct_err));
    hist_pct_errs[set][index][2]++;
  } else if (pct_err < 1.00) {
    DEBUG(printf("RADAR_LT100_ERR : %f vs %f : ERROR : %f   PCT_ERR : %f\n", tr_dist, dist, error, pct_err));
    hist_pct_errs[set][index][3]++;
  } else {
    DEBUG(printf("RADAR_GT100_ERR : %f vs %f : ERROR : %f   PCT_ERR : %f\n", tr_dist, dist, error, pct_err));
    hist_pct_errs[set][index][4]++;
  }
}


/* Each time-step of the trace, we read in the 
 * trace values for the left, middle and right lanes
 * (i.e. which message if the autonomous car is in the 
 *  left, middle or right lane).
 */
vit_dict_entry_t* iterate_vit_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_vit_kernel in lane %u = %s\n", vs.lane, lane_names[vs.lane]));
  hist_total_objs[total_obj]++;
  unsigned tr_val = 0; // set a default to avoid compiler messages
  switch (vs.lane) {
  case lhazard:
    {
      unsigned nd_1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      DEBUG(printf("  Lane %u : obj in %u is %c at %u\n", vs.lane, vs.lane+1, nearest_obj[vs.lane+1], nd_1));
      if ((nearest_obj[1] != 'N') && (nd_1 < VIT_CLEAR_THRESHOLD)) {  
	// Some object is in the left lane within threshold distance
	tr_val = 3; // Unsafe to move from lhazard lane into the left lane 
      } else {
	tr_val = 1;
      }
    }
    break;
  case left:
  case center:
  case right:
    {
      unsigned ndp1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane+1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      unsigned ndm1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane-1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      tr_val = 0;
      DEBUG(printf("  Lane %u : obj in %u is %c at %.1f : obj in %u is %c at %.1f\n", vs.lane, 
		   vs.lane-1, nearest_obj[vs.lane-1], nearest_dist[vs.lane-1],
		   vs.lane+1, nearest_obj[vs.lane+1], nearest_dist[vs.lane+1]));
      if ((nearest_obj[vs.lane-1] != 'N') && (ndm1 < VIT_CLEAR_THRESHOLD)) {
	// Some object is in the Left lane at distance 0 or 1
	DEBUG(printf("    Marking unsafe to move left\n"));
	tr_val += 1; // Unsafe to move from this lane to the left.
      }
      if ((nearest_obj[vs.lane+1] != 'N') && (ndp1 < VIT_CLEAR_THRESHOLD)) {
	// Some object is in the Right lane at distance 0 or 1
	DEBUG(printf("    Marking unsafe to move right\n"));
	tr_val += 2; // Unsafe to move from this lane to the right.
      }
    }
    break;
  case rhazard:
    {
      unsigned nd_3 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[3] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      DEBUG(printf("  Lane %u : obj in %u is %c at %u\n", vs.lane, vs.lane-1, nearest_obj[vs.lane-1], nd_3));
      if ((nearest_obj[3] != 'N') && (nd_3 < VIT_CLEAR_THRESHOLD)) {
	// Some object is in the right lane within threshold distance
	tr_val = 3; // Unsafe to move from center lane to the right.
      } else {
	tr_val = 2;
      }
    }
    break;
  }

  DEBUG(printf("Viterbi final message for lane %u %s = %u\n", vs.lane, lane_names[vs.lane], tr_val));	

  vit_dict_entry_t* trace_msg; // Will hold msg input data for decode, based on trace input

  // Here we determine short or long messages, based on global vit_msgs_size; offset is into the Dictionary
  int msg_offset = vit_msgs_size * NUM_MESSAGES; // 0 = short messages, 4 = long messages

  viterbi_messages_histogram[vit_msgs_size][tr_val]++; 
  switch(tr_val) {
  case 0: // safe_to_move_right_or_left
    trace_msg = &(the_viterbi_trace_dict[0 + msg_offset]);
    break;
  case 1: // safe_to_move_right
    trace_msg = &(the_viterbi_trace_dict[1 + msg_offset]);
    break;
  case 2: // safe_to_move_left
    trace_msg = &(the_viterbi_trace_dict[2 + msg_offset]);
    break;
  case 3: // unsafe_to_move_left_or_right
    trace_msg = &(the_viterbi_trace_dict[3 + msg_offset]);
    break;
  }
  DEBUG(printf(" VIT: Using msg %u Id %u : %s \n", trace_msg->msg_num, trace_msg->msg_id, message_names[trace_msg->msg_id]));

#if USE_VIT_SENSOR
  // Wait for DMA (consumer) to be ready.
  while(!poll_vitdma_cons_rdy());
  // Reset flag for next iteration.
  update_vitdma_cons_rdy();
  update_vitdma_prod_rdy();

  fftHW_token_t *vitdmaHW_lmem = (fftHW_token_t *) vitHW_lmem;

  // We're now storing data from the DMA's scratchpad.
  // vitHW_lmem[vit_dma_offset + LOAD_STORE_FLAG_OFFSET] = 1;
  vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_LOAD_STORE_FLAG_OFFSET]);
	*vitdmaHW_lmem = 1;

  // DMA will write the input data to the same location for the CPU to read.
  // vitHW_lmem[vit_dma_offset + MEM_DST_OFFSET] = SYNC_VAR_SIZE + 72;
  vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_MEM_DST_OFFSET]);
	*vitdmaHW_lmem = SYNC_VAR_SIZE + (72/4); // (vit_dma_offset + vit_dma_len)/4 + (2 * SYNC_VAR_SIZE);

	// Size for each transfer is the same.
  // vitHW_lmem[vit_dma_offset + WR_SIZE] = ENC_BYTES/sizeof(int64_t);
  vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_WR_SIZE]);
	*vitdmaHW_lmem = round_up(ENC_BYTES/4, 4);

	// Offsets for sync variables to vit.
  // vitHW_lmem[vit_dma_offset + CONS_VALID_OFFSET] = VALID_FLAG_OFFSET;
  vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_CONS_VALID_OFFSET]);
	*vitdmaHW_lmem = (vit_dma_offset + vit_dma_len)/4 + VALID_FLAG_OFFSET;
  // vitHW_lmem[vit_dma_offset + CONS_READY_OFFSET] = READY_FLAG_OFFSET;
  vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_CONS_READY_OFFSET]);
	*vitdmaHW_lmem = (vit_dma_offset + vit_dma_len)/4 + READY_FLAG_OFFSET;

  // We increment the scratchpad offset every iteration.
  // vitHW_lmem[vit_dma_offset + WR_SP_OFFSET] = tr_val * ENC_BYTES/sizeof(int64_t);
  vitdmaHW_lmem = (fftHW_token_t *) &(vitHW_lmem[vit_dma_offset + VIT_WR_SP_OFFSET]);
	*vitdmaHW_lmem = tr_val * round_up(ENC_BYTES/4, 4);

  // Inform DMA (consumer) to start.
  update_vitdma_cons_valid();

  while(!poll_vitdma_prod_valid());
  // Reset flag for next iteration.
  update_vitdma_prod_valid();

  // for (unsigned i = 0; i < ENC_BYTES; i++) {
  //   printf("V E = %0x A = %0x\n",
  //     trace_msg->in_bits[i],
  //     vitHW_lmem[vit_dma_offset + vit_dma_len + (2 * VIT_SYNC_VAR_SIZE) + i]
  //     );
  // }
#endif // USE_FFT_SENSOR

  return trace_msg;
}

message_t execute_vit_kernel(vit_dict_entry_t* trace_msg, int num_msgs)
{
  // Send each message (here they are all the same) through the viterbi decoder
  DEBUG(printf("Begin: execute_vit_kernel\n"));
  message_t msg = num_message_t;
  uint8_t *result;
  char     msg_text[1600]; // Big enough to hold largest message (1500?)
  int64_t temp, temp2;
  for (int mi = 0; mi < num_msgs; mi++) {
    DEBUG(printf("  Calling the viterbi decode routine for message %u iter %u\n", trace_msg->msg_num, mi));
    viterbi_messages_histogram[vit_msgs_size][trace_msg->msg_id]++; 
    int n_res_char;
    temp = get_counter();
    result = decode(&(trace_msg->ofdm_p), &(trace_msg->frame_p), &(trace_msg->in_bits[0]), &n_res_char);
    temp2 = get_counter();
    decode_total_cycles += temp2-temp;
    // descramble the output - put it in result
    int psdusize = trace_msg->frame_p.psdu_size;
    DEBUG(printf("  Calling the viterbi descrambler routine\n"));
    temp = get_counter();
    descrambler(result, psdusize, msg_text, NULL /*descram_ref*/, NULL /*msg*/);
    temp2 = get_counter();
    descram_cycles += temp2-temp;

   #if(0)
    printf(" PSDU %u : Msg : = `", psdusize);
    for (int ci = 0; ci < (psdusize - 26); ci++) {
      printf("%c", msg_text[ci]);
    }
    printf("'\n");
   #endif
    if (mi == 0) { 
      // Here we look at the message string and select proper message_t out (just for the first message)
      switch(msg_text[3]) {
      case '0' : msg = safe_to_move_right_or_left; break;
      case '1' : msg = safe_to_move_right_only; break;
      case '2' : msg = safe_to_move_left_only; break;
      case '3' : msg = unsafe_to_move_left_or_right; break;
      default  : msg = num_message_t; break;
      }
    } // if (mi == 0)
  }
  DEBUG(printf("The execute_vit_kernel is returning msg %u\n", msg));
  return msg;
}

void post_execute_vit_kernel(message_t tr_msg, message_t dec_msg)
{
  total_msgs++;
  if (dec_msg != tr_msg) {
    bad_decode_msgs++;
  }
}


/* #undef DEBUG */
/* #define DEBUG(x) x */

vehicle_state_t plan_and_control(label_t label, distance_t distance, message_t message, vehicle_state_t vehicle_state)
{
  DEBUG(printf("In the plan_and_control routine : label %u %s distance %.1f (T1 %.1f T1 %.1f T3 %.1f) message %u\n", 
	       label, object_names[label], distance, THRESHOLD_1, THRESHOLD_2, THRESHOLD_3, message));
  vehicle_state_t new_vehicle_state = vehicle_state;
  if (!vehicle_state.active) {
    // Our car is broken and burning, no plan-and-control possible.
    return vehicle_state;
  }
  
  if (//(label != no_label) && // For safety, assume every return is from SOMETHING we should not hit!
      ((distance <= THRESHOLD_1)
       #ifdef USE_SIM_ENVIRON
       || ((vehicle_state.speed < car_goal_speed) && (distance <= THRESHOLD_2))
       #endif
       )) {
    if (distance <= IMPACT_DISTANCE) {
      DEBUG(printf("WHOOPS: We've suffered a collision on time_step %u!\n", time_step));
      //fprintf(stderr, "WHOOPS: We've suffered a collision on time_step %u!\n", time_step);
      new_vehicle_state.speed = 0.0;
      new_vehicle_state.active = false; // We should add visualizer stuff for this!
      return new_vehicle_state;
    }
    
    // Some object ahead of us that needs to be avoided.
    DEBUG(printf("  In lane %s with %c (%u) at %.1f (trace: %.1f)\n", lane_names[vehicle_state.lane], nearest_obj[vehicle_state.lane], label, distance, nearest_dist[vehicle_state.lane]));
    switch (message) {
      case safe_to_move_right_or_left   :
	/* Bias is move right, UNLESS we are in the Right lane and would then head into the RHazard Lane */
	if (vehicle_state.lane < right) { 
	  DEBUG(printf("   In %s with Safe_L_or_R : Moving Right\n", lane_names[vehicle_state.lane]));
	  new_vehicle_state.lane += 1;
	} else {
	  DEBUG(printf("   In %s with Safe_L_or_R : Moving Left\n", lane_names[vehicle_state.lane]));
	  new_vehicle_state.lane -= 1;
	}	  
	break; // prefer right lane
      case safe_to_move_right_only      :
	DEBUG(printf("   In %s with Safe_R_only : Moving Right\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.lane += 1;
	break;
      case safe_to_move_left_only       :
	DEBUG(printf("   In %s with Safe_L_Only : Moving Left\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.lane -= 1;
	break;
      case unsafe_to_move_left_or_right :
	#ifdef USE_SIM_ENVIRON
	if (vehicle_state.speed > car_decel_rate) {
	  new_vehicle_state.speed = vehicle_state.speed - car_decel_rate; // was / 2.0;
	  DEBUG(printf("   In %s with No_Safe_Move -- SLOWING DOWN from %.2f to %.2f\n", lane_names[vehicle_state.lane], vehicle_state.speed, new_vehicle_state.speed));
	} else {
	  DEBUG(printf("   In %s with No_Safe_Move -- Going < 15.0 so STOPPING!\n", lane_names[vehicle_state.lane]));
	  new_vehicle_state.speed = 0.0;
	}
	#else
	DEBUG(printf("   In %s with No_Safe_Move : STOPPING\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.speed = 0.0;
	#endif
	break; /* Stop!!! */
    default:
      DEBUG(printf(" ERROR  In %s with UNDEFINED MESSAGE: %u\n", lane_names[vehicle_state.lane], message));
      //exit(-6);
    }
  } else {
    // No obstacle-inspired lane change, so try now to occupy the center lane
    switch (vehicle_state.lane) {
    case lhazard:
    case left:
      if ((message == safe_to_move_right_or_left) ||
	  (message == safe_to_move_right_only)) {
	DEBUG(printf("  In %s with Can_move_Right: Moving Right\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.lane += 1;
      }
      break;
    case center:
      // No need to alter, already in the center
      break;
    case right:
    case rhazard:
      if ((message == safe_to_move_right_or_left) ||
	  (message == safe_to_move_left_only)) {
	DEBUG(printf("  In %s with Can_move_Left : Moving Left\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.lane -= 1;
      }
      break;
    }
    #ifdef USE_SIM_ENVIRON
    if ((vehicle_state.speed < car_goal_speed) &&  // We are going slower than we want to, and
	//((label == no_label) ||      // There is no object ahead of us -- don't need; NOTHING is at INF_DISTANCE
	(distance >= THRESHOLD_2)) { // Any object is far enough away 
      if (vehicle_state.speed <= (car_goal_speed - car_accel_rate)) {
	new_vehicle_state.speed += 15.0;
      } else {
	new_vehicle_state.speed = car_goal_speed;
      }
      DEBUG(printf("  Going %.2f : slower than target speed %.2f : Speeding up to %.2f\n", vehicle_state.speed, 50.0, new_vehicle_state.speed));
    }
    #endif
  } // else clause


  return new_vehicle_state;
}
/* #undef DEBUG */
/* #define DEBUG(x) */


void closeout_cv_kernel()
{
  float label_correct_pctg = (100.0*label_match[NUM_OBJECTS])/(1.0*label_lookup[NUM_OBJECTS]);
  printf("\nFinal CV CNN Accuracy: %u correct of %u classifications = %.2f%%\n", label_match[NUM_OBJECTS], label_lookup[NUM_OBJECTS], label_correct_pctg);
  for (int i = 0; i < NUM_OBJECTS; i++) {
    label_correct_pctg = (100.0*label_match[i])/(1.0*label_lookup[i]);
    printf("  CV CNN Accuracy for %10s : %u correct of %u classifications = %.2f%%\n", object_names[i], label_match[i], label_lookup[i], label_correct_pctg);
  }

  unsigned errs = label_lookup[NUM_OBJECTS] - label_match[NUM_OBJECTS];
  if (errs > 0) {
    printf("\nAnalysis of the %u mis-identifications:\n", errs);
    for (int i = 0; i < NUM_OBJECTS; i++) {
      for (int j = 0; j < NUM_OBJECTS; j++) {
	if (label_mismatch[i][j] != 0) {
	  printf("  Mislabeled %10s as %10s on %u occasions\n", object_names[i], object_names[j], label_mismatch[i][j]);
	}
      }
    }
  }

#ifndef BYPASS_KERAS_CV_CODE
    Py_DECREF(pModule);
    Py_Finalize();
#endif   
}

void closeout_rad_kernel()
{
  printf("\nHistogram of Radar Distances:\n");
  printf("    %3s | %3s | %8s | %9s \n", "Set", "Idx", "Distance", "Occurs");
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      printf("    %3u | %3u | %8.3f | %9u \n", si, di, the_radar_return_dict[si][di].distance, hist_distances[si][di]);
    }
  }

  printf("\nHistogram of Radar Distance ABS-PCT-ERROR:\n");
  unsigned totals[] = {0, 0, 0, 0, 0};
  
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      printf("    Set %u Entry %u Id %u Distance %f Occurs %u Histogram:\n", si, di, the_radar_return_dict[si][di].index, the_radar_return_dict[si][di].distance, hist_distances[si][di]);
      for (int i = 0; i < 5; i++) {
	printf("    %7s | %9u \n", hist_pct_err_label[i], hist_pct_errs[si][di][i]);
	totals[i] += hist_pct_errs[si][di][i];
      }
    }
  }

  printf("\n  TOTALS Histogram of Radar Distance ABS-PCT-ERROR:\n");
  for (int i = 0; i < 5; i++) {
    printf("  %7s | %9u \n", hist_pct_err_label[i], totals[i]);
  }


  printf("\nHistogram of Radar Task Inputs Used:\n");
  printf("    %3s | %5s | %9s \n", "Set", "Entry", "NumOccurs");
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      printf("    %3u | %3u | %9u \n", si, di, radar_inputs_histogram[si][di]);
    }
  }
  printf("\n");
}

void closeout_vit_kernel()
{
  // Nothing to do?

  printf("\nHistogram of Total Objects:\n");
  unsigned sum = 0;
  for (int i = 0; i < NUM_LANES * MAX_OBJ_IN_LANE; i++) {
    if (hist_total_objs[i] != 0) {
      printf("%3u | %9u \n", i, hist_total_objs[i]);
      sum += i*hist_total_objs[i];
    }
  }
  double avg_objs = (1.0 * sum)/(1.0 * radar_total_calc); // radar_total_calc == total time steps
  printf("There were %.3lf obstacles per time step (average)\n", avg_objs);
  double avg_msgs = (1.0 * total_msgs)/(1.0 * radar_total_calc); // radar_total_calc == total time steps
  printf("There were %.3lf messages per time step (average)\n", avg_msgs);
  printf("There were %u bad decodes of the %u messages\n", bad_decode_msgs, total_msgs);

  printf("\nHistogram of Viterbi Messages:\n");
  fflush(stdout);
  printf("    %3s | %3s | %9s \n", "Len", "Msg", "NumOccurs");
  fflush(stdout);
  for (int li = 0; li < VITERBI_MSG_LENGTHS; li++) {
    for (int mi = 0; mi < NUM_MESSAGES; mi++) {
      printf("    %3u | %3u | %9u \n", li, mi, viterbi_messages_histogram[li][mi]);
      fflush(stdout);
    }
  }
  printf("\n");

#ifdef HW_VIT
  contig_free(vitHW_mem);
  close(vitHW_fd);
#endif

#ifdef HW_FFT
  contig_free(fftHW_mem);
  close(fftHW_fd);
#endif

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

//#define FFT_DEVNAME  "/dev/fft.0"

//extern int32_t fftHW_len;
//extern int32_t fftHW_log_len;

// extern int fftHW_fd;
// extern contig_handle_t fftHW_mem;
// extern fftHW_token_t* fftHW_lmem;

// extern struct fftHW_access fftHW_desc;

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
  DEBUG(printf("fft_in_hw before prod rdy\n"));
  update_fft_cons_valid();

  DEBUG(printf("fft_in_hw after cons valid, before poll prod valid\n"));
  // if (ioctl(*fd, FFTHW_IOC_ACCESS, *desc)) {
  //   perror("IOCTL:");
  //   exit(EXIT_FAILURE);
  // }
  while(!poll_fft_prod_valid());
  update_fft_prod_valid();
  DEBUG(printf("fft_in_hw done\n"));
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
	fftHW_native_t* src;
	token_union_t DstData;
	fftHW_token_t* dst;
  float max_psd = 0;
  unsigned int max_index = 0;
  unsigned int i;
  float __temp;

#ifdef HW_FFT
 #ifndef HW_FFT_BITREV
  // preprocess with bitreverse (fast in software anyway)
  //fft_bit_reverse(data, fftHW_len, fftHW_log_len);
 DEBUG(printf("calling bitreverse\n"));
  fft_bit_reverse(data, RADAR_N, RADAR_LOGN);
 #endif // HW_FFT
 #ifdef INT_TIME

  temp2 = get_counter();
  fft_br_cycles += temp2-fft_calc_start;
  temp = get_counter();
 #endif // INT_TIME


//BM
DEBUG(printf("Before fft_in_hw: wait for cons rdy\n"));
  while(!poll_fft_cons_rdy());
  update_fft_cons_rdy();

	src = data;
	dst = fftHW_li_mem;

  for (unsigned niSample = 0; niSample < InitLength; niSample+=2, src+=2, dst+=2)
	{
		SrcData.value_64 = read_mem((void *) src);

		DstData.value_32_1 = float2fx(SrcData.value_32_1, FX_IL);
		DstData.value_32_2 = float2fx(SrcData.value_32_2, FX_IL);

		write_mem((void *) dst, DstData.value_64);
	}

  // convert input to fixed point
  //for (int j = 0; j < 2 * fftHW_len; j++) {
  // for (int j = 0; j < 2 * RADAR_N; j++) {
  // for (int j = 0; j < 2 * RADAR_N; j+=2) {
  //   //fftHW_lmem[j] = float2fx((fftHW_native_t) data[j], FX_IL);
  //   //BM
  //   // [j] [j+1]
  //   // [j+1, j]
  //   // fftHW_lmem[j] = float2fx(data[j], FX_IL);
  //   uint64_t val_64 = float2fx(data[j+1], FX_IL);
  //   val_64 = val_64 << 32;
  //   val_64 |= (((uint64_t)(float2fx(data[j], FX_IL)))&0xFFFFFFFF);
  //   // write_mem(&fftHW_lmem[SYNC_VAR_SIZE+j], val_64);
  //   write_mem(&fftHW_li_mem[j], val_64);

  //   SDEBUG(if (j < 64) { 
	//     printf("FFT_IN_DATA %u : %f\n", j, data[j]);
  //     });
  // }
 #ifdef INT_TIME

  temp2 = get_counter();
  fft_cvtin_cycles += temp2-temp;
  temp = get_counter();
 #endif // INT_TIME

  DEBUG(printf("calling fft in hw\n"));
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
 
  update_fft_prod_rdy();

  // //for (int j = 0; j < 2 * fftHW_len; j++) {
  //   //BM
  // // for (int j = 0; j < 2 * RADAR_N; j++) {
  // for (int j = 0; j < 2 * RADAR_N; j+=2) {
  //   // data[j] = (float)fx2float(fftHW_lmem[j], FX_IL);
  //   //printf("%u,0x%08x,%f\n", j, fftHW_lmem[j], data[j]);

  //   uint64_t val_64 = read_mem(&fftHW_lo_mem[j]); // SYNC_VAR_SIZE+acc_len+
  //   // uint64_t val_64 = read_mem(&fftHW_lmem[SYNC_VAR_SIZE+acc_len+j]);

  //   data[j] = (float)fx2float((int)(val_64&0xFFFFFFFF), FX_IL);
  //   data[j+1] = (float)fx2float((int)((val_64>>32)&0xFFFFFFFF), FX_IL);

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
  // printf("fft_cvtin_cycles = %llu fft_br_cycles = %llu fft_cvtout_cycles = %llu fft_cycles=%llu calc_cycles=%llu\n",fft_cvtin_cycles,fft_br_cycles,fft_cvtout_cycles, fft_cycles, calc_cycles);
 #ifdef INT_TIME
  temp2 = get_counter();
  cdfmcw_cycles += temp2 - temp;
 #endif // INT_TIME
  //printf("max_psd = %f  vs %f\n", max_psd, 1e-10*pow(8192,2));
  if (max_psd > RADAR_psd_threshold) {
    return distance;
  } else {
    return INFINITY;
  }
}

