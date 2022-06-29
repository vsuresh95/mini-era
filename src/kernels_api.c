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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kernels_api.h"
#include "read_trace.h"
#include "vit_dictionary.h"
#include "norm_radar_01k_dictionary.h"

/* These are types, functions, etc. required for VITERBI */
#include "viterbi_flat.h"

/* Size of the contiguous chunks for scatter/gather */
#define BIT(nr) (1UL << (nr))
#define CHUNK_SHIFT 20
#define CHUNK_SIZE BIT(CHUNK_SHIFT)
#define NCHUNK(_sz) ((_sz % CHUNK_SIZE == 0) ?		\
			(_sz / CHUNK_SIZE) :		\
			(_sz / CHUNK_SIZE) + 1)

static unsigned DMA_WORD_PER_BEAT(unsigned _st)
{
        return (sizeof(void *) / _st);
}

extern unsigned time_step;

unsigned use_device_number = 0; // Default to /dev/*_stratus.0

char* lane_names[NUM_LANES] = {"LHazard", "Left", "Center", "Right", "RHazard" };
char* message_names[NUM_MESSAGES] = {"Safe_L_or_R", "Safe_R_only", "Safe_L_only", "Unsafe_L_or_R" };
char* object_names[NUM_OBJECTS] = {"Nothing", "Car", "Truck", "Person", "Bike" };

unsigned fft_logn_samples = 10; // Defaults to 1k samples

unsigned total_obj; // Total non-'N' obstacle objects across all lanes this time step
unsigned obj_in_lane[NUM_LANES]; // Number of obstacle objects in each lane this time step (at least one, 'n')
unsigned lane_dist[NUM_LANES][MAX_OBJ_IN_LANE]; // The distance to each obstacle object in each lane
char     lane_obj[NUM_LANES][MAX_OBJ_IN_LANE]; // The type of each obstacle object in each lane

char     nearest_obj[NUM_LANES]  = { 'N', 'N', 'N', 'N', 'N' };
float    nearest_dist[NUM_LANES] = { INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE };

unsigned hist_total_objs[NUM_LANES * MAX_OBJ_IN_LANE];

unsigned rand_seed = 0; // Only used if -r <N> option set

float IMPACT_DISTANCE = 50.0; // Minimum distance at which an obstacle "impacts" MyCar (collision case)

/* These are some top-level defines needed for CV kernel */
unsigned label_match[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0};  // Times CNN matched dictionary
unsigned label_lookup[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0}; // Times we used CNN for object classification
unsigned label_mismatch[NUM_OBJECTS][NUM_OBJECTS] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
  
cv_dictionary_t the_cv_image_dict;

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

uint64_t descram_start;
uint64_t descram_stop;
uint64_t descram_intvl;

#ifdef HW_VIT
// These are Viterbi Harware Accelerator Variales, etc.
char VIT_DEVNAME[128];

int vitHW_fd;
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

struct vitdodec_access vitHW_desc;

static void init_vit_parameters()
{
	//printf("Doing init_vit_parameters\n");
	if (DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)) == 0) {
		vitHW_in_words_adj  = 24852;
		vitHW_out_words_adj = 18585;
	} else {
		vitHW_in_words_adj  = round_up(24852, DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)));
		vitHW_out_words_adj = round_up(18585, DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)));
	}
	vitHW_in_len = vitHW_in_words_adj;
	vitHW_out_len =  vitHW_out_words_adj;
	vitHW_in_size = vitHW_in_len * sizeof(vitHW_token_t);
	vitHW_out_size = vitHW_out_len * sizeof(vitHW_token_t);
	vitHW_out_offset = vitHW_in_len;
	vitHW_size = (vitHW_out_offset * sizeof(vitHW_token_t)) + vitHW_out_size;
}

#endif

#ifdef HW_FFT

char FFT_DEVNAME[128];

int fftHW_fd;
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

struct fftHW_access fftHW_desc;

const float FFT_ERR_TH = 0.05;

/* User-defined code */
static void init_fft_parameters()
{
	int len = 0x1 << fft_logn_samples;
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
	fftHW_out_offset = fftHW_in_len;
	fftHW_size = (fftHW_out_offset * sizeof(fftHW_token_t)) + fftHW_out_size;
}
#endif

unsigned **ptable_fft;
unsigned **ptable_vit;

unsigned **ptable_sense_fft;
unsigned **ptable_sense_vit;

int64_t *input_rad_mem;
int64_t *input_vit_mem;

extern void descrambler(uint8_t* in, int psdusize, char* out_msg, uint8_t* ref, uint8_t *msg);

status_t init_rad_kernel()
{
  DEBUG(printf("In init_rad_kernel...\n"));

  init_calculate_peak_dist(fft_logn_samples);

  // hardcoded based on norm_radar_01_dictionary.h
  num_radar_samples_sets = 1;
  radar_dict_items_per_set = 12;

  SIM_DEBUG(printf("  There are %u dictionary sets of %u entries each\n", num_radar_samples_sets, radar_dict_items_per_set));
    
  //BM: Allocating memory space for the radar return dict
  // the_radar_return_dict = (radar_dict_entry_t**)aligned_malloc(num_radar_samples_sets*radar_dict_items_per_set*sizeof(radar_dict_entry_t));
  the_radar_return_dict = (radar_dict_entry_t**) aligned_malloc(num_radar_samples_sets * sizeof(radar_dict_entry_t));
  if (the_radar_return_dict == NULL) {
    printf("ERROR : Cannot allocate Radar Trace Dictionary memory space\n");
    return error;
  }

  SIM_DEBUG(printf("the_radar_return_dict = %p, sizeof(radar_dict_entry_t) = %d\n", the_radar_return_dict, sizeof(radar_dict_entry_t)));
   
  for (int si = 0; si < num_radar_samples_sets; si++) {
    the_radar_return_dict[si] = (radar_dict_entry_t*) aligned_malloc(radar_dict_items_per_set * sizeof(radar_dict_entry_t));
    if (the_radar_return_dict[si] == NULL) {
      printf("ERROR : Cannot allocate Radar Trace Dictionary memory space for set %u\n", si);
      return error;
    }

    SIM_DEBUG(printf("the_radar_return_dict[%d] = %p, sizeof(radar_dict_entry_t) = %d, MAX_RADAR_N = %d\n", si, the_radar_return_dict[si], sizeof(radar_dict_entry_t), MAX_RADAR_N));
  }

  unsigned tot_dict_values = 0;
  unsigned tot_index = 0;

  for (int si = 0; si < num_radar_samples_sets; si++) {
    radar_log_nsamples_per_dict_set[si] = 10;
    //BM
    //DEBUG(printf("  Dictionary set %u entries should all have %u log_nsamples\n", si, radar_log_nsamples_per_dict_set[si]));
    SIM_DEBUG(printf("  Dictionary set %u entries should all have %u log_nsamples\n", si, radar_log_nsamples_per_dict_set[si]));

    for (int di = 0; di < radar_dict_items_per_set; di++) {
      unsigned entry_id;
      unsigned entry_log_nsamples;
      float entry_dist;
      unsigned entry_dict_values = 0;

      entry_id = di;
      entry_log_nsamples = 10;

      entry_dist = entry_dist_01k[di];

      if (radar_log_nsamples_per_dict_set[si] != entry_log_nsamples) {
	      printf("ERROR reading Radar Dictionary set %u entry %u header : Mismatch in log2 samples : %u vs %u\n", si, di, entry_log_nsamples, radar_log_nsamples_per_dict_set[si]);
	      exit(-2);
      }
	
      SIM_DEBUG(printf("  Reading rad dictionary set %u entry %u : %u %u %d\n", si, di, entry_id, entry_log_nsamples, (int)(entry_dist)));

      the_radar_return_dict[si][di].index = tot_index++;  // Set, and increment total index
      the_radar_return_dict[si][di].set = si;
      the_radar_return_dict[si][di].index_in_set = di;
      the_radar_return_dict[si][di].return_id = entry_id;
      the_radar_return_dict[si][di].log_nsamples = entry_log_nsamples;
      the_radar_return_dict[si][di].distance =  entry_dist;

	    the_radar_return_dict[si][di].return_data = &(norm_radar_01k[di][0]);

	    tot_dict_values += 2*(1<<entry_log_nsamples);
	    entry_dict_values += 2*(1<<entry_log_nsamples);

      SIM_DEBUG(printf("    Read in dict set %u entry %u with %u total values\n", si, di, entry_dict_values));
    } // for (int di across radar dictionary entries per set

    DEBUG(printf("   Done reading in Radar dictionary set %u\n", si));
  } // for (si across radar dictionary sets)

  DEBUG(printf("  Read %u sets with %u entries totalling %u values across them all\n", num_radar_samples_sets, radar_dict_items_per_set, tot_dict_values));
  SIM_DEBUG(printf("  Read %u sets with %u entries totalling %u values across them all\n", num_radar_samples_sets, radar_dict_items_per_set, tot_dict_values));

  // Initialize hist_pct_errs values
  // Clear the inputs (injected) histogram
  MIN_DEBUG(
    for (int si = 0; si < num_radar_samples_sets; si++) {
      for (int di = 0; di < radar_dict_items_per_set; di++) {
        hist_distances[si][di] = 0;
        for (int i = 0; i < 5; i++) {
	       hist_pct_errs[si][di][i] = 0;
        }
      }
    }

    for (int i = 0; i < MAX_RDICT_SAMPLE_SETS; i++) {
      for (int j = 0; j < MAX_RDICT_ENTRIES; j++) {
        radar_inputs_histogram[i][j] = 0;
      }
    }
  );

 #ifdef HW_FFT
  init_fft_parameters();

  SIM_DEBUG(printf("Allocate hardware buffer of size %d\n", fftHW_size));
  fftHW_lmem = (fftHW_token_t*)aligned_malloc(fftHW_size);
  SIM_DEBUG(printf("fftHW_lmem = %p\n", fftHW_lmem));

  fftHW_li_mem = &(fftHW_lmem[0]);
  fftHW_lo_mem = &(fftHW_lmem[fftHW_in_words_adj]);
  SIM_DEBUG(printf("Set fftHW_li_mem = %p  AND fftHW_lo_mem = %p\n", fftHW_li_mem, fftHW_lo_mem));

#ifdef HW_FFT
	// ptable_fft = aligned_malloc(NCHUNK(radar_dict_items_per_set*sizeof(radar_dict_entry_t)) * sizeof(unsigned *));
	ptable_fft = aligned_malloc(NCHUNK(fftHW_size) * sizeof(unsigned *));

	for (int i = 0; i < NCHUNK(fftHW_size); i++) {
		ptable_fft[i] = (unsigned *) &fftHW_lmem[i * (CHUNK_SIZE / sizeof(fftHW_token_t))];

    SIM_DEBUG(printf("ptable_fft[%d] = %p\n", i, ptable_fft[i]));
  }

	// Pass common configuration parameters
	iowrite32(fft_dev, SELECT_REG, ioread32(fft_dev, DEVID_REG));
	iowrite32(fft_dev, PT_ADDRESS_REG, (unsigned long) ptable_fft);
	iowrite32(fft_dev, PT_NCHUNK_REG, NCHUNK(fftHW_size));
	iowrite32(fft_dev, PT_SHIFT_REG, CHUNK_SHIFT);

#ifdef USE_FFT_SENSOR
  size_t sample_set_size = 2 * MAX_RADAR_N * sizeof(float);
  unsigned sample_words = sample_set_size/sizeof(int64_t);

  // copy radar data to FFT sensor scratchpad
  for (int i = 0; i < radar_dict_items_per_set; i++)
  {
    input_rad_mem = (int64_t*) &(the_radar_return_dict[0][i].return_data[0]);
    // printf("  memory = %p\n", input_rad_mem);

    // Allocate and populate page table
    ptable_sense_fft = aligned_malloc(NCHUNK(sample_set_size) * sizeof(unsigned *));
    
		for (int j = 0; j < NCHUNK(sample_set_size); j++)
      ptable_sense_fft[j] = (unsigned *) &input_rad_mem[j * (CHUNK_SIZE / sizeof(int64_t))];

    // printf("  ptable = %p\n", ptable_sense_fft);
    // printf("  nchunk = %lu\n", NCHUNK(sample_set_size));

    asm volatile ("fence w, w");

    // Pass common configuration parameters 
    iowrite32(fft_sense_dev, SELECT_REG, ioread32(fft_sense_dev, DEVID_REG));
#if (FFT_SPANDEX_MODE == 1)
    iowrite32(fft_sense_dev, COHERENCE_REG, ACC_COH_RECALL);
#else
    iowrite32(fft_sense_dev, COHERENCE_REG, ACC_COH_FULL);
#endif

    iowrite32(fft_sense_dev, PT_ADDRESS_REG, (unsigned long) ptable_sense_fft);
    iowrite32(fft_sense_dev, PT_NCHUNK_REG, NCHUNK(sample_set_size));
    iowrite32(fft_sense_dev, PT_SHIFT_REG, CHUNK_SHIFT);

    // Use the following if input and output data are not allocated at the default offsets
    iowrite32(fft_sense_dev, SRC_OFFSET_REG, 0);
    iowrite32(fft_sense_dev, DST_OFFSET_REG, 0);

    // Pass accelerator-specific configuration parameters
    /* <<--regs-config-->> */
    iowrite32(fft_sense_dev, SENSOR_DMA_RD_SP_OFFSET_REG, sample_words*i);
    iowrite32(fft_sense_dev, SENSOR_DMA_RD_WR_ENABLE_REG, 0);
    iowrite32(fft_sense_dev, SENSOR_DMA_RD_SIZE_REG, sample_words);
    iowrite32(fft_sense_dev, SENSOR_DMA_SRC_OFFSET_REG, 0);

    // Start a_ccelerators
    // printf("  Start...\n");
    iowrite32(fft_sense_dev, CMD_REG, CMD_MASK_START);

    // Wait for completion
    unsigned done = 0;
    while (!done) {
      done = ioread32(fft_sense_dev, STATUS_REG);
      done &= STATUS_MASK_DONE;
    }
    iowrite32(fft_sense_dev, CMD_REG, 0x0);
    // printf("  Done...\n");
  }

  // address to be used for all FFT input data streaming in from the sensor
  input_rad_mem = aligned_malloc(sample_set_size);
  printf("  input_rad_mem = %p\n", input_rad_mem);
#endif // if USE_FFT_SENSOR
#endif // if HW_FFT

  fftHW_desc.run = true;
#if (FFT_SPANDEX_MODE == 1)
	fftHW_desc.coherence = ACC_COH_RECALL;
#else
	fftHW_desc.coherence = ACC_COH_FULL;
#endif
  fftHW_desc.p2p_store = 0;
  fftHW_desc.p2p_nsrcs = 0;

 #if (USE_FFT_ACCEL_TYPE == 1) // fft_stratus
  #ifdef HW_FFT_BITREV
  fftHW_desc.do_bitrev  = FFTHW_DO_BITREV;
  #else
  fftHW_desc.do_bitrev  = FFTHW_NO_BITREV;
  #endif
  fftHW_desc.log_len    = fft_logn_samples; // fftHW_log_len;
 #elif (USE_FFT_ACCEL_TYPE == 2) // fft2_stratus
  fftHW_desc.scale_factor = 0;
  fftHW_desc.logn_samples = fft_logn_samples;
  fftHW_desc.num_ffts     = 1;
  fftHW_desc.do_inverse   = 0;
  fftHW_desc.do_shift     = 0;
  fftHW_desc.do_inverse   = 0;
 #endif
  fftHW_desc.src_offset = 0;
  fftHW_desc.dst_offset = 0;
#endif

  return success;
}

/* This is the initialization of the Viterbi dictionary data, etc.
 * The format is:
 *  <n> = number of dictionary entries (message types)
 * For each dictionary entry:
 *  n1 n2 n3 n4 n5 : OFDM parms: 
 *  m1 m2 m3 m4 m5 : FRAME parms:
 *  x1 x2 x3 ...   : The message bits (input to decode routine)
 */

status_t init_vit_kernel()
{
  DEBUG(printf("In init_vit_kernel...\n"));
  if (vit_msgs_size >= VITERBI_MSG_LENGTHS) {
    printf("ERROR: Specified too large a vit_msgs_size (-v option): %u but max is %u\n", vit_msgs_size, VITERBI_MSG_LENGTHS);
    exit(-1);
  }

  // Read the number of messages
  num_viterbi_dictionary_items = 16;
  SIM_DEBUG(printf("  There are %u dictionary entries\n", num_viterbi_dictionary_items));

  //BM
  the_viterbi_trace_dict = (vit_dict_entry_t*) aligned_malloc(num_viterbi_dictionary_items * sizeof(vit_dict_entry_t));
  if (the_viterbi_trace_dict == NULL) 
  {
    printf("ERROR : Cannot allocate Viterbi Trace Dictionary memory space\n");
    return error;
  }

  // Read in each dictionary item
  for (int i = 0; i < num_viterbi_dictionary_items; i++) 
  {
    SIM_DEBUG(printf("  Reading vit dictionary entry %u\n", i));

    int mnum = mnum_mid[i][0];
    int mid = mnum_mid[i][1];
    DEBUG(printf(" V_MSG: num %d Id %d\n", mnum, mid));

    if (mnum != i) {
      printf("ERROR : Check Viterbi Dictionary : i = %d but Mnum = %d  (Mid = %d)\n", i, mnum, mid);
      exit(-5);
    }

    the_viterbi_trace_dict[i].msg_num = mnum;
    the_viterbi_trace_dict[i].msg_id = mid;

    int in_bpsc = bpsc_cbps_dbps_encoding_rate[i][0];
    int in_cbps = bpsc_cbps_dbps_encoding_rate[i][1];
    int in_dbps = bpsc_cbps_dbps_encoding_rate[i][2];
    int in_encoding = bpsc_cbps_dbps_encoding_rate[i][3];
    int in_rate = bpsc_cbps_dbps_encoding_rate[i][4];

    DEBUG(printf("  OFDM: %d %d %d %d %d\n", in_bpsc, in_cbps, in_dbps, in_encoding, in_rate));
    the_viterbi_trace_dict[i].ofdm_p.encoding   = in_encoding;
    the_viterbi_trace_dict[i].ofdm_p.n_bpsc     = in_bpsc;
    the_viterbi_trace_dict[i].ofdm_p.n_cbps     = in_cbps;
    the_viterbi_trace_dict[i].ofdm_p.n_dbps     = in_dbps;
    the_viterbi_trace_dict[i].ofdm_p.rate_field = in_rate;

    int in_pdsu_size = pdsu_size_sym_pad_encoded_bits_data_bits[i][0];
    int in_sym = pdsu_size_sym_pad_encoded_bits_data_bits[i][1];
    int in_pad = pdsu_size_sym_pad_encoded_bits_data_bits[i][2];
    int in_encoded_bits = pdsu_size_sym_pad_encoded_bits_data_bits[i][3];
    int in_data_bits = pdsu_size_sym_pad_encoded_bits_data_bits[i][4];

    DEBUG(printf("  FRAME: %d %d %d %d %d\n", in_pdsu_size, in_sym, in_pad, in_encoded_bits, in_data_bits));
    the_viterbi_trace_dict[i].frame_p.psdu_size      = in_pdsu_size;
    the_viterbi_trace_dict[i].frame_p.n_sym          = in_sym;
    the_viterbi_trace_dict[i].frame_p.n_pad          = in_pad;
    the_viterbi_trace_dict[i].frame_p.n_encoded_bits = in_encoded_bits;
    the_viterbi_trace_dict[i].frame_p.n_data_bits    = in_data_bits;

    the_viterbi_trace_dict[i].in_bits = &(in_bits[i][0]);
  }

  //Clear the messages (injected) histogram
  MIN_DEBUG(
    for (int i = 0; i < VITERBI_MSG_LENGTHS; i++) {
      for (int j = 0; j < NUM_MESSAGES; j++) {
        viterbi_messages_histogram[i][j] = 0;
      }
    }
  

    for (int i = 0; i < NUM_LANES * MAX_OBJ_IN_LANE; i++) {
      hist_total_objs[i] = 0;
    }
  );

#ifdef HW_VIT
  init_vit_parameters();

  vitHW_lmem = (vitHW_token_t*)aligned_malloc(vitHW_size);
  SIM_DEBUG(printf("vitHW_lmem = %p\n", vitHW_lmem));

  vitHW_li_mem = &(vitHW_lmem[0]);
  vitHW_lo_mem = &(vitHW_lmem[vitHW_in_words_adj]);
  SIM_DEBUG(printf("Set vitHW_li_mem = %p  AND vitHW_lo_mem = %p\n", vitHW_li_mem, vitHW_lo_mem));

#ifdef HW_VIT
	ptable_vit = aligned_malloc(NCHUNK(vitHW_size) * sizeof(unsigned *));
	for (int i = 0; i < NCHUNK(vitHW_size); i++) {
		ptable_vit[i] = (unsigned *) &vitHW_lmem[i * (CHUNK_SIZE / sizeof(vitHW_token_t))];

    SIM_DEBUG(printf("ptable_vit[%d] = %p\n", i, ptable_vit[i]));
  }

	// Pass common configuration parameters
	iowrite32(vit_dev, SELECT_REG, ioread32(vit_dev, DEVID_REG));
	iowrite32(vit_dev, PT_ADDRESS_REG, (unsigned long) ptable_vit);
	iowrite32(vit_dev, PT_NCHUNK_REG, NCHUNK(vitHW_size));
	iowrite32(vit_dev, PT_SHIFT_REG, CHUNK_SHIFT);

#ifdef USE_VIT_SENSOR
  size_t sample_set_size = ENC_BYTES * sizeof(uint8_t);
  unsigned sample_words = sample_set_size/sizeof(int64_t);

  // copy viterbi data to Viterbi sensor scratchpad
  for (int i = 8; i < 12; i++)
  {
    input_vit_mem = (int64_t*) &(the_viterbi_trace_dict[i].in_bits[0]);
    // printf("  memory = %p\n", input_vit_mem);

    // Allocate and populate page table
    ptable_sense_vit = aligned_malloc(NCHUNK(sample_set_size) * sizeof(unsigned *));
    
		for (int j = 0; j < NCHUNK(sample_set_size); j++)
      ptable_sense_vit[j] = (unsigned *) &input_vit_mem[j * (CHUNK_SIZE / sizeof(int64_t))];

    // printf("  ptable = %p\n", ptable_sense_fft);
    // printf("  nchunk = %lu\n", NCHUNK(sample_set_size));

    asm volatile ("fence w, w");

    // Pass common configuration parameters 
    iowrite32(vit_sense_dev, SELECT_REG, ioread32(vit_sense_dev, DEVID_REG));
#if (VIT_SPANDEX_MODE == 1)
    iowrite32(vit_sense_dev, COHERENCE_REG, ACC_COH_RECALL);
#else
    iowrite32(vit_sense_dev, COHERENCE_REG, ACC_COH_FULL);
#endif

    iowrite32(vit_sense_dev, PT_ADDRESS_REG, (unsigned long) ptable_sense_vit);
    iowrite32(vit_sense_dev, PT_NCHUNK_REG, NCHUNK(sample_set_size));
    iowrite32(vit_sense_dev, PT_SHIFT_REG, CHUNK_SHIFT);

    // Use the following if input and output data are not allocated at the default offsets
    iowrite32(vit_sense_dev, SRC_OFFSET_REG, 0);
    iowrite32(vit_sense_dev, DST_OFFSET_REG, 0);

    // Pass accelerator-specific configuration parameters
    /* <<--regs-config-->> */
    iowrite32(vit_sense_dev, SENSOR_DMA_RD_SP_OFFSET_REG, sample_words*(i-8));
    iowrite32(vit_sense_dev, SENSOR_DMA_RD_WR_ENABLE_REG, 0);
    iowrite32(vit_sense_dev, SENSOR_DMA_RD_SIZE_REG, sample_words);
    iowrite32(vit_sense_dev, SENSOR_DMA_SRC_OFFSET_REG, 0);

    // Start accelerators
    // printf("  Start...\n");
    iowrite32(vit_sense_dev, CMD_REG, CMD_MASK_START);

    // Wait for completion
    unsigned done = 0;
    while (!done) {
      done = ioread32(vit_sense_dev, STATUS_REG);
      done &= STATUS_MASK_DONE;
    }
    iowrite32(vit_sense_dev, CMD_REG, 0x0);
    // printf("  Done...\n");
  }
#endif // if USE_VIT_SENSOR
#endif // if HW_VIT

  vitHW_desc.run = true;
#if (VIT_SPANDEX_MODE == 1)
	vitHW_desc.coherence = ACC_COH_RECALL;
#else
	vitHW_desc.coherence = ACC_COH_FULL;
#endif
  vitHW_desc.p2p_store = 0;
  vitHW_desc.p2p_nsrcs = 0;
#endif

  DEBUG(printf("DONE with init_vit_kernel -- returning success\n"));
  return success;
}

status_t init_cv_kernel(char* py_file, char* dict_cv)
{
  #if 0
  DEBUG(printf("In the init_cv_kernel routine\n"));
  // Generate the object image paths from the cnn_dict path

  for (unsigned i = 0; i < IMAGES_PER_OBJECT_TYPE; i++) {
    snprintf(the_cv_image_dict[no_label][i], 128, "%s/empty_%02u.jpg", dict_cv, i);
    snprintf(the_cv_image_dict[bicycle][i], 128, "%s/bike_%02u.jpg", dict_cv, i);
    snprintf(the_cv_image_dict[car][i], 128, "%s/car_%02u.jpg", dict_cv, i);
    snprintf(the_cv_image_dict[pedestrian][i], 128, "%s/person_%02u.jpg", dict_cv, i);
    snprintf(the_cv_image_dict[truck][i], 128, "%s/truck_%02u.jpg", dict_cv, i);
  }
  for (int i = 0; i < num_label_t; i++) {
    int j = 0;
    printf("the_cv_image_dict[%2u][%2u] = %s\n", i, j, the_cv_image_dict[i][j]);
    DEBUG(for (j = 1; j < IMAGES_PER_OBJECT_TYPE; j++) {
      printf("the_cv_image_dict[%2u][%2u] = %s\n", i, j, the_cv_image_dict[i][j]);
    });
  }

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
  #endif
}

label_t run_object_classification_syscall(unsigned tr_val) 
{
  #if 0
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
  #endif
}

label_t run_object_classification(unsigned tr_val) 
{
  #if 0
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
  #endif
}


label_t iterate_cv_kernel(vehicle_state_t vs)
{
  #if 0
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
  #endif
}


unsigned image_index = 0;

static inline label_t parse_output_dimg() {
  #if 0
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
  #endif
}


label_t execute_cv_kernel(label_t in_tr_val)
{
  #if 0
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
  #endif
}

void post_execute_cv_kernel(label_t tr_val, label_t cv_object)
{
  #if 0
  if (cv_object == tr_val) {
    label_match[cv_object]++;
    label_match[NUM_OBJECTS]++;
  } else {
    label_mismatch[tr_val][cv_object]++;
  }
  label_lookup[NUM_OBJECTS]++;
  label_lookup[cv_object]++;
  #endif
}



radar_dict_entry_t* iterate_rad_kernel(vehicle_state_t vs)
{
  // printf("  aaaaa\n");
  DEBUG(printf("In iterate_rad_kernel\n"));
  unsigned tr_val = nearest_dist[vs.lane] / RADAR_BUCKET_DISTANCE;  // The proper message for this time step and car-lane
  radar_inputs_histogram[crit_fft_samples_set][tr_val]++;
  MIN_DEBUG(printf("crit_fft_samples_set = %d tr_val = %d vs.lane = %d nearest_dist = %d\n", crit_fft_samples_set, tr_val, vs.lane, (int) nearest_dist[vs.lane]));
  
#ifdef USE_FFT_SENSOR 
  // copy radar data to CPU cache
  size_t sample_set_size = 2 * MAX_RADAR_N * sizeof(float);
  unsigned sample_words = sample_set_size/sizeof(int64_t);

  if (time_step == 0) {
    // Allocate and populate page table
    ptable_sense_fft = aligned_malloc(NCHUNK(sample_set_size) * sizeof(unsigned *));
  
    // printf("  bbbbb\n");

    for (int j = 0; j < NCHUNK(sample_set_size); j++)
      ptable_sense_fft[j] = (unsigned *) &input_rad_mem[j * (CHUNK_SIZE / sizeof(int64_t))];

    // printf("  ptable = %p\n", ptable_sense_fft);
    // printf("  nchunk = %lu\n", NCHUNK(sample_set_size));

    asm volatile ("fence rw, rw");

    printf("ptable_sense_fft[0] = %x\n", ptable_sense_fft[0]);
  } else if (time_step == 1) {
#ifdef USE_VIT_SENSOR
    printf("ptable_sense_vit[0] = %x\n", ptable_sense_vit[0]);
#endif
  }

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
	iowrite32(fft_sense_dev, SPANDEX_REG, spandex_config.spandex_reg);
#endif

  // Pass common configuration parameters 
  iowrite32(fft_sense_dev, SELECT_REG, ioread32(fft_sense_dev, DEVID_REG));
#if (FFT_SPANDEX_MODE == 1)
  iowrite32(fft_sense_dev, COHERENCE_REG, ACC_COH_RECALL);
#else
  iowrite32(fft_sense_dev, COHERENCE_REG, ACC_COH_FULL);
#endif

  iowrite32(fft_sense_dev, PT_ADDRESS_REG, (unsigned long) ptable_sense_fft);
  iowrite32(fft_sense_dev, PT_NCHUNK_REG, NCHUNK(sample_set_size));
  iowrite32(fft_sense_dev, PT_SHIFT_REG, CHUNK_SHIFT);

  // Use the following if input and output data are not allocated at the default offsets
  iowrite32(fft_sense_dev, SRC_OFFSET_REG, 0);
  iowrite32(fft_sense_dev, DST_OFFSET_REG, 0);

  /* <<--regs-config-->> */
  iowrite32(fft_sense_dev, SENSOR_DMA_WR_SP_OFFSET_REG, sample_words * tr_val);
  iowrite32(fft_sense_dev, SENSOR_DMA_RD_WR_ENABLE_REG, 1);
  iowrite32(fft_sense_dev, SENSOR_DMA_WR_SIZE_REG, sample_words);
  iowrite32(fft_sense_dev, SENSOR_DMA_DST_OFFSET_REG, 0);

  // Start a_ccelerators
  // printf("  Start... tr_val = %d\n", tr_val);
  iowrite32(fft_sense_dev, CMD_REG, CMD_MASK_START);

  // Wait for completion
  unsigned done = 0;
  while (!done) {
    done = ioread32(fft_sense_dev, STATUS_REG);
    done &= STATUS_MASK_DONE;
  }
  iowrite32(fft_sense_dev, CMD_REG, 0x0);
  // printf("  Done...\n");
#endif // USE_FFT_SENSOR

  //printf("Incrementing radar_inputs_histogram[%u][%u] to %u\n", crit_fft_samples_set, tr_val, radar_inputs_histogram[crit_fft_samples_set][tr_val]);
  return &(the_radar_return_dict[crit_fft_samples_set][tr_val]);
}
  


distance_t execute_rad_kernel(float * inputs)
{
  MIN_DEBUG(printf("In execute_rad_kernel\n"));

  /* 2) Conduct distance estimation on the waveform */
  MIN_DEBUG(printf("  Calling calculate_peak_dist_from_fmcw\n"));
  distance_t dist = calculate_peak_dist_from_fmcw(inputs);
  MIN_DEBUG(printf("  Returning distance = %d\n", (int) dist));
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
  
  MIN_DEBUG(printf("%d vs %d : ERROR : %lx   ABS_ERR : %lx PCT_ERR : %lx\n", (int) tr_dist, (int) dist, error, abs_err, pct_err));
  //printf("IDX: %u :: %f vs %f : ERROR : %f   ABS_ERR : %f PCT_ERR : %f\n", index, tr_dist, dist, error, abs_err, pct_err);
  if (pct_err == 0.0) {
    hist_pct_errs[set][index][0]++;
  } else if (pct_err < 0.01) {
    hist_pct_errs[set][index][1]++;
  } else if (pct_err < 0.1) {
    printf("RADAR_LT010_ERR : %d vs %d : ERROR : %d   PCT_ERR : %d\n", (int) tr_dist, (int) dist, (int) error, (int) pct_err);
    hist_pct_errs[set][index][2]++;
  } else if (pct_err < 1.00) {
    printf("RADAR_LT100_ERR : %d vs %d : ERROR : %d   PCT_ERR : %d\n", (int) tr_dist, (int) dist, (int) error, (int) pct_err);
    hist_pct_errs[set][index][3]++;
  } else {
    printf("RADAR_GT100_ERR : %d vs %d : ERROR : %d   PCT_ERR : %d\n", (int) tr_dist, (int) dist, (int) error, (int) pct_err);
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
      DEBUG(printf("  Lane %u : obj in %u is %c at %d: obj in %u is %c at %d\n", vs.lane, 
		   vs.lane-1, nearest_obj[vs.lane-1], (int) nearest_dist[vs.lane-1],
		   vs.lane+1, nearest_obj[vs.lane+1], (int) nearest_dist[vs.lane+1]));
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

#ifdef USE_VIT_SENSOR
  // copy viterbi data to the viterbi cache
  size_t sample_set_size = ENC_BYTES * sizeof(uint8_t);
  unsigned sample_words = sample_set_size/sizeof(int64_t);

  if (time_step == 0) {
    // Allocate and populate page table
    ptable_sense_vit = aligned_malloc(NCHUNK(sample_set_size) * sizeof(unsigned *));
  
    input_vit_mem = (int64_t*) &(vitHW_li_mem[72]);
    // printf("  input_vit_mem = %p\n", input_vit_mem);

    for (int j = 0; j < NCHUNK(sample_set_size); j++)
      ptable_sense_vit[j] = (unsigned *) &input_vit_mem[j * (CHUNK_SIZE / sizeof(int64_t))];

    // printf("  ptable = %p\n", ptable_sense_fft);
    // printf("  nchunk = %lu\n", NCHUNK(sample_set_size));

    asm volatile ("fence rw, rw");

    printf("ptable_sense_vit[0] = %x\n", ptable_sense_vit[0]);
  } else if (time_step == 1) {
#ifdef USE_FFT_SENSOR
    printf("ptable_sense_fft[0] = %x\n", ptable_sense_fft[0]);
#endif
  }

	// Configure Spandex request types
#if (VIT_SPANDEX_MODE > 1)
	spandex_config_t spandex_config;
	spandex_config.spandex_reg = 0;
#if (VIT_SPANDEX_MODE == 2)
	spandex_config.r_en = 1;
	spandex_config.r_type = 1;
#elif (VIT_SPANDEX_MODE == 3)
	spandex_config.r_en = 1;
	spandex_config.r_type = 2;
	spandex_config.w_en = 1;
	spandex_config.w_type = 1;
#elif (VIT_SPANDEX_MODE == 4)
	spandex_config.r_en = 1;
	spandex_config.r_type = 2;
	spandex_config.w_en = 1;
	spandex_config.w_op = 1;
	spandex_config.w_type = 1;
	spandex_config.w_cid = 5;
#endif
	iowrite32(vit_sense_dev, SPANDEX_REG, spandex_config.spandex_reg);
#endif

  // Pass common configuration parameters 
  iowrite32(vit_sense_dev, SELECT_REG, ioread32(vit_sense_dev, DEVID_REG));
#if (VIT_SPANDEX_MODE == 1)
  iowrite32(vit_sense_dev, COHERENCE_REG, ACC_COH_RECALL);
#else
  iowrite32(vit_sense_dev, COHERENCE_REG, ACC_COH_FULL);
#endif

  iowrite32(vit_sense_dev, PT_ADDRESS_REG, (unsigned long) ptable_sense_vit);
  iowrite32(vit_sense_dev, PT_NCHUNK_REG, NCHUNK(sample_set_size));
  iowrite32(vit_sense_dev, PT_SHIFT_REG, CHUNK_SHIFT);

  // Use the following if input and output data are not allocated at the default offsets
  iowrite32(vit_sense_dev, SRC_OFFSET_REG, 0);
  iowrite32(vit_sense_dev, DST_OFFSET_REG, 0);

  /* <<--regs-config-->> */
  iowrite32(vit_sense_dev, SENSOR_DMA_WR_SP_OFFSET_REG, sample_words * tr_val);
  iowrite32(vit_sense_dev, SENSOR_DMA_RD_WR_ENABLE_REG, 1);
  iowrite32(vit_sense_dev, SENSOR_DMA_WR_SIZE_REG, sample_words);
  iowrite32(vit_sense_dev, SENSOR_DMA_DST_OFFSET_REG, 0);

  // printf("  bbbbb %d\n", tr_val);

  // Start a_ccelerators
  // printf("  Start... tr_val = %d\n", tr_val);
  iowrite32(vit_sense_dev, CMD_REG, CMD_MASK_START);

  // Wait for completion
  unsigned done = 0;
  while (!done) {
    done = ioread32(vit_sense_dev, STATUS_REG);
    done &= STATUS_MASK_DONE;
  }
  iowrite32(vit_sense_dev, CMD_REG, 0x0);

  // printf("  ccccc\n");
#endif // USE_VIT_SENSOR

  return trace_msg;
}

message_t execute_vit_kernel(vit_dict_entry_t* trace_msg, int num_msgs)
{
  // Send each message (here they are all the same) through the viterbi decoder
  message_t msg = num_message_t;
  uint8_t *result;
  char     msg_text[1600]; // Big enough to hold largest message (1500?)
  for (int mi = 0; mi < num_msgs; mi++) {
    DEBUG(printf("  Calling the viterbi decode routine for message %u iter %u\n", trace_msg->msg_num, mi));
    viterbi_messages_histogram[vit_msgs_size][trace_msg->msg_id]++; 
    int n_res_char;
    //BM: Uncommenting
    result = decode(&(trace_msg->ofdm_p), &(trace_msg->frame_p), &(trace_msg->in_bits[0]), &n_res_char);
    // descramble the output - put it in result
    int psdusize = trace_msg->frame_p.psdu_size;
    DEBUG(printf("  Calling the viterbi descrambler routine\n"));
    //BM: Uncommenting

  	descram_start = get_counter();

    descrambler(result, psdusize, msg_text, NULL /*descram_ref*/, NULL /*msg*/);

  	descram_stop = get_counter();
  	descram_intvl += descram_stop - descram_start;
	
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
  DEBUG(printf("In the plan_and_control routine : label %u %s distance %d (T1 %d T1 %d T3 %d) message %u\n", 
	       label, object_names[label], (int) distance, (int) THRESHOLD_1, (int) THRESHOLD_2, (int) THRESHOLD_3, message));
  vehicle_state_t new_vehicle_state = vehicle_state;
  if (!vehicle_state.active) {
    // Our car is broken and burning, no plan-and-control possible.
    return vehicle_state;
  }
  
  if (distance <= THRESHOLD_1) {
    if (distance <= IMPACT_DISTANCE) {
      printf("WHOOPS: We've suffered a collision on time_step %u distance = %d!\n", time_step, (int) distance);
      new_vehicle_state.speed = 0.0;
      new_vehicle_state.active = false; // We should add visualizer stuff for this!
      return new_vehicle_state;
    }
    
    // Some object ahead of us that needs to be avoided.
    DEBUG(printf("  In lane %s with %c (%u) at %d (trace: %d)\n", lane_names[vehicle_state.lane], nearest_obj[vehicle_state.lane], label, (int) distance, (int) nearest_dist[vehicle_state.lane]));
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
	      DEBUG(printf("   In %s with No_Safe_Move : STOPPING\n", lane_names[vehicle_state.lane]));
	      new_vehicle_state.speed = 0.0;
	      break; /* Stop!!! */
      default:
        printf(" ERROR  In %s with UNDEFINED MESSAGE: %u\n", lane_names[vehicle_state.lane], message);
        //exit(-6);
    }
  } else {
    // No obstacle-inspired lane change, so try now to occupy the center lane
    switch (vehicle_state.lane) {
    case lhazard:
    case left:
      if ((message == safe_to_move_right_or_left) || (message == safe_to_move_right_only)) {
	      DEBUG(printf("  In %s with Can_move_Right: Moving Right\n", lane_names[vehicle_state.lane]));
	      new_vehicle_state.lane += 1;
      }
      break;
    case center:
      // No need to alter, already in the center
      break;
    case right:
    case rhazard:
      if ((message == safe_to_move_right_or_left) || (message == safe_to_move_left_only)) {
	      DEBUG(printf("  In %s with Can_move_Left : Moving Left\n", lane_names[vehicle_state.lane]));
	      new_vehicle_state.lane -= 1;
      }
      break;
    }
  } // else clause

  return new_vehicle_state;
}
/* #undef DEBUG */
/* #define DEBUG(x) */


void closeout_cv_kernel()
{
  #if 0
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
#endif
}

void closeout_rad_kernel()
{
  MIN_DEBUG(printf("\nHistogram of Radar Distances:\n"));
  MIN_DEBUG(printf("    %3s | %3s | %8s | %9s \n", "Set", "Idx", "Distance", "Occurs"));
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      MIN_DEBUG(printf("    %3u | %3u | %d | %9u \n", si, di, the_radar_return_dict[si][di].distance, hist_distances[si][di]));
    }
  }

  MIN_DEBUG(printf("\nHistogram of Radar Distance ABS-PCT-ERROR:\n"));
  unsigned totals[] = {0, 0, 0, 0, 0};
  
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      MIN_DEBUG(printf("    Set %u Entry %u Id %u Distance %d Occurs %u Histogram:\n", si, di, the_radar_return_dict[si][di].index, the_radar_return_dict[si][di].distance, hist_distances[si][di]));
      for (int i = 0; i < 5; i++) {
	      MIN_DEBUG(printf("    %7s | %9u \n", hist_pct_err_label[i], hist_pct_errs[si][di][i]));
	      totals[i] += hist_pct_errs[si][di][i];
      }
    }
  }

  MIN_DEBUG(printf("\n  TOTALS Histogram of Radar Distance ABS-PCT-ERROR:\n"));
  for (int i = 0; i < 5; i++) {
    MIN_DEBUG(printf("  %7s | %9u \n", hist_pct_err_label[i], totals[i]));
  }


  MIN_DEBUG(printf("\nHistogram of Radar Task Inputs Used:\n"));
  MIN_DEBUG(printf("    %3s | %5s | %9s \n", "Set", "Entry", "NumOccurs"));
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      MIN_DEBUG(printf("    %3u | %3u | %9u \n", si, di, radar_inputs_histogram[si][di]));
    }
  }
  MIN_DEBUG(printf("\n"));
}

void closeout_vit_kernel()
{
  // Nothing to do?

  MIN_DEBUG(printf("\nHistogram of Total Objects:\n"));
  unsigned sum = 0;
  for (int i = 0; i < NUM_LANES * MAX_OBJ_IN_LANE; i++) {
    if (hist_total_objs[i] != 0) {
      MIN_DEBUG(printf("%3u | %9u \n", i, hist_total_objs[i]));
      sum += i*hist_total_objs[i];
    }
  }
  double avg_objs = (1.0 * sum)/(1.0 * radar_total_calc); // radar_total_calc == total time steps
  MIN_DEBUG(printf("There were %d obstacles per time step (average)\n", (int) avg_objs));
  double avg_msgs = (1.0 * total_msgs)/(1.0 * radar_total_calc); // radar_total_calc == total time steps
  MIN_DEBUG(printf("There were %d messages per time step (average)\n", (int) avg_msgs));
  MIN_DEBUG(printf("There were %u bad decodes of the %u messages\n", bad_decode_msgs, total_msgs));

  MIN_DEBUG(printf("\nHistogram of Viterbi Messages:\n"));
  MIN_DEBUG(printf("    %3s | %3s | %9s \n", "Len", "Msg", "NumOccurs"));
  for (int li = 0; li < VITERBI_MSG_LENGTHS; li++) {
    for (int mi = 0; mi < NUM_MESSAGES; mi++) {
      MIN_DEBUG(printf("    %3u | %3u | %9u \n", li, mi, viterbi_messages_histogram[li][mi]));
    }
  }
  MIN_DEBUG(printf("\n"));
}
