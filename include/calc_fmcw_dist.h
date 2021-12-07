/* -*-Mode: C;-*- */
#ifndef INCLUDED_CALC_FMCW_DIST_H
#define INCLUDED_CALC_FMCW_DIST_H

#include <stdint.h>
#include "get_counter.h"

/* Some global FFT Radar definitions */
#define MAX_RADAR_N  (1<<10) // Max we allow is 16k samples
extern unsigned RADAR_LOGN;  // Log2 of the number of samples
extern unsigned RADAR_N;     // The number of samples (2^LOGN)
extern float    RADAR_fs;    // Sampling Frequency
extern float    RADAR_alpha; // Chirp rate (saw-tooth)

extern size_t fftHW_in_size;

extern struct esp_device *espdevs;
extern struct esp_device *fft_dev;
extern int ndev;

/* Some function declarations */
extern void  init_calculate_peak_dist();
extern float calculate_peak_dist_from_fmcw(float* data);

#endif
