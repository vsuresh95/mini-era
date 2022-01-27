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
#include <string.h>

#include "kernels_api.h"

uint64_t start_prog;
uint64_t stop_prog;
uint64_t intvl_prog;
uint64_t start_iter_cv;
uint64_t stop_iter_cv;
uint64_t intvl_iter_cv;
uint64_t start_iter_rad;
uint64_t stop_iter_rad;
uint64_t intvl_iter_rad;
uint64_t start_iter_vit;
uint64_t stop_iter_vit;
uint64_t intvl_iter_vit;
uint64_t start_exec_cv;
uint64_t stop_exec_cv;
uint64_t intvl_exec_cv;
uint64_t start_exec_rad;
uint64_t stop_exec_rad;
uint64_t intvl_exec_rad;
uint64_t start_exec_vit;
uint64_t stop_exec_vit;
uint64_t intvl_exec_vit;

extern uint64_t calc_start;
extern uint64_t calc_stop;
extern uint64_t calc_intvl;
extern uint64_t fft_br_stop;
extern uint64_t fft_br_intvl;
extern uint64_t fft_cvtin_start;
extern uint64_t fft_cvtin_stop;
extern uint64_t fft_cvtin_intvl;
extern uint64_t fft_start;
extern uint64_t fft_stop;
extern uint64_t fft_intvl;
extern uint64_t fft_cvtout_start;
extern uint64_t fft_cvtout_stop;
extern uint64_t fft_cvtout_intvl;
extern uint64_t fft_start;
extern uint64_t fft_stop;
extern uint64_t fft_intvl;
extern uint64_t cdfmcw_start;
extern uint64_t cdfmcw_stop;
extern uint64_t cdfmcw_intvl;

extern uint64_t depunc_start;
extern uint64_t depunc_stop;
extern uint64_t depunc_intvl;
extern uint64_t dodec_start;
extern uint64_t dodec_stop;
extern uint64_t dodec_intvl;
extern uint64_t init_vit_buffer_start;
extern uint64_t init_vit_buffer_stop;
extern uint64_t init_vit_buffer_intvl;
extern uint64_t copy_vit_buffer_start;
extern uint64_t copy_vit_buffer_stop;
extern uint64_t copy_vit_buffer_intvl;
extern uint64_t descram_start;
extern uint64_t descram_stop;
extern uint64_t descram_intvl;

extern uint64_t bitrev_start;
extern uint64_t bitrev_stop;
extern uint64_t bitrev_intvl;

extern unsigned use_device_number;

bool_t all_obstacle_lanes_mode = false;

unsigned time_step = 0;         // The number of elapsed time steps
unsigned max_time_steps = 5000; // The max time steps to simulate (default to 5000)

struct esp_device *espdevs;
struct esp_device *fft_dev, *vit_dev;
int ndev;

int main(int argc, char *argv[])
{
  volatile vehicle_state_t vehicle_state;
  volatile label_t label;
  volatile distance_t distance;
  volatile message_t message;

  volatile uint64_t* checkpoint = (volatile uint64_t*) 0x90010000;

  printf("Here\n");

 	// read hart ID
 	uint64_t hartid = read_hartid();

  intvl_prog = 0;
  intvl_iter_rad = 0;
  intvl_iter_vit = 0;
  intvl_iter_cv = 0;
  intvl_exec_rad = 0;
  intvl_exec_vit = 0;
  intvl_exec_cv = 0;
  calc_intvl = 0;
  fft_br_intvl = 0;
  bitrev_intvl = 0;
  fft_cvtin_intvl = 0;
  fft_intvl = 0;
  fft_cvtout_intvl = 0;
  cdfmcw_intvl = 0;
  depunc_intvl = 0;
  dodec_intvl = 0;
  init_vit_buffer_start = 0;
  init_vit_buffer_stop = 0;
  init_vit_buffer_intvl = 0;
  copy_vit_buffer_start = 0;
  copy_vit_buffer_stop = 0;
  copy_vit_buffer_intvl = 0;
  descram_start = 0;
  descram_stop = 0;
  descram_intvl = 0;

  // replaces sim opt "-f 0"
  crit_fft_samples_set = 0;
  SIM_DEBUG(printf("Using Radar Dictionary samples set %u for the critical FFT tasks\n", crit_fft_samples_set));

  // replaces sim opt "-v 2"
  vit_msgs_size = 2;
  SIM_DEBUG(printf("Using viterbi message size %u = %s\n", vit_msgs_size, vit_msgs_size_str[vit_msgs_size]));

  SIM_DEBUG(printf("Using %u maximum time steps (simulation)\n", max_time_steps));
  //BM: Commenting
  //printf("Using viterbi messages per step behavior %u = %s\n", vit_msgs_per_step, vit_msgs_per_step_str[vit_msgs_per_step]);

#ifdef HW_FFT
  // find the FFT device
  #ifdef TWO_CORE_SCHED
  if (hartid == 0)
  #endif
  {
	  ndev = probe(&espdevs, VENDOR_SLD, SLD_FFT, FFT_DEV_NAME);
	  if (ndev == 0) {
	  	printf("fft not found\n");
	  	return 0;
	  }

	  fft_dev = &espdevs[0];
  }
#endif // if HW_FFT

#ifdef HW_VIT
  // find the Viterbi device
  #ifdef TWO_CORE_SCHED
  if (hartid == 1)
  #endif
  {
	  ndev = probe(&espdevs, VENDOR_SLD, SLD_VITDODEC, VIT_DEV_NAME);
	  if (ndev == 0) {
	  	printf("vitdodec not found\n");
	  	return 0;
	  }

	  vit_dev = &espdevs[0];
  }
#endif // if HW_VIT

  //BM: Runs sometimes do not reset timesteps unless in main()
  time_step = 0;   
  SIM_DEBUG(printf("Doing initialization tasks...\n"));

  // initialize radar kernel - set up buffer
  SIM_DEBUG(printf("Initializing the Radar kernel...\n"));
  #ifdef TWO_CORE_SCHED
  if (hartid == 0)
  #endif
  {
    if (!init_rad_kernel())
    {
      printf("Error: the radar kernel couldn't be initialized properly.\n");
      return 1;
    }
  }

  #ifdef TWO_CORE_SCHED
  if (hartid == 0)
  {
    *checkpoint = 0xcafebeed;
    store_rl();
  }
  while(*checkpoint != 0xcafebeed)
  {
    printf("Waiting %d\n", hartid);
  }
  #endif

  // initialize viterbi kernel - set up buffer
  SIM_DEBUG(printf("Initializing the Viterbi kernel...\n"));
  #ifdef TWO_CORE_SCHED
  if (hartid == 1)
  #endif
  {
    if (!init_vit_kernel())
    {
      printf("Error: the Viterbi decoding kernel couldn't be initialized properly.\n");
      return 1;
    }
  }

  #ifdef TWO_CORE_SCHED
  if (hartid == 1)
  {
    *checkpoint = 0xcafebeaf;
    store_rl();
  }
  while(*checkpoint != 0xcafebeaf)
  {
    printf("Waiting %d\n", hartid);
  }
  #endif

  /* We assume the vehicle starts in the following state:
   *  - Lane: center
   *  - Speed: 50 mph
   */
  vehicle_state.active  = true;
  vehicle_state.lane    = center;
  vehicle_state.speed   = 50;
  SIM_DEBUG(printf("Vehicle starts with the following state: active: %u lane %u speed %d\n", vehicle_state.active, vehicle_state.lane, (int) vehicle_state.speed));

  printf("Starting the main loop...\n");

  #ifdef TWO_CORE_SCHED
  if (hartid == 0)
  #endif
  {
    start_prog = get_counter();
  }

  radar_dict_entry_t* rdentry_p;
  distance_t rdict_dist;
  float * ref_in;
  float radar_inputs[2*RADAR_N];
 
  vit_dict_entry_t* vdentry_p;

  // hardcoded for 100 trace samples
  for (int i = 0; i < ITERATIONS; i++)
  {
    #ifdef TWO_CORE_SCHED
    if (hartid == 0)
    #endif
    {
      if (!read_next_trace_record(vehicle_state))
      {
        break;
      }

      *checkpoint = (ITERATIONS+1)*10;
      store_rl();
    }

    while(*checkpoint != (ITERATIONS+1)*10);

    printf("ABC %d\n", hartid);
    
    MIN_DEBUG(printf("Vehicle_State: Lane %u %s Speed %d\n", vehicle_state.lane, lane_names[vehicle_state.lane], (int) vehicle_state.speed));

    /* The radar kernel performs distance estimation on the next radar
     * data, and returns the estimated distance to the object.
     */
    #ifdef TWO_CORE_SCHED
    if (hartid == 0)
    #endif
    {
      start_iter_rad = get_counter();
      rdentry_p = iterate_rad_kernel(vehicle_state);
      stop_iter_rad = get_counter();
      intvl_iter_rad += stop_iter_rad - start_iter_rad;

      rdict_dist = rdentry_p->distance;
      ref_in = rdentry_p->return_data;

      MIN_DEBUG(printf("\nCopying radar inputs...\n"));

      for (int ii = 0; ii < 2*RADAR_N; ii++) {
        radar_inputs[ii] = ref_in[ii];

        #if 0
         if (ii < 64) { printf("radar_inputs[%u] = %x %x %x\n", ii, radar_inputs[ii], ref_in[ii], rdentry_p->return_data[ii]); }
        #endif
      }
    }

    /* The Viterbi decoding kernel performs Viterbi decoding on the next
     * OFDM symbol (message), and returns the extracted message.
     * This message can come from another car (including, for example,
     * its 'pose') or from the infrastructure (like speed violation or
     * road construction warnings). For simplicity, we define a fix set
     * of message classes (e.g. car on the right, car on the left, etc.)
     */
    #ifdef TWO_CORE_SCHED
    if (hartid == 1)
    #endif
    {
      start_iter_vit = get_counter();
      vdentry_p = iterate_vit_kernel(vehicle_state);
      stop_iter_vit = get_counter();
      intvl_iter_vit += stop_iter_vit - start_iter_vit;
    }

    printf("DEF %d\n", hartid);
    
    // Here we will simulate multiple cases, based on global vit_msgs_behavior
    int num_vit_msgs = 1;   // the number of messages to send this time step (1 is default) 

    #ifdef TWO_CORE_SCHED
    if (hartid == 0)
    #endif
    {
      start_exec_rad = get_counter();
      //BM: added print
      MIN_DEBUG(printf("\nInvoking execute_rad_kernel...\n"));
      distance = execute_rad_kernel(radar_inputs);
      //BM: added print
      MIN_DEBUG(printf("\nBack from execute_rad_kernel... distance = %d\n", (int) distance));
      stop_exec_rad = get_counter();
      intvl_exec_rad += stop_exec_rad - start_exec_rad;
    }

    #ifdef TWO_CORE_SCHED
    if (hartid == 1)
    #endif
    {
      start_exec_vit = get_counter();
      //BM: added print
      MIN_DEBUG(printf("\nInvoking execute_vit_kernel...\n"));
      message = execute_vit_kernel(vdentry_p, num_vit_msgs);
      //BM: added print
      MIN_DEBUG(printf("\nBack from execute_vit_kernel... message = %d\n", message));
      stop_exec_vit = get_counter();
      intvl_exec_vit += stop_exec_vit - start_exec_vit;
    }

    printf("GHI %d\n", hartid);
    
    // POST-EXECUTE each kernels to gather stats, etc.
    #ifdef TWO_CORE_SCHED
    if (hartid == 0)
    #endif
    {
      post_execute_rad_kernel(rdentry_p->set, rdentry_p->index_in_set, rdict_dist, distance);
    }

    #ifdef TWO_CORE_SCHED
    if (hartid == 1)
    #endif
    {
      for (int mi = 0; mi < num_vit_msgs; mi++) {
        post_execute_vit_kernel(vdentry_p->msg_id, message);
      }

      *checkpoint = (ITERATIONS+1)*10 + 1;
      store_rl();
    }

    printf("JKL %d\n", hartid);
    
    /* The plan_and_control() function makes planning and control decisions
     * based on the currently perceived information. It returns the new
     * vehicle state.
     */
    #ifdef TWO_CORE_SCHED
    if (hartid == 0)
    #endif
    {
      while(*checkpoint != (ITERATIONS+1)*10 + 1);
    
      MIN_DEBUG(printf("Time Step %3u : Calling Plan and Control with message %u and distance %d\n", time_step, message, (int) distance));
      vehicle_state = plan_and_control(label, distance, message, vehicle_state);
      MIN_DEBUG(printf("New vehicle state: lane %u speed %d\n\n", vehicle_state.lane, (int) vehicle_state.speed));
    }

    time_step++;

    printf("MNO %d\n", hartid);
  }

  #ifdef TWO_CORE_SCHED
  if (hartid == 0)
  #endif
  {
    stop_prog = get_counter();
    intvl_prog += stop_prog - start_prog;

    /* All the trace/simulation-time has been completed -- Quitting... */
    printf("\nRun completed %u time steps\n", time_step);

    /* All the traces have been fully consumed. Quitting... */
    closeout_cv_kernel();
    closeout_rad_kernel();
    closeout_vit_kernel();

    printf("  iterate_rad_kernel run time    %lu cycles\n", intvl_iter_rad/ITERATIONS);
    printf("  iterate_vit_kernel run time    %lu cycles\n", intvl_iter_vit/ITERATIONS);

    // These are timings taken from called routines...
    printf("\n");
    printf("  execute_rad_kernel run time    %lu cycles\n", intvl_exec_rad/ITERATIONS);
    printf("  fft-total   run time    %lu cycles\n", calc_intvl/ITERATIONS);
 #ifdef HW_FFT
    printf("  bitrev      run time    %lu cycles\n", fft_br_intvl/ITERATIONS);
 #else 
    printf("  bit-reverse run time    %lu cycles\n", bitrev_intvl/ITERATIONS);
 #endif
    printf("  fft_cvtin   run time    %lu cycles\n", fft_cvtin_intvl/ITERATIONS);
    printf("  fft-comp    run time    %lu cycles\n", fft_intvl/ITERATIONS);
    printf("  fft_cvtout  run time    %lu cycles\n", fft_cvtout_intvl/ITERATIONS);
    printf("  calc-dist   run time    %lu cycles\n", cdfmcw_intvl/ITERATIONS);

    printf("\n");
    printf("  execute_vit_kernel run time    %lu cycles\n", intvl_exec_vit/ITERATIONS);
    printf("  init_vit_buffer run time    %lu cycles\n", init_vit_buffer_intvl/ITERATIONS);
    printf("  depuncture  run time    %lu cycles\n", depunc_intvl/ITERATIONS);
    printf("  do-decoding run time    %lu cycles\n", dodec_intvl/ITERATIONS);
    printf("  copy_vit_buffer run time    %lu cycles\n", copy_vit_buffer_intvl/ITERATIONS);
    printf("  descram run time    %lu cycles\n", descram_intvl/ITERATIONS);

    printf("\nProgram total execution time     %lu cycles\n", intvl_prog/ITERATIONS);

    printf("\nDone.\n");
  }
  return 0;
}
