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
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "kernels_api.h"
#include "sim_environs.h"
#include "getopt.h"

#define TIME

#ifdef INT_TIME
extern uint64_t calc_sec;
extern uint64_t calc_usec;
extern uint64_t calc_cycles;

extern uint64_t fft_sec;
extern uint64_t fft_usec;
extern uint64_t fft_cycles;

extern uint64_t bitrev_sec;
extern uint64_t bitrev_usec;
extern uint64_t bitrev_cycles;

extern uint64_t fft_br_sec;
extern uint64_t fft_br_usec;
extern uint64_t fft_br_cycles;

extern uint64_t fft_cvtin_sec;
extern uint64_t fft_cvtin_usec;
extern uint64_t fft_cvtin_cycles;

extern uint64_t fft_cvtout_sec;
extern uint64_t fft_cvtout_usec;
extern uint64_t fft_cvtout_cycles;

extern uint64_t cdfmcw_sec;
extern uint64_t cdfmcw_usec;
extern uint64_t cdfmcw_cycles;

extern uint64_t dodec_sec;
extern uint64_t dodec_usec;
extern uint64_t dodec_cycles;

extern uint64_t depunc_sec;
extern uint64_t depunc_usec;
extern uint64_t depunc_cycles;

extern uint64_t cv_call_sec;
extern uint64_t cv_call_usec;

extern uint64_t nvdla_sec;
extern uint64_t nvdla_usec;

extern uint64_t parse_sec;
extern uint64_t parse_usec;

extern uint64_t decode_total_cycles;
extern uint64_t descram_cycles;
extern uint64_t dodec_input_cycles;

#endif

char cv_dict[256]; 
char rad_dict[256];
char vit_dict[256];

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

extern unsigned use_device_number;
unsigned fft_logn_samples;

bool_t all_obstacle_lanes_mode = false;

unsigned time_step = 0;         // The number of elapsed time steps
unsigned max_time_steps = 5000; // The max time steps to simulate (default to 5000)

void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h         : print this helpful usage info\n");
  printf("    -o         : print the Visualizer output traace information during the run\n");
  printf("    -R <file>  : defines the input Radar dictionary file <file> to use\n");
  printf("    -V <file>  : defines the input Viterbi dictionary file <file> to use\n");
  printf("    -C <file>  : defines the input CV/CNN dictionary file <file> to use\n");
  printf("    -s <N>     : Sets the max number of time steps to simulate\n");
#ifdef USE_SIM_ENVIRON
  printf("    -r <N>     : Sets the rand random number seed to N\n");
  printf("    -A         : Allow obstacle vehciles in All lanes (otherwise not in left or right hazard lanes)\n");
  printf("    -W <wfile> : defines the world environment parameters description file <wfile> to use\n");
#else
  printf("    -t <trace> : defines the input trace file <trace> to use\n");
#endif
  printf("    -f <N>     : defines which Radar Dictionary Set is used for Critical FFT Tasks\n");
  printf("               :      Each Set of Radar Dictionary Entries Can use a different sample size, etc.\n");
  printf("    -n <N>     : defines number of Viterbi messages per time step behavior:\n");
  printf("               :      0 = One message per time step\n");
  printf("               :      1 = One message per obstacle per time step\n");
  printf("               :      2 = One msg per obstacle + 1 per time step\n");
  printf("    -v <N>     : defines Viterbi message size:\n");
  printf("               :      0 = Short messages (4 characters)\n");
  printf("               :      1 = Medium messages (500 characters)\n");
  printf("               :      2 = Long messages (1000 characters)\n");
  printf("               :      3 = Max-sized messages (1500 characters)\n");
}


	 
int main(int argc, char *argv[])
{
  vehicle_state_t vehicle_state;
  label_t label;
  distance_t distance;
  message_t message;
  fft_logn_samples = 10;
#ifdef USE_SIM_ENVIRON
  char* world_desc_file_name = "default_world.desc";
#else
  char* trace_file = "traces/test_trace1.new";
#endif
  int opt;

  rad_dict[0] = '\0';
  vit_dict[0] = '\0';
  cv_dict[0]  = '\0';

  // put ':' in the starting of the
  // string so that program can
  // distinguish between '?' and ':'
  while((opt = getopt(argc, argv, ":hAot:v:n:s:r:W:R:V:C:D:f:b:")) != -1) {
    switch(opt) {
    case 'h':
      print_usage(argv[0]);
      exit(0);
    case 'A':
      all_obstacle_lanes_mode = true;
      break;
    case 'D':
      use_device_number = atoi(optarg);
      DEBUG(printf("Setting to use device number %d\n", use_device_number));
      break;
    case 'o':
      output_viz_trace = true;
      break;
    case 'R':
      snprintf(rad_dict, 255, "%s", optarg);
      break;
    case 'C':
      snprintf(cv_dict, 255, "%s", optarg);
      break;
    case 'V':
      snprintf(vit_dict, 255, "%s", optarg);
      break;
    case 's':
      max_time_steps = atoi(optarg);
      DEBUG(printf("Using %u maximum time steps (simulation)\n", max_time_steps));
      break;
    case 'b':
      fft_logn_samples = atoi(optarg);
      DEBUG(printf("fft_logn_samples: %u \n", fft_logn_samples));
      break;
    case 'f':
      crit_fft_samples_set = atoi(optarg);
      DEBUG(printf("Using Radar Dictionary samples set %u for the critical FFT tasks\n", crit_fft_samples_set));
      break;
    case 'r':
#ifdef USE_SIM_ENVIRON
      rand_seed = atoi(optarg);
#endif
      break;
    case 't':
#ifndef USE_SIM_ENVIRON
      trace_file = optarg;
      DEBUG(printf("Using trace file: %s\n", trace_file));
#endif
      break;
    case 'v':
      vit_msgs_size = atoi(optarg);
      if (vit_msgs_size >= VITERBI_MSG_LENGTHS) {
	printf("ERROR: Specified viterbi message size (%u) is larger than max (%u) : from the -v option\n", vit_msgs_size, VITERBI_MSG_LENGTHS);
	exit(-1);
      } else {
	DEBUG(printf("Using viterbi message size %u = %s\n", vit_msgs_size, vit_msgs_size_str[vit_msgs_size]));
      }
      break;
    case 'n':
      vit_msgs_per_step = atoi(optarg);
      if (vit_msgs_size >= VITERBI_MSG_LENGTHS) {
	printf("ERROR: Specified viterbi messages per time step behavior (%u) is larger than max (%u) : from the -n option\n", vit_msgs_size, VITERBI_MSG_LENGTHS);
	exit(-1);
      } else {
	DEBUG(printf("Using viterbi messages per step behavior %u = %s\n", vit_msgs_per_step, vit_msgs_per_step_str[vit_msgs_per_step]));
      }
      break;
    case 'W':
#ifdef USE_SIM_ENVIRON
      world_desc_file_name = optarg;
      DEBUG(printf("Using world description file: %s\n", world_desc_file_name));
#endif
      break;
    case ':':
      printf("option needs a value\n");
      break;
    case '?':
      printf("unknown option: %c\n", optopt);
    break;
    }
  }

  // optind is for the extra arguments
  // which are not parsed
  for(; optind < argc; optind++){
    printf("extra arguments: %s\n", argv[optind]);
  }


  if (rad_dict[0] == '\0') {
    // sprintf(rad_dict, "traces/norm_radar_16k_dictionary.dfn");
    sprintf(rad_dict, "traces/norm_radar_01k_dictionary.dfn");
  }
  if (vit_dict[0] == '\0') {
    sprintf(vit_dict, "traces/vit_dictionary.dfn");
  }
  if (cv_dict[0] == '\0') {
    sprintf(cv_dict, "traces/cnn_dictionary");
  }

#ifdef SUPER_VERBOSE
  printf("\nDictionaries:\n");
  printf("   CV/CNN : %s\n", cv_dict);
  printf("   Radar  : %s\n", rad_dict);
  printf("   Viterbi: %s\n", vit_dict);

  #ifdef USE_SIM_ENVIRON
  printf("Using world description file: %s\n", world_desc_file_name);
 #else
  printf("Using trace file: %s\n", trace_file);
 #endif
  printf("Using %u maximum time steps (simulation)\n", max_time_steps);
  printf("Using viterbi messages per step behavior %u = %s\n", vit_msgs_per_step, vit_msgs_per_step_str[vit_msgs_per_step]);
  printf("Using device number %d\n", use_device_number);
  printf("Using Radar Dictionary samples set %u for the critical FFT tasks\n", crit_fft_samples_set);
  printf("\n");

  #endif

  /* We plan to use three separate trace files to drive the three different kernels
   * that are part of mini-ERA (CV, radar, Viterbi). All these three trace files
   * are required to have the same base name, using the file extension to indicate
   * which kernel the trace corresponds to (cv, rad, vit).
   */
  /* if (argc != 2) */
  /* { */
  /*   printf("Usage: %s <trace_basename>\n\n", argv[0]); */
  /*   printf("Where <trace_basename> is the basename of the trace files to load:\n"); */
  /*   printf("  <trace_basename>.cv  : trace to feed the computer vision kernel\n"); */
  /*   printf("  <trace_basename>.rad : trace to feed the radar (FFT-1D) kernel\n"); */
  /*   printf("  <trace_basename>.vit : trace to feed the Viterbi decoding kernel\n"); */

  /*   return 1; */
  /* } */


  char cv_py_file[] = "../cv/keras_cnn/lenet.py";

  printf("Doing initialization tasks...\n");
#ifndef USE_SIM_ENVIRON
  /* Trace filename construction */
  /* char * trace_file = argv[1]; */
  //printf("Input trace file: %s\n", trace_file);

  /* Trace Reader initialization */
  if (!init_trace_reader(trace_file))
  {
    printf("Error: the trace reader couldn't be initialized properly.\n");
    return 1;
  }
#endif

  /* Kernels initialization */
  // printf("Initializing the CV kernel...\n");
  // if (!init_cv_kernel(cv_py_file, cv_dict))
  // {
  //   printf("Error: the computer vision kernel couldn't be initialized properly.\n");
  //   return 1;
  // }
  printf("Initializing the Radar kernel...\n");
  if (!init_rad_kernel(rad_dict))
  {
    printf("Error: the radar kernel couldn't be initialized properly.\n");
    return 1;
  }
  printf("Initializing the Viterbi kernel...\n");
  if (!init_vit_kernel(vit_dict))
  {
    printf("Error: the Viterbi decoding kernel couldn't be initialized properly.\n");
    return 1;
  }

  if (crit_fft_samples_set >= num_radar_samples_sets) {
    printf("ERROR : Selected FFT Tasks from Radar Dictionary Set %u but there are only %u sets in the dictionary %s\n", crit_fft_samples_set, num_radar_samples_sets, rad_dict);
    exit(-1);
  }
    
  /* We assume the vehicle starts in the following state:
   *  - Lane: center
   *  - Speed: 50 mph
   */
  vehicle_state.active  = true;
  vehicle_state.lane    = center;
  vehicle_state.speed   = 50;
  DEBUG(printf("Vehicle starts with the following state: active: %u lane %u speed %.1f\n", vehicle_state.active, vehicle_state.lane, vehicle_state.speed));

  #ifdef USE_SIM_ENVIRON
  // In simulation mode, we could start the main car is a different state (lane, speed)
  init_sim_environs(world_desc_file_name, &vehicle_state);
  #endif

/*** MAIN LOOP -- iterates until all the traces are fully consumed ***/
  time_step = 0;
 #ifdef TIME
  struct timeval stop_prog, start_prog;

  struct timeval stop_iter_rad, start_iter_rad;
  struct timeval stop_iter_vit, start_iter_vit;
  struct timeval stop_iter_cv , start_iter_cv;

  uint64_t iter_rad_sec = 0LL;
  uint64_t iter_vit_sec = 0LL;
  uint64_t iter_cv_sec  = 0LL;

  uint64_t iter_rad_usec = 0LL;
  uint64_t iter_vit_usec = 0LL;
  uint64_t iter_cv_usec  = 0LL;


  uint64_t iter_rad_cycles = 0LL;
  uint64_t iter_vit_cycles = 0LL;
  uint64_t iter_cv_cycles  = 0LL;

  struct timeval stop_exec_rad, start_exec_rad;
  struct timeval stop_exec_vit, start_exec_vit;
  struct timeval stop_exec_cv , start_exec_cv;

  uint64_t exec_rad_sec = 0LL;
  uint64_t exec_vit_sec = 0LL;
  uint64_t exec_cv_sec  = 0LL;

  uint64_t exec_rad_usec = 0LL;
  uint64_t exec_vit_usec = 0LL;
  uint64_t exec_cv_usec  = 0LL;

  uint64_t exec_rad_cycles = 0LL;
  uint64_t exec_vit_cycles = 0LL;
  uint64_t exec_cv_cycles  = 0LL;

  uint64_t copy_rad_cycles  = 0LL;
  uint64_t post_exec_rad_cycles  = 0LL;
  uint64_t post_exec_vit_cycles  = 0LL;
  uint64_t plan_control_cycles  = 0LL;

  //printf("Program run time in milliseconds %f\n", (double) (stop.tv_sec - start.tv_sec) * 1000 + (double) (stop.tv_usec - start.tv_usec) / 1000);
 #endif // TIME

  printf("Starting the main loop...\n");
  fflush(stdout);
  /* The input trace contains the per-epoch (time-step) input data */
 #ifdef TIME
  uint64_t prog_start = get_counter();
 #endif
  
 #ifdef USE_SIM_ENVIRON
  DEBUG(printf("\n\nTime Step %d\n", time_step));
  while (iterate_sim_environs(vehicle_state))
 #else //TRACE DRIVEN MODE
  //read_next_trace_record(vehicle_state);
  //while (!eof_trace_reader())
  while ( read_next_trace_record(vehicle_state) )
 #endif
  {
    DEBUG(printf("Vehicle_State: Lane %u %s Speed %.1f\n", vehicle_state.lane, lane_names[vehicle_state.lane], vehicle_state.speed));
    /* The computer vision kernel performs object recognition on the
     * next image, and returns the corresponding label. 
     * This process takes place locally (i.e. within this car).
     */
  //  #ifdef TIME
  //   gettimeofday(&start_iter_cv, NULL);
  //  #endif
  //   label_t cv_tr_label = iterate_cv_kernel(vehicle_state);
  //  #ifdef TIME
  //   gettimeofday(&stop_iter_cv, NULL);
  //   iter_cv_sec  += stop_iter_cv.tv_sec  - start_iter_cv.tv_sec;
  //   iter_cv_usec += stop_iter_cv.tv_usec - start_iter_cv.tv_usec;
  //  #endif

    /* The radar kernel performs distance estimation on the next radar
     * data, and returns the estimated distance to the object.
     */
   #ifdef TIME
    uint64_t temp = get_counter();
   #endif
    radar_dict_entry_t* rdentry_p = iterate_rad_kernel(vehicle_state);
   #ifdef TIME
    uint64_t temp2 = get_counter();
    iter_rad_cycles += temp2-temp;
   #endif
   #ifdef TIME
    temp = get_counter();
   #endif
    distance_t rdict_dist = rdentry_p->distance;
    float * ref_in = rdentry_p->return_data;
    float radar_inputs[2*RADAR_N];
    SDEBUG(printf("\nCopying radar inputs...\n"));
    for (int ii = 0; ii < 2*RADAR_N; ii++) {
      radar_inputs[ii] = ref_in[ii];
      #ifdef SUPER_VERBOSE
       if (ii < 64) { printf("radar_inputs[%2u] = %f  %f\n", radar_inputs[ii], ref_in[ii]); }
      #endif
    }

   #ifdef TIME
    temp2 = get_counter();
    copy_rad_cycles += temp2-temp;
   #endif

    /* The Viterbi decoding kernel performs Viterbi decoding on the next
     * OFDM symbol (message), and returns the extracted message.
     * This message can come from another car (including, for example,
     * its 'pose') or from the infrastructure (like speed violation or
     * road construction warnings). For simplicity, we define a fix set
     * of message classes (e.g. car on the right, car on the left, etc.)
     */
   #ifdef TIME
    temp = get_counter();
   #endif
    vit_dict_entry_t* vdentry_p = iterate_vit_kernel(vehicle_state);
   #ifdef TIME
   temp2 = get_counter();

    iter_vit_cycles += temp2-temp;
   #endif

    // Here we will simulate multiple cases, based on global vit_msgs_behavior
    int num_vit_msgs = 1;   // the number of messages to send this time step (1 is default) 
    switch(vit_msgs_per_step) {
    case 1: num_vit_msgs = total_obj; break;
    case 2: num_vit_msgs = total_obj + 1; break;
    }

  //   // EXECUTE the kernels using the now known inputs 
  //  #ifdef TIME
  //   gettimeofday(&start_exec_cv, NULL);
  //  #endif
  //   label = execute_cv_kernel(cv_tr_label);
  //  #ifdef TIME
  //   gettimeofday(&stop_exec_cv, NULL);
  //   exec_cv_sec  += stop_exec_cv.tv_sec  - start_exec_cv.tv_sec;
  //   exec_cv_usec += stop_exec_cv.tv_usec - start_exec_cv.tv_usec;
  //  #endif
   #ifdef TIME

    temp = get_counter();
   #endif
    distance = execute_rad_kernel(radar_inputs);
   #ifdef TIME
    temp2 = get_counter();
    exec_rad_cycles  += temp2-temp;

    temp = get_counter();
   #endif
    message = execute_vit_kernel(vdentry_p, num_vit_msgs);
   #ifdef TIME
    temp2 = get_counter();
    exec_vit_cycles += temp2-temp;
   #endif

    // POST-EXECUTE each kernels to gather stats, etc.
    // post_execute_cv_kernel(cv_tr_label, label);
    temp = get_counter();
    post_execute_rad_kernel(rdentry_p->set, rdentry_p->index_in_set, rdict_dist, distance);
    temp2 = get_counter();
    post_exec_rad_cycles += temp2-temp;
    // printf("Entering post_execute_vit_kernel\n");
    temp = get_counter();
    for (int mi = 0; mi < num_vit_msgs; mi++) {
      post_execute_vit_kernel(vdentry_p->msg_id, message);
    }
    temp2 = get_counter();
    post_exec_vit_cycles += temp2-temp;

    /* The plan_and_control() function makes planning and control decisions
     * based on the currently perceived information. It returns the new
     * vehicle state.
     */
    DEBUG(printf("Time Step %3u : Calling Plan and Control with message %u and distance %.1f\n", time_step, message, distance));
    temp = get_counter();
    vehicle_state = plan_and_control(label, distance, message, vehicle_state);
    temp2 = get_counter();
    plan_control_cycles += temp2-temp;
    DEBUG(printf("New vehicle state: lane %u speed %.1f\n\n", vehicle_state.lane, vehicle_state.speed));

    #ifdef TIME
    time_step++;
   #endif

    /**#ifndef USE_SIM_ENVIRON
    read_next_trace_record(vehicle_state);
    #endif**/
  }

 #ifdef TIME
 int64_t prog_stop = get_counter();
 #endif

  /* All the trace/simulation-time has been completed -- Quitting... */
  printf("\nRun completed %u time steps\n\n", time_step);

  /* All the traces have been fully consumed. Quitting... */
  // closeout_cv_kernel();

#ifdef SUPER_VERBOSE
  closeout_rad_kernel();
  closeout_vit_kernel();
#endif

  #ifdef TIME
  {
    printf("\nProgram total execution time        %lu cycles\n", (int64_t) (prog_stop-prog_start)/time_step);
    printf("  iterate_rad_kernel run time         %lu cycles\n", iter_rad_cycles/time_step);
    printf("  iterate_vit_kernel run time         %lu cycles\n", iter_vit_cycles/time_step);
    printf("  execute_rad_kernel run time         %lu cycles\n", exec_rad_cycles/time_step);
    printf("  execute_vit_kernel run time         %lu cycles\n", exec_vit_cycles/time_step);
    printf("  post_execute_rad_kernel run time    %lu cycles\n", post_exec_rad_cycles/time_step);
    printf("  post_execute_vit_kernel run time    %lu cycles\n", post_exec_vit_cycles/time_step);
    printf("  plan_and_control run time           %lu cycles\n", plan_control_cycles/time_step);
  }
 #endif // TIME
 #ifdef INT_TIME
  // These are timings taken from called routines...
  printf("\n");
  printf("  cp-rad-data run time    %lu cycles\n", copy_rad_cycles/time_step);
  printf("  fft-total   run time    %lu cycles\n", calc_cycles/time_step);
 #ifndef HW_FFT
  printf("  bit-reverse run time    %lu cycles\n", bitrev_cycles/time_step);
 #endif
  printf("  fft_cvtin   run time    %lu cycles\n", fft_cvtin_cycles/time_step);
  printf("  fft-comp    run time    %lu cycles\n", fft_cycles/time_step);
  printf("  fft_cvtout  run time    %lu cycles\n", fft_cvtout_cycles/time_step);
  printf("  calc-dist   run time    %lu cycles\n", cdfmcw_cycles/time_step);

  printf("\n");
  printf("  decode-total run time    %lu cycles\n", decode_total_cycles/time_step);
  printf("  depuncture   run time    %lu cycles\n", depunc_cycles/time_step);
  printf("  dodec-input  run time    %lu cycles\n", dodec_input_cycles/time_step);
  printf("  do-decoding  run time    %lu cycles\n", dodec_cycles/time_step);
  printf("  descram      run time    %lu cycles\n", descram_cycles/time_step);

  printf("\n");
#endif // INT_TIME

  sleep(1);
  printf("\nDone.\n");
  return 0;
}
