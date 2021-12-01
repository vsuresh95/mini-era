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
uint64_t start_iter_cv;
uint64_t stop_iter_cv;
uint64_t start_iter_rad;
uint64_t stop_iter_rad;
uint64_t start_iter_vit;
uint64_t stop_iter_vit;
uint64_t start_exec_cv;
uint64_t stop_exec_cv;
uint64_t start_exec_rad;
uint64_t stop_exec_rad;
uint64_t start_exec_vit;
uint64_t stop_exec_vit;
uint64_t stop_prog;

extern unsigned use_device_number;

bool_t all_obstacle_lanes_mode = false;

unsigned time_step = 0;         // The number of elapsed time steps
unsigned max_time_steps = 5000; // The max time steps to simulate (default to 5000)

struct esp_device *espdevs;
struct esp_device *fft_dev, *vit_dev;
int ndev;

int main(int argc, char *argv[])
{
  vehicle_state_t vehicle_state;
  label_t label;
  distance_t distance;
  message_t message;

  int opt;

  // replaces sim opt "-f 0"
  crit_fft_samples_set = 0;
  printf("Using Radar Dictionary samples set %u for the critical FFT tasks\n", crit_fft_samples_set);

  // replaces sim opt "-v 2"
  vit_msgs_size = 2;
  printf("Using viterbi message size %u = %s\n", vit_msgs_size, vit_msgs_size_str[vit_msgs_size]);

  printf("Using %u maximum time steps (simulation)\n", max_time_steps);
  //BM: Commenting
  //printf("Using viterbi messages per step behavior %u = %s\n", vit_msgs_per_step, vit_msgs_per_step_str[vit_msgs_per_step]);

#ifdef HW_FFT
  // find the FFT device
	ndev = probe(&espdevs, VENDOR_SLD, SLD_FFT, FFT_DEV_NAME);
	if (ndev == 0) {
		printf("fft not found\n");
		return 0;
	}

	fft_dev = &espdevs[0];
#endif // if HW_FFT

#ifdef HW_VIT
  // find the Viterbi device
	ndev = probe(&espdevs, VENDOR_SLD, SLD_VITDODEC, VIT_DEV_NAME);
	if (ndev == 0) {
		printf("vitdodec not found\n");
		return 0;
	}

	vit_dev = &espdevs[0];
#endif // if HW_VIT

  //BM: Runs sometimes do not reset timesteps unless in main()
  time_step = 0;   
  printf("Doing initialization tasks...\n");

  // initialize radar kernel - set up buffer
  printf("Initializing the Radar kernel...\n");
  if (!init_rad_kernel())
  {
    printf("Error: the radar kernel couldn't be initialized properly.\n");
    return 1;
  }

  // initialize viterbi kernel - set up buffer
  printf("Initializing the Viterbi kernel...\n");
  if (!init_vit_kernel())
  {
    printf("Error: the Viterbi decoding kernel couldn't be initialized properly.\n");
    return 1;
  }

  /* We assume the vehicle starts in the following state:
   *  - Lane: center
   *  - Speed: 50 mph
   */
  vehicle_state.active  = true;
  vehicle_state.lane    = center;
  vehicle_state.speed   = 50;
  printf("Vehicle starts with the following state: active: %u lane %u speed %d\n", vehicle_state.active, vehicle_state.lane, (int) vehicle_state.speed);

  printf("Starting the main loop...\n");
  start_prog = get_counter();
 
 #if 1
  // hardcoded for 100 trace samples
  for (int i = 0; i < 10; i++)
  {
    if (!read_next_trace_record(vehicle_state))
    {
      break;
    }
    
    printf("Vehicle_State: Lane %u %s Speed %d\n", vehicle_state.lane, lane_names[vehicle_state.lane], (int) vehicle_state.speed);

    /* The computer vision kernel performs object recognition on the
     * next image, and returns the corresponding label. 
     * This process takes place locally (i.e. within this car).
     */
    start_iter_cv = get_counter();
    label_t cv_tr_label = iterate_cv_kernel(vehicle_state);
    stop_iter_cv = get_counter();

    /* The radar kernel performs distance estimation on the next radar
     * data, and returns the estimated distance to the object.
     */
    start_iter_rad = get_counter();
    radar_dict_entry_t* rdentry_p = iterate_rad_kernel(vehicle_state);
    stop_iter_rad = get_counter();

    distance_t rdict_dist = rdentry_p->distance;
    float * ref_in = rdentry_p->return_data;
    float radar_inputs[2*RADAR_N];

    printf("\nCopying radar inputs...\n");

    for (int ii = 0; ii < 2*RADAR_N; ii++) {
      radar_inputs[ii] = ref_in[ii];

      #if 0
       if (ii < 64) { printf("radar_inputs[%u] = %x %x %x\n", ii, radar_inputs[ii], ref_in[ii], rdentry_p->return_data[ii]); }
      #endif
    }

    /* The Viterbi decoding kernel performs Viterbi decoding on the next
     * OFDM symbol (message), and returns the extracted message.
     * This message can come from another car (including, for example,
     * its 'pose') or from the infrastructure (like speed violation or
     * road construction warnings). For simplicity, we define a fix set
     * of message classes (e.g. car on the right, car on the left, etc.)
     */
    start_iter_vit = get_counter();
    vit_dict_entry_t* vdentry_p = iterate_vit_kernel(vehicle_state);
    stop_iter_vit = get_counter();

    // Here we will simulate multiple cases, based on global vit_msgs_behavior
    int num_vit_msgs = 1;   // the number of messages to send this time step (1 is default) 
    //BM: vit_msgs_per_step is not being passed from commandline
    // switch(vit_msgs_per_step) {
    //   case 1: num_vit_msgs = total_obj; break;
    //   case 2: num_vit_msgs = total_obj + 1; break;
    // }

    // EXECUTE the kernels using the now known inputs 
    start_exec_cv = get_counter();
    //BM: added print
    printf("\nInvoking execute_cv_kernel...\n");
    label = execute_cv_kernel(cv_tr_label);
    //BM: added print
    printf("\nBack from execute_cv_kernel...\n");
    stop_exec_cv = get_counter();

    start_exec_rad = get_counter();
    //BM: added print
    printf("\nInvoking execute_rad_kernel...\n");
    distance = execute_rad_kernel(radar_inputs);
    //BM: added print
    printf("\nBack from execute_rad_kernel... distance = %d\n", (int) distance);
    stop_exec_rad = get_counter();

    start_exec_vit = get_counter();
    //BM: added print
    printf("\nInvoking execute_vit_kernel...\n");
    message = execute_vit_kernel(vdentry_p, num_vit_msgs);
    //BM: added print
    printf("\nBack from execute_vit_kernel... message = %d\n", message);
    stop_exec_vit = get_counter();

    // POST-EXECUTE each kernels to gather stats, etc.
    post_execute_cv_kernel(cv_tr_label, label);
    post_execute_rad_kernel(rdentry_p->set, rdentry_p->index_in_set, rdict_dist, distance);
    for (int mi = 0; mi < num_vit_msgs; mi++) {
      post_execute_vit_kernel(vdentry_p->msg_id, message);
    }

    /* The plan_and_control() function makes planning and control decisions
     * based on the currently perceived information. It returns the new
     * vehicle state.
     */
    printf("Time Step %3u : Calling Plan and Control with message %u and distance %d\n", time_step, message, (int) distance);
    vehicle_state = plan_and_control(label, distance, message, vehicle_state);
    printf("New vehicle state: lane %u speed %d\n\n", vehicle_state.lane, (int) vehicle_state.speed);

    time_step++;
  }
  #endif

  stop_prog = get_counter();

  /* All the trace/simulation-time has been completed -- Quitting... */
  printf("\nRun completed %u time steps\n\n", time_step);

  /* All the traces have been fully consumed. Quitting... */
  closeout_cv_kernel();
  closeout_rad_kernel();
  closeout_vit_kernel();

  printf("\nDone.\n");
  return 0;
}
