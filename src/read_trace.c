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
#include "tt00_100.h"

/* These are globals for the trace read parsing routines */
#define MAX_TR_LINE_SZ   256

char in_line_buf[MAX_TR_LINE_SZ];
int last_i = 0;
int in_tok = 0;
int in_lane = 0;

extern unsigned time_step;         // The number of elapsed time steps
extern unsigned max_time_steps;    // The max time steps to simulate (default to 5000)

void
get_object_token(char obj)
{
  if (in_tok == 0) { // 0 => the object character
    char objc = obj;
    lane_obj[in_lane][obj_in_lane[in_lane]] = objc;
    nearest_obj[in_lane] = objc;
    if (objc != 'N') {
      total_obj++;
    }
  } else { // a distance
    printf("ERROR : trace syntax is weird!\n");
    printf(" LINE : %s\n", in_line_buf);
    printf(" TOKN : %u hit from %s\n", last_i, &in_line_buf[last_i]);
    exit(-3);
  }
  in_tok = 1 - in_tok; // Flip to expect distance token
}

void
get_distance_token(unsigned dist)
{
  if (in_tok == 1) { // 0 => the distance value
    unsigned distv = dist;
    lane_dist[in_lane][obj_in_lane[in_lane]] = distv;
    nearest_dist[in_lane] = distv;
    obj_in_lane[in_lane]++;
  } else { // a distance
    printf("ERROR : trace syntax is weird!\n");
    printf(" LINE : %s\n", in_line_buf);
    printf(" TOKN : %u hit from %s\n", last_i, &in_line_buf[last_i]);
    exit(-4);
  }
  in_tok = 1 - in_tok; // Flip to expect object char token
}

bool_t read_next_trace_record(vehicle_state_t vs)
{
  DEBUG(printf("In read_next_trace_record\n"));

  if (time_step == max_time_steps) {
    return false;
  }

  total_obj = 0;
  for (int i = 0; i < NUM_LANES; i++) {
    obj_in_lane[i] = 0;
    nearest_obj[i]  = 'N';
    nearest_dist[i] = INF_DISTANCE;
  }
  
  last_i = 0;
  in_tok = 0;
  in_lane = 1;
  for (int i = 0; i < 3; i++) { // Scan the input line
    // Find the object tokens
    char obj = tt00_char[time_step][i];
    get_object_token(obj);

    unsigned dist = tt00_val[time_step][i];
    get_distance_token(dist);
  }

#ifdef SUPER_VERBOSE
  for (int i = 1; i < (NUM_LANES-1); i++) {
    printf("  Lane %u %8s : ", i, lane_names[i]);
    if (obj_in_lane[i] > 0) {
      for (int j = 0; j < obj_in_lane[i]; j++) {
	if (j > 0) {
	  printf(", ");
	}
	printf("%c:%u", lane_obj[i][j], lane_dist[i][j]);
      }
      printf("\n");
    } else {
      printf("%c:%u\n", 'N', (unsigned)INF_DISTANCE);
    }
  }
#endif
  return true;
}