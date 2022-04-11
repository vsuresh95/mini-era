/*
 * Descrambler_function.c
 *
 *  Created on: Jul 11, 2019
 *      Author: Varun Mannam
 *      function: input from Viterbi decoder, gets output of descrambled bytes for
 *      the size of (psdu_size +2) and compares with the reference descrambler data
 *      Input: data bits from Viterbi decode, (psdu_szie) , reference descrambler bytes
 *      Output: nothing
 *      Source: https://github.com/IBM/dsrc/blob/master/gr-ieee802-11/lib/decode_mac.cc
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "base.h"
#include "utils.h"
#include "viterbi_parms.h"
#include "verbose.h"

typedef unsigned char   uint8_t;

void descrambler(uint8_t* in, int psdusize, char* out_msg, uint8_t* ref, uint8_t *msg) //definition
{
	uint32_t output_length = (psdusize)+2; //output is 2 more bytes than psdu_size
	uint32_t msg_length = (psdusize)-28;
	uint8_t out_result = 0;
	int state = 0; //start
	int verbose = ((ref != NULL) && (msg != NULL));

	// find the initial state of LFSR (linear feedback shift register: 7 bits) from first 7 input bits
	for(int i = 0; i < 7; i++)
	{
		if(*(in+i))
		{
			state |= 1 << (6 - i);
		}
	}

	int feedback;
	int bit;
	int index = 0;
	int mod = 0;
	int comp1, comp2, val;

	for(int i = 7; i < 232; i++)
	{
		feedback = ((!!(state & 64))) ^ (!!(state & 8));
		state = ((state << 1) & 0x7e) | feedback;
	}

	for(int i = 232; i < 240; i++)
	{
		feedback = ((!!(state & 64))) ^ (!!(state & 8));
		state = ((state << 1) & 0x7e) | feedback;

		bit = feedback ^ (*(in+i) & 0x1);
		mod = i%8;
		comp1 = (bit << mod);
		val = out_result;
		comp2 = val | comp1;
		out_result = comp2;
	}

	out_msg[3] = out_result;
}
