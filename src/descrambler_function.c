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
	uint8_t out[output_length];
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

	out[29] = 0;

	out[0] = state; //initial value
	int feedback;
	int bit;
	int index = 0;
	int mod = 0;

#ifdef DOUBLE_WORD
	uint8_t value_8[8];
	int64_t value_64;

	{
		feedback = ((!!(state & 64))) ^ (!!(state & 8));
		bit = feedback ^ (*(in+7) & 0x1);
		index = 7/8;
		mod =  7%8;
		int comp1, comp2, val;
		comp1 = (bit << mod);
		val = out[index];
		comp2 = val | comp1;
		out[index] =  comp2;
		state = ((state << 1) & 0x7e) | feedback;
	}

	for(int i = 8; i < 240; i+=8)
	{
#if (VIT_SPANDEX_MODE == 2)
		void* dst = (void*)((uint64_t) in+i);

		asm volatile (
			"mv t0, %1;"
			".word 0x2002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#elif (VIT_SPANDEX_MODE > 2)
		void* dst = (void*)((uint64_t) in+i);

		asm volatile (
			"mv t0, %1;"
			".word 0x4002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#else
		void* dst = (void*)((uint64_t) in+i);

		asm volatile (
			"mv t0, %1;"
			".word 0x0002B30B;"
			"mv %0, t1"
			: "=r" (value_64)
			: "r" (dst)
			: "t0", "t1", "memory"
		);
#endif
		{
			value_8[0] = (value_64 >> 0) & 0xFF;
			value_8[1] = (value_64 >> 8) & 0xFF;
			value_8[2] = (value_64 >> 16) & 0xFF;
			value_8[3] = (value_64 >> 24) & 0xFF;
			value_8[4] = (value_64 >> 32) & 0xFF;
			value_8[5] = (value_64 >> 40) & 0xFF;
			value_8[6] = (value_64 >> 48) & 0xFF;
			value_8[7] = (value_64 >> 56) & 0xFF;

			for (int j = 0; j < 8; j++)
			{
				feedback = ((!!(state & 64))) ^ (!!(state & 8));
				bit = feedback ^ (value_8[j] & 0x1);
				index = i/8;
				mod = j;
				int comp1, comp2, val;
				comp1 = (bit << mod);
				val = out[index];
				comp2 = val | comp1;
				out[index] =  comp2;
				state = ((state << 1) & 0x7e) | feedback;
			}
		}
	}
#else
	for(int i = 7; i < (psdusize*8)+16; i++) // 2 bytes more than psdu_size -> convert to bits
	{
		feedback = ((!!(state & 64))) ^ (!!(state & 8));
		bit = feedback ^ (*(in+i) & 0x1);
		index = i/8;
		mod =  i%8;
		int comp1, comp2, val; //, comp3;
		comp1 = (bit << mod);
		val = out[index];
		comp2 = val | comp1;
		out[index] =  comp2;
		//comp3 = out[index];
		state = ((state << 1) & 0x7e) | feedback;
	}
#endif

	out_msg[3] = out[29];

	if (verbose) {
	  DEBUG(printf("\n"));
	  DEBUG(printf(">>>>>> Descrambler output is here: >>>>>> \n"));
	  int  des_error_count = 0;
	  for (int i = 0; i < output_length ; i++)
	    {
	      if (out[i] != *(ref+i))
		{
		  //DEBUG(printf(">>>>>> Miscompare: descrambler[%d] = %u vs %u = EXPECTED_VALUE[%d]\n", i, out[i], *(ref+i), i));
		  des_error_count++;
		}
	    }
	  if (des_error_count !=0)
	    {
	      DEBUG(printf(">>>>>> Mismatch in the descrambler block, please check the inputs and algorithm one last time. >>>>>> \n"));
	    }
	  else
	    {
	      DEBUG(printf("!!!!!! Great Job, descrambler algorithm works fine for the given configuration. !!!!!! \n"));
	    }
	  DEBUG(printf("\n"));
	  DEBUG(printf(">>>>>> Decoded text message is here: >>>>>> \n"));

	  for (int i = 0; i< msg_length; i++)
	    {
	      DEBUG(printf("%c", out_msg[i]));	
	    }
	  DEBUG(printf("\n"));

	  int  msg_error_count = 0;
	  for (int i = 0; i < msg_length ; i++)
	    {
	      if (out_msg[i] != *(msg+i))
		{
		  DEBUG(printf(">>>>>> Miscompare: text_msg[%c] = %c vs %c = EXPECTED_VALUE[%c]\n", i, out_msg[i], *(msg+i), i));
		  msg_error_count++;
		}
	    }
	  if (msg_error_count !=0)
	    {
	      DEBUG(printf(">>>>>> Mismatch in the text message, please check the inputs and algorithm one last time. >>>>>> \n"));
	    }
	  else
	    {
	      DEBUG(printf("!!!!!! Great Job, text message decoding algorithm works fine for the given configuration. !!!!!! \n"));
	    } 
	  DEBUG(printf("\n"));
	}
}
