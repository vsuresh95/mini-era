#ifndef INCLUDED_GET_COUNTER_H
#define INCLUDED_GET_COUNTER_H

#include <stdio.h>
#include <stdlib.h>

uint64_t get_counter();

void load_aq();

void store_rl();

uint64_t read_hartid();
#ifdef TWO_CORE_SCHED
void thread_entry(int cid, int nc);
void amo_add(volatile uint64_t* handshake, uint64_t value);
uint64_t amo_swap(volatile uint64_t* handshake, uint64_t value);
void delay(int cycles);
#endif

#endif // INCLUDED_GET_COUNTER_H