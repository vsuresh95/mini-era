#include "get_counter.h"

uint64_t get_counter() {
  uint64_t counter;
  asm volatile (
    "li t0, 0;"
    "csrr t0, mcycle;"
    "mv %0, t0"
    : "=r" ( counter )
    :
    : "t0"
  );

  return counter;
}

void load_aq() {
#if(SPANDEX_MODE == 2)
	asm volatile ("fence r, r");
#endif
}

void store_rl() {
#if(SPANDEX_MODE > 1)
	asm volatile ("fence w, w");
#endif
}

uint64_t read_hartid() {
	uint64_t hartid;

	asm volatile (
		"csrr %0, mhartid"
		: "=r" (hartid)
	);

	return hartid;
}	

#ifdef TWO_CORE_SCHED
void thread_entry(int cid, int nc) {
	return;
}

void amo_add(volatile uint64_t* handshake, uint64_t value) {
 	asm volatile (
 		"mv t0, %1;"
 		"mv t2, %0;"
 		"amoadd.d.aqrl t1, t0, (t2);"
 		:
 		: "r" (handshake), "r" (value)
 		: "t0", "t1", "t2", "memory"
 	);
}

uint64_t amo_swap(volatile uint64_t* handshake, uint64_t value) {
 	uint64_t old_val;

 	asm volatile (
 		"mv t0, %2;"
 		"mv t2, %1;"
 		"amoswap.d.aqrl t1, t0, (t2);"
 		"mv %0, t1"
 		: "=r" (old_val)
 		: "r" (handshake), "r" (value)
 		: "t0", "t1", "t2", "memory"
 	);
 	return old_val;
}

void delay(int cycles) {
 	for (int i = 0; i < cycles; i++) {
 		asm volatile ("nop");
 	}
}
#endif