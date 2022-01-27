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
#endif