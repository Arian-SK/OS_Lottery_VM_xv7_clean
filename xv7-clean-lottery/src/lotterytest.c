#include "param.h"
#include "types.h"
#include "stat.h"
#include "user.h"
#include "fs.h"
#include "fcntl.h"
#include "syscall.h"
#include "traps.h"
#include "memlayout.h"

// ---------------------------------------------------------------------------
// Configuration knobs
//   TOTAL_PAGES   – how many 4 KB pages to allocate.  Must exceed the number
//                   of free physical frames so that swapping is exercised.
//   HOT_PERCENT   – denominator for the hot-set size.  With 10 the hot set is
//                   TOTAL_PAGES / 10, matching the 90/10 working-set rule.
//   ITERATIONS    – outer loop count.
// ---------------------------------------------------------------------------
#define PGSIZE           4096
#define TOTAL_PAGES       700
#define HOT_PERCENT        10
#define ITERATIONS       1000
#define ACCESSES_PER_ITER (TOTAL_PAGES * 2)

// ---------------------------------------------------------------------------
// Tiny PRNG – seeded from uptime() each call so it drifts naturally over the
// benchmark.  Not cryptographic; just needs enough variation to spread
// accesses across the hot and cold sets.
// ---------------------------------------------------------------------------
static uint randstate;

static uint
prand(void)
{
  randstate = uptime();
  if(randstate == 0)
    randstate = 1;
  randstate = randstate * 1664525 + 1013904223;
  return randstate;
}

int
main(int argc, char *argv[])
{
  char *pages[TOTAL_PAGES];
  uint  i, hot_pages, cold_pages, iter, j, idx, r;
  int   sum = 0;
  int   faults, start_faults, last_faults;

  printf(1, "lotterytest: %d pages\n", TOTAL_PAGES);

  // ---- allocate and touch every page so it is resident ---------------------
  for(i = 0; i < TOTAL_PAGES; i++) {
    pages[i] = malloc(PGSIZE);
    if(pages[i] == 0)
      goto failed;
    // Write sentinel bytes at the start and end of each page so that the
    // kernel must back each one with a real frame.
    pages[i][0]          = (char)i;
    pages[i][PGSIZE - 1] = (char)(i ^ 0x5a);
  }

  // hot_pages = 10 % of total  (the "working set")
  hot_pages  = TOTAL_PAGES / HOT_PERCENT;
  if(hot_pages < 1)
    hot_pages = 1;
  cold_pages = TOTAL_PAGES - hot_pages;

  start_faults = get_faults();
  last_faults  = start_faults;
  printf(1, "starting faults=%d\n", start_faults);

  // ---- main access loop ---------------------------------------------------
  for(iter = 1; iter <= ITERATIONS; iter++) {
    for(j = 0; j < ACCESSES_PER_ITER; j++) {
      r = prand();

      // 90 % of accesses go to the hot set, 10 % to the cold set.
      if(cold_pages == 0)
        idx = r % hot_pages;
      else if((r % 10) != 0)
        idx = r % hot_pages;          // hot
      else
        idx = hot_pages + (r % cold_pages);  // cold

      // Read-modify-write so the compiler cannot optimise the access away.
      pages[idx][0] ^= 1;
      sum += pages[idx][0];
    }

    // Print a progress line every 10 iterations so that the output can be
    // parsed later for graphs.  Format matches what the project spec asks for.
    if((iter % 10) == 0) {
      faults = get_faults();
      printf(1, "iter %d: faults=%d (+%d, since start +%d)\n",
             iter, faults, faults - last_faults, faults - start_faults);
      last_faults = faults;
    }
  }

  faults = get_faults();
  printf(1, "done: sum=%d faults=%d (since start +%d)\n",
         sum, faults, faults - start_faults);
  exit();

failed:
  printf(1, "test failed!\n");
  exit();
}
