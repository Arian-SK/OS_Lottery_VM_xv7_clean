// Physical memory allocator, intended to allocate
// memory for user processes, kernel stacks, page table pages,
// and pipe buffers. Allocates 4096-byte pages.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "spinlock.h"

void freerange(void *vstart, void *vend);
extern char end[]; // first address after kernel loaded from ELF file
                   // defined by the kernel linker script in kernel.ld

struct run {
  struct run *next;
};

// ---------------------------------------------------------------------------
// Per-frame metadata array.  Indexed by physical-page number (phys_addr / PGSIZE).
//   tickets    – lottery heat; raised by the scanner when the page is accessed,
//                decayed when idle.  Governs eviction probability.
//   alloc_seq  – monotonically increasing stamp written at allocation time.
//                The FIFO policy simply picks the page with the smallest stamp.
// ---------------------------------------------------------------------------
#define NUM_PHYS_PAGES  (PHYSTOP / PGSIZE)
struct page_meta page_info[NUM_PHYS_PAGES];
static uint      next_alloc_seq;              // bumped on every kalloc()

// Incremented inside handle_pgfault(); exposed via the get_faults syscall so
// that user-space benchmarks can track page-fault counts over time.
int total_page_faults;

struct {
  struct spinlock lock;
  int use_lock;
  struct run *freelist;
} kmem;

// Initialization happens in two phases.
// 1. main() calls kinit1() while still using entrypgdir to place just
// the pages mapped by entrypgdir on free list.
// 2. main() calls kinit2() with the rest of the physical pages
// after installing a full page table that maps them on all cores.
void
kinit1(void *vstart, void *vend)
{
  initlock(&kmem.lock, "kmem");
  kmem.use_lock = 0;
  freerange(vstart, vend);
}

void
kinit2(void *vstart, void *vend)
{
  freerange(vstart, vend);
  kmem.use_lock = 1;
}

void
freerange(void *vstart, void *vend)
{
  char *p;
  p = (char*)PGROUNDUP((uint)vstart);
  for(; p + PGSIZE <= (char*)vend; p += PGSIZE)
    kfree(p);
}

//PAGEBREAK: 21
// Free the page of physical memory pointed at by v,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(char *v)
{
  struct run *r;
  uint frame;

  if((uint)v % PGSIZE || v < end || V2P(v) >= PHYSTOP)
    panic("kfree in kalloc.c");

  // Fill with junk to catch dangling refs.
  memset(v, 1, PGSIZE);

  if(kmem.use_lock)
    acquire(&kmem.lock);

  // Wipe the per-frame metadata so a later kalloc() starts clean.
  frame = V2P(v) / PGSIZE;
  page_info[frame].tickets   = 0;
  page_info[frame].alloc_seq = 0;

  r = (struct run*)v;
  r->next = kmem.freelist;
  kmem.freelist = r;
  if(kmem.use_lock)
    release(&kmem.lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
char*
kalloc(void)
{
  struct run *r;
  uint frame;

  if(kmem.use_lock)
    acquire(&kmem.lock);
  r = kmem.freelist;
  if(r)
    kmem.freelist = r->next;
  if(kmem.use_lock)
    release(&kmem.lock);

  if(r) {
    // Stamp the frame with an initial ticket count and a fresh sequence number.
    frame = V2P(r) / PGSIZE;
    page_info[frame].tickets   = 10;
    next_alloc_seq++;
    page_info[frame].alloc_seq = next_alloc_seq;
  }

  return (char*)r;
}
