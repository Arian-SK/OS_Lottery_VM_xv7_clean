#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"
#include "paging.h"

// Interrupt descriptor table (shared by all CPUs).
struct gatedesc idt[256];
extern uint vectors[]; // in vectors.S: array of 256 entry pointers
struct spinlock tickslock;
uint ticks;

void
tvinit(void)
{
  int i;

  for(i = 0; i < 256; i++)
    SETGATE(idt[i], 0, SEG_KCODE << 3, vectors[i], 0);
  SETGATE(idt[T_SYSCALL], 1, SEG_KCODE << 3, vectors[T_SYSCALL], DPL_USER);

  initlock(&tickslock, "time");
}

void
idtinit(void)
{
  lidt(idt, sizeof(idt));
}

// ---------------------------------------------------------------------------
// Scanner – runs once per timer tick on the current process's page table.
//
// For every resident (present, not swapped) user page it checks the hardware
// PTE_A (accessed) bit:
//   set   -> the page was touched since the last scan.  Reward it with extra
//            tickets and then clear the bit so the hardware can set it again
//            on the next access.
//   clear -> the page has been idle.  Subtract a small number of tickets to
//            simulate "aging".
//
// After modifying PTE_A bits we flush the TLB (lcr3) so the CPU picks up the
// changes immediately.
//
// A compile-time knob controls how often debug output is printed.  Set
// SCAN_DEBUG_PAGES to 0 to silence it entirely; any positive value N causes
// the first N pages seen in each scan to be printed.
// ---------------------------------------------------------------------------
#define SCAN_DEBUG_PAGES  10   // set to 0 to disable ticket-state output

static void
scan_page_tables(struct proc *p)
{
  pde_t *pgdir;
  pte_t *pte;
  uint   va, pa, idx;
  int    cur_tickets, accessed;
  int    printed  = 0;
  int    cleared  = 0;           // did we clear at least one PTE_A?

  if(p == 0 || p->pgdir == 0 || p->sz == 0)
    return;

  pgdir = p->pgdir;

  for(va = 0; va < p->sz; va += PGSIZE) {
    pte = uva2pte(pgdir, va);
    if(pte == 0)
      continue;
    // Only care about pages that are physically present and not on disk.
    if(!(*pte & PTE_P) || (*pte & PTE_SWAPPED))
      continue;

    pa = PTE_ADDR(*pte);
    if(pa >= PHYSTOP)
      continue;

    idx          = pa / PGSIZE;
    cur_tickets  = page_info[idx].tickets;
    accessed     = (*pte & PTE_A) != 0;

    if(accessed) {
      cur_tickets += 10;
      if(cur_tickets > 500)
        cur_tickets = 500;
    } else {
      cur_tickets -= 5;
      if(cur_tickets < 10)
        cur_tickets = 10;
    }
    page_info[idx].tickets = cur_tickets;

    // Optional debug dump of the first N pages each scan.
    if(SCAN_DEBUG_PAGES > 0 && printed < SCAN_DEBUG_PAGES) {
      cprintf("page 0x%x : tickets=%d, accessed=%d\n", va, cur_tickets, accessed);
      printed++;
    }

    // Clear the accessed bit so the hardware will set it fresh next time.
    *pte &= ~PTE_A;
    cleared = 1;
  }

  // Flushing cr3 invalidates the entire TLB.  Only do it when we actually
  // modified at least one entry.
  if(cleared)
    lcr3(V2P(pgdir));
}

//PAGEBREAK: 41
void
trap(struct trapframe *tf)
{
  if(tf->trapno == T_SYSCALL){
    if(myproc()->killed)
      exit();
    myproc()->tf = tf;
    syscall();
    if(myproc()->killed)
      exit();
    return;
  }

  switch(tf->trapno){
  case T_PGFLT:
    handle_pgfault();
    break;

  case T_IRQ0 + IRQ_TIMER:
    if(cpuid() == 0){
      acquire(&tickslock);
      ticks++;
      wakeup(&ticks);
      release(&tickslock);

      // Run the page-table scanner once every tick for the process that is
      // currently executing on this CPU (if any).  The scanner updates ticket
      // counts and clears PTE_A bits so that the next tick can detect new
      // accesses.
      if(myproc() && myproc()->pgdir && myproc()->sz > 0)
        scan_page_tables(myproc());
    }
    lapiceoi();
    break;

  case T_IRQ0 + IRQ_IDE:
    ideintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE + 1:
    // Bochs generates spurious IDE1 interrupts.
    break;
  case T_IRQ0 + IRQ_KBD:
    kbdintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_COM1:
    uartintr();
    lapiceoi();
    break;
  case T_IRQ0 + 7:
  case T_IRQ0 + IRQ_SPURIOUS:
    cprintf("cpu%d: spurious interrupt at %x:%x\n",
            cpuid(), tf->cs, tf->eip);
    lapiceoi();
    break;

  //PAGEBREAK: 13
  default:
    if(myproc() == 0 || (tf->cs & 3) == 0){
      // In kernel, it must be our mistake.
      cprintf("unexpected trap %d from cpu %d eip %x (cr2=0x%x)\n",
              tf->trapno, cpuid(), tf->eip, rcr2());
      panic("trap");
    }
    // In user space, assume process misbehaved.
    cprintf("pid %d %s: trap %d err %d on cpu %d "
            "eip 0x%x addr 0x%x--kill proc\n",
            myproc()->pid, myproc()->name, tf->trapno,
            tf->err, cpuid(), tf->eip, rcr2());
    myproc()->killed = 1;
  }

  // Force process exit if it has been killed and is in user space.
  // (If it is still executing in the kernel, let it keep running
  // until it gets to the regular system call return.)
  if(myproc() && myproc()->killed && (tf->cs & 3) == DPL_USER)
    exit();

  // Force process to give up CPU on clock tick.
  // If interrupts were on while locks held, would need to check nlock.
  if(myproc() && myproc()->state == RUNNING &&
      tf->trapno == T_IRQ0 + IRQ_TIMER)
    yield();

  // Check if the process has been killed since we yielded.
  if(myproc() && myproc()->killed && (tf->cs & 3) == DPL_USER)
    exit();
}
