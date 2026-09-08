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
#include "fs.h"

// Toggle for low-level swap/fault trace output.  Set to 1 while debugging;
// set to 0 for production runs to avoid console spam.
#define SWAP_DEBUG  0
#if SWAP_DEBUG
#define SDBG  cprintf
#else
#define SDBG(...)  ((void)0)
#endif

// ---------------------------------------------------------------------------
// Local copies of walkpgdir / mappages / deallocuvm that are needed by the
// page-fault path.  The originals in vm.c are declared static so they are
// not visible here.
// ---------------------------------------------------------------------------
static pte_t *
walkpgdir(pde_t *pgdir, const void *va, int alloc)
{
  pde_t *pde;
  pte_t *pgtab;

  pde = &pgdir[PDX(va)];
  if(*pde & PTE_P){
    pgtab = (pte_t*)P2V(PTE_ADDR(*pde));
  } else {
    if(!alloc || (pgtab = (pte_t*)kalloc()) == 0)
      return 0;
    memset(pgtab, 0, PGSIZE);
    *pde = V2P(pgtab) | PTE_P | PTE_W | PTE_U;
  }
  return &pgtab[PTX(va)];
}

static int
mappages(pde_t *pgdir, void *va, uint size, uint pa, int perm)
{
  char *a, *last;
  pte_t *pte;

  a    = (char*)PGROUNDDOWN((uint)va);
  last = (char*)PGROUNDDOWN(((uint)va) + size - 1);
  for(;;){
    if((pte = walkpgdir(pgdir, a, 1)) == 0)
      return -1;
    *pte = pa | perm | PTE_P;
    if(a == last)
      break;
    a  += PGSIZE;
    pa += PGSIZE;
  }
  return 0;
}

static int
deallocuvmXV7(pde_t *pgdir, uint oldsz, uint newsz)
{
  pte_t *pte;
  uint a, pa;

  if(newsz >= oldsz)
    return oldsz;

  a = PGROUNDUP(newsz);
  for(; a < oldsz; a += PGSIZE){
    pte = walkpgdir(pgdir, (char*)a, 0);
    if(!pte)
      a = PGADDR(PDX(a)+1, 0, 0) - PGSIZE;
    else if((*pte & PTE_P) != 0){
      pa = PTE_ADDR(*pte);
      if(pa == 0)
        panic("kfree");
      kfree(P2V(pa));
      *pte = 0;
    }
  }
  return newsz;
}

// ---------------------------------------------------------------------------
// swap_page_from_pte
//   Given a PTE that points to a resident physical page:
//     1. Allocate 8 consecutive disk sectors via balloc_page().
//     2. Write the page contents to disk.
//     3. Rewrite the PTE: store the disk block-id in the upper 20 bits,
//        set PTE_SWAPPED, and clear PTE_P so that any future access to
//        this virtual address triggers a page fault.
//     4. Free the physical frame.
// ---------------------------------------------------------------------------
void
swap_page_from_pte(pte_t *pte)
{
  uint phys_addr = PTE_ADDR(*pte);
  uint disk_blk;

  if(phys_addr == 0)
    SDBG("swap_page_from_pte: physical address is zero\n");

  disk_blk = balloc_page(ROOTDEV);
  write_page_to_disk(ROOTDEV, (char*)P2V(phys_addr), disk_blk);

  // Encode the block id in the upper bits and mark the page as swapped.
  *pte  = (disk_blk << 12) | PTE_SWAPPED;
  *pte &= ~PTE_P;

  kfree(P2V(phys_addr));
  SDBG("swap_page_from_pte: done, block=%d\n", disk_blk);
}

// ---------------------------------------------------------------------------
// swap_page
//   Asks the eviction policy (select_a_victim) to pick a page, then hands
//   that PTE to swap_page_from_pte to do the actual disk write.  If the
//   policy returns NULL on the first try we call clearaccessbit() and retry
//   once (this path is only exercised by the Second Chance policy; the
//   other two should never return NULL unless the system is truly out of
//   pages).
// ---------------------------------------------------------------------------
int
swap_page(pde_t *pgdir)
{
  pte_t *pte = select_a_victim(pgdir);

  if(pte == 0) {
    SDBG("swap_page: no victim on first try, clearing access bits\n");
    clearaccessbit(pgdir);
    pte = select_a_victim(pgdir);
    if(pte == 0) {
      SDBG("swap_page: still no victim after clearing access bits\n");
      return 0;
    }
  }

  swap_page_from_pte(pte);
  lcr3(V2P(pgdir));   // flush TLB after modifying the PTE
  return 1;
}

// ---------------------------------------------------------------------------
// map_address
//   Called from the page-fault handler to bring a page back into memory.
//   Two scenarios:
//     a) The PTE has PTE_SWAPPED set  →  the page is on disk; read it back
//        into a freshly allocated frame and update the PTE.
//     b) The PTE is simply not present  →  this is a new page that was never
//        mapped (e.g. heap growth via sbrk).  Zero-fill a new frame and
//        create the mapping.
//   In either case, if kalloc() fails we evict another page first and retry.
// ---------------------------------------------------------------------------
void
map_address(pde_t *pgdir, uint addr)
{
  struct proc *curproc = myproc();
  uint cursz = curproc->sz;
  uint a     = PGROUNDDOWN(rcr2());   // page-align the faulting address

  pte_t *pte = walkpgdir(pgdir, (char*)a, 0);
  char  *mem = kalloc();

  if(mem == 0) {
    // Physical memory is full – evict a page to make room.
    swap_page(pgdir);
    mem = kalloc();
    SDBG("map_address: kalloc succeeded after swap\n");
  }

  if(pte != 0 && (*pte & PTE_SWAPPED)) {
    // ---- restore a swapped-out page ----
    int blockid = getswappedblk(pgdir, a);
    read_page_from_disk(ROOTDEV, mem, blockid);

    *pte = V2P(mem) | PTE_W | PTE_U | PTE_P;
    *pte &= ~PTE_SWAPPED;
    lcr3(V2P(pgdir));
    bfree_page(ROOTDEV, blockid);
  } else {
    // ---- brand-new page (e.g. heap growth) ----
    memset(mem, 0, PGSIZE);
    if(mappages(pgdir, (char*)a, PGSIZE, V2P(mem), PTE_P|PTE_W|PTE_U) < 0) {
      panic("map_address: mappages failed");
      deallocuvmXV7(pgdir, cursz + PGSIZE, cursz);
      kfree(mem);
    }
  }
}

// ---------------------------------------------------------------------------
// handle_pgfault  –  page-fault exception handler (T_PGFLT)
//   Increments the per-process fault counter (exposed via get_faults syscall)
//   and delegates to map_address().
// ---------------------------------------------------------------------------
void
handle_pgfault(void)
{
  struct proc *curproc = myproc();
  total_page_faults++;
  map_address(curproc->pgdir, rcr2());
}
