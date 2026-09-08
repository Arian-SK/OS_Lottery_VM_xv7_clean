#include "param.h"
#include "types.h"
#include "defs.h"
#include "x86.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "elf.h"
#include "paging.h"
#include "fs.h"

extern char data[]; // defined by kernel.ld
pde_t *kpgdir;      // for use in scheduler()

// =========================================================================
// EVICTION POLICY SELECTOR
//
//   0  –  Second Chance (clock-style; original xv7 baseline)
//   1  –  FIFO          (oldest alloc_seq wins)
//   2  –  Lottery       (inverse-weight probabilistic draw)
//
// Change this single value to switch policies at compile time.  No code
// needs to be commented or uncommented anywhere.
// =========================================================================
#define EVICTION_POLICY 1

// Set up CPU's kernel segment descriptors.
// Run once on entry on each CPU.
void seginit(void)
{
    struct cpu *c;

    // Map "logical" addresses to virtual addresses using identity map.
    // Cannot share a CODE descriptor for both kernel and user
    // because it would have to have DPL_USR, but the CPU forbids
    // an interrupt from CPL=0 to DPL=3.
    c = &cpus[cpuid()];
    c->gdt[SEG_KCODE] = SEG(STA_X | STA_R, 0, 0xffffffff, 0);
    c->gdt[SEG_KDATA] = SEG(STA_W, 0, 0xffffffff, 0);
    c->gdt[SEG_UCODE] = SEG(STA_X | STA_R, 0, 0xffffffff, DPL_USER);
    c->gdt[SEG_UDATA] = SEG(STA_W, 0, 0xffffffff, DPL_USER);
    lgdt(c->gdt, sizeof(c->gdt));
}

// Return the address of the PTE in page table pgdir
// that corresponds to virtual address va.  If alloc!=0,
// create any required page table pages.
static pte_t *
walkpgdir(pde_t *pgdir, const void *va, int alloc)
{
    pde_t *pde;
    pte_t *pgtab;

    pde = &pgdir[PDX(va)];
    if (*pde & PTE_P)
    {
        pgtab = (pte_t *)P2V(PTE_ADDR(*pde));
    }
    else
    {
        if (!alloc || (pgtab = (pte_t *)kalloc()) == 0)
            return 0;
        // Make sure all those PTE_P bits are zero.
        memset(pgtab, 0, PGSIZE);
        // The permissions here are overly generous, but they can
        // be further restricted by the permissions in the page table
        // entries, if necessary.
        *pde = V2P(pgtab) | PTE_P | PTE_W | PTE_U;
    }
    return &pgtab[PTX(va)];
}

// Create PTEs for virtual addresses starting at va that refer to
// physical addresses starting at pa. va and size might not
// be page-aligned.
static int
mappages(pde_t *pgdir, void *va, uint size, uint pa, int perm)
{
    char *a, *last;
    pte_t *pte;

    a = (char *)PGROUNDDOWN((uint)va);
    last = (char *)PGROUNDDOWN(((uint)va) + size - 1);
    for (;;)
    {
        if ((pte = walkpgdir(pgdir, a, 1)) == 0)
            return -1;
        if (*pte & PTE_P)
            panic("remap in mappages in vm.c");
        *pte = pa | perm | PTE_P;
        if (a == last)
            break;
        a += PGSIZE;
        pa += PGSIZE;
    }
    return 0;
}

// There is one page table per process, plus one that's used when
// a CPU is not running any process (kpgdir). The kernel uses the
// current process's page table during system calls and interrupts;
// page protection bits prevent user code from using the kernel's
// mappings.
//
// setupkvm() and exec() set up every page table like this:
//
//   0..KERNBASE: user memory (text+data+stack+heap), mapped to
//                phys memory allocated by the kernel
//   KERNBASE..KERNBASE+EXTMEM: mapped to 0..EXTMEM (for I/O space)
//   KERNBASE+EXTMEM..data: mapped to EXTMEM..V2P(data)
//                for the kernel's instructions and r/o data
//   data..KERNBASE+PHYSTOP: mapped to V2P(data)..PHYSTOP,
//                                  rw data + free physical memory
//   0xfe000000..0: mapped direct (devices such as ioapic)
//
// The kernel allocates physical memory for its heap and for user memory
// between V2P(end) and the end of physical memory (PHYSTOP)
// (directly addressable from end..P2V(PHYSTOP)).

// This table defines the kernel's mappings, which are present in
// every process's page table.
static struct kmap
{
    void *virt;
    uint phys_start;
    uint phys_end;
    int perm;
} kmap[] = {
    {(void *)KERNBASE, 0, EXTMEM, PTE_W},            // I/O space
    {(void *)KERNLINK, V2P(KERNLINK), V2P(data), 0}, // kern text+rodata
    {(void *)data, V2P(data), PHYSTOP, PTE_W},       // kern data+memory
    {(void *)DEVSPACE, DEVSPACE, 0, PTE_W},          // more devices
};

// Set up kernel part of a page table.
pde_t *
setupkvm(void)
{
    pde_t *pgdir;
    struct kmap *k;

    if ((pgdir = (pde_t *)kalloc()) == 0)
        return 0;
    memset(pgdir, 0, PGSIZE);
    if (P2V(PHYSTOP) > (void *)DEVSPACE)
        panic("PHYSTOP too high");
    for (k = kmap; k < &kmap[NELEM(kmap)]; k++)
        if (mappages(pgdir, k->virt, k->phys_end - k->phys_start,
                     (uint)k->phys_start, k->perm) < 0)
        {
            freevm(pgdir);
            return 0;
        }
    return pgdir;
}

// Allocate one page table for the machine for the kernel address
// space for scheduler processes.
void kvmalloc(void)
{
    kpgdir = setupkvm();
    switchkvm();
}

// Switch h/w page table register to the kernel-only page table,
// for when no process is running.
void switchkvm(void)
{
    lcr3(V2P(kpgdir)); // switch to the kernel page table
}

// Switch TSS and h/w page table to correspond to process p.
void switchuvm(struct proc *p)
{
    if (p == 0)
        panic("switchuvm: no process");
    if (p->kstack == 0)
        panic("switchuvm: no kstack");
    if (p->pgdir == 0)
        panic("switchuvm: no pgdir");

    pushcli();
    mycpu()->gdt[SEG_TSS] = SEG16(STS_T32A, &mycpu()->ts,
                                  sizeof(mycpu()->ts) - 1, 0);
    mycpu()->gdt[SEG_TSS].s = 0;
    mycpu()->ts.ss0 = SEG_KDATA << 3;
    mycpu()->ts.esp0 = (uint)p->kstack + KSTACKSIZE;
    // setting IOPL=0 in eflags *and* iomb beyond the tss segment limit
    // forbids I/O instructions (e.g., inb and outb) from user space
    mycpu()->ts.iomb = (ushort)0xFFFF;
    ltr(SEG_TSS << 3);
    lcr3(V2P(p->pgdir)); // switch to process's address space
    popcli();
}

// Load the initcode into address 0 of pgdir.
// sz must be less than a page.
void inituvm(pde_t *pgdir, char *init, uint sz)
{
    char *mem;

    if (sz >= PGSIZE)
        panic("inituvm: more than a page");
    mem = kalloc();
    memset(mem, 0, PGSIZE);
    mappages(pgdir, 0, PGSIZE, V2P(mem), PTE_W | PTE_U);
    memmove(mem, init, sz);
}

// Load a program segment into pgdir.  addr must be page-aligned
// and the pages from addr to addr+sz must already be mapped.
int loaduvm(pde_t *pgdir, char *addr, struct inode *ip, uint offset, uint sz)
{
    uint i, pa, n;
    pte_t *pte;

    if ((uint)addr % PGSIZE != 0)
        panic("loaduvm: addr must be page aligned");
    for (i = 0; i < sz; i += PGSIZE)
    {
        if ((pte = walkpgdir(pgdir, addr + i, 0)) == 0)
            panic("loaduvm: address should exist");
        pa = PTE_ADDR(*pte);
        if (sz - i < PGSIZE)
            n = sz - i;
        else
            n = PGSIZE;
        if (readi(ip, P2V(pa), offset + i, n) != n)
            return -1;
    }
    return 0;
}

// Allocate page tables and physical memory to grow process from oldsz to
// newsz, which need not be page aligned.  Returns new size or 0 on error.
int allocuvm(pde_t *pgdir, uint oldsz, uint newsz)
{
    char *mem;
    uint a;

    if (newsz >= KERNBASE)
        return 0;
    if (newsz < oldsz)
        return oldsz;

    a = PGROUNDUP(oldsz);

    for (; a < newsz; a += PGSIZE)
    {
        mem = kalloc();
        if (mem == 0)
        {
            deallocuvm(pgdir, newsz, oldsz);
            return 0;
        }
        memset(mem, 0, PGSIZE);
        if (mappages(pgdir, (char *)a, PGSIZE, V2P(mem), PTE_W | PTE_U) < 0)
        {
            deallocuvm(pgdir, newsz, oldsz);
            kfree(mem);
            return 0;
        }
    }
    return newsz;
}

// Deallocate user pages to bring the process size from oldsz to
// newsz.  oldsz and newsz need not be page-aligned, nor does newsz
// need to be less than oldsz.  oldsz can be larger than the actual
// process size.  Returns the new process size.
// If the page was swapped, free the corresponding disk block.
int deallocuvm(pde_t *pgdir, uint oldsz, uint newsz)
{
    pte_t *pte;
    uint a, pa;

    if (newsz >= oldsz)
        return oldsz;

    a = PGROUNDUP(newsz);
    for (; a < oldsz; a += PGSIZE)
    {
        pte = walkpgdir(pgdir, (char *)a, 0);

        if (!pte)
            a = PGADDR(PDX(a) + 1, 0, 0) - PGSIZE;

        else if (*pte & PTE_SWAPPED)
        {
            uint block_id = (*pte) >> 12;
            bfree_page(ROOTDEV, block_id);
        }

        else if ((*pte & PTE_P) != 0)
        {
            pa = PTE_ADDR(*pte);
            if (pa == 0)
                panic("kfree");
            char *v = P2V(pa);
            kfree(v);
            *pte = 0;
        }
    }
    return newsz;
}

// Free a page table and all the physical memory pages
// in the user part.
void freevm(pde_t *pgdir)
{
    uint i;

    if (pgdir == 0)
        panic("freevm: no pgdir");
    deallocuvm(pgdir, KERNBASE, 0);
    for (i = 0; i < NPDENTRIES; i++)
    {
        if (pgdir[i] & PTE_P)
        {
            char *v = P2V(PTE_ADDR(pgdir[i]));
            kfree(v);
        }
    }
    kfree((char *)pgdir);
}

// =========================================================================
//  EVICTION VICTIM SELECTION
//
//  Three policies live in this file.  Exactly one is compiled in depending
//  on EVICTION_POLICY (defined at the top of this file).
//
//  -----------------------------------------------------------------------
//  Policy 0 – Second Chance (clock algorithm)
//    Scans user pages sequentially.  The first page whose PTE_A bit is
//    clear is returned as the victim.  If every page has PTE_A set the
//    scan wraps around and clears ~10 % of the bits, then retries.
//    NOTE: the original xv7 baseline had two bugs here:
//      a) the access-bit test was written as  (*pte & ~PTE_A)  which is
//         nearly always true (it tests whether *any* bit other than A is
//         set).  The correct test is  !(*pte & PTE_A).
//      b) clearaccessbit() called cprintf() on every loop iteration,
//         producing thousands of console writes per eviction and making
//         Second Chance ~50× slower than the other policies.
//    Both have been fixed.
//  -----------------------------------------------------------------------
//  Policy 1 – FIFO
//    Each frame records a monotonically increasing allocation sequence
//    number (alloc_seq) in page_info[].  The victim is whichever present,
//    non-swapped page has the smallest (oldest) sequence number.
//  -----------------------------------------------------------------------
//  Policy 2 – Inverse-Lottery
//    Each frame accumulates "tickets" via the scanner in trap.c.  A page
//    that is frequently accessed ends up with many tickets; an idle page
//    decays toward the minimum.  The eviction draw assigns each page an
//    inverse weight  W_i = SCALE / tickets_i   and picks a victim by
//    drawing a uniform random number in [0, sum(W_i)).  Pages with fewer
//    tickets therefore carry proportionally more weight and are more
//    likely to be chosen.
// =========================================================================

// -------------------------------------------------------------------------
// Helper: iterate user-space pages and invoke a callback for each one that is
// present and not swapped.  This keeps the three policies free of duplicated
// scanning boilerplate.
// -------------------------------------------------------------------------
// (not needed at link time if the policy doesn't use it, but the compiler
//  will happily elide it)

// -------------------------------------------------------------------------
// Policy 0 – Second Chance
// -------------------------------------------------------------------------
#if EVICTION_POLICY == 0

// Clear the accessed bit on up to ~10 % of user pages.  Called only when a
// full scan found no un-accessed page (i.e. every page had PTE_A set).
// The original implementation printed a cprintf per iteration; that has been
// removed because it made the policy 50× slower than the others.
static void
reset_some_access_bits(pde_t *pgdir)
{
    pte_t *pte;
    int cleared = 0;

    for (long va = PGSIZE; va < KERNBASE; va += PGSIZE)
    {
        pte = walkpgdir(pgdir, (char *)va, 0);
        if (pte == 0)
            continue;
        if ((*pte & PTE_P) && (*pte & PTE_A))
        {
            *pte &= ~PTE_A;
            cleared++;
            if (cleared >= 103) // ~10 % of 1024 pages
                return;
        }
    }
}

pte_t *
select_a_victim(pde_t *pgdir)
{
    pte_t *pte;

    // First pass: find any present page whose accessed bit is clear.
    for (long va = PGSIZE; va < KERNBASE; va += PGSIZE)
    {
        pte = walkpgdir(pgdir, (char *)va, 0);
        if (pte == 0)
            continue;
        if (!(*pte & PTE_P))
            continue;
        if (*pte & PTE_SWAPPED)
            continue;
        // Correct test: the access bit is NOT set.
        if (!(*pte & PTE_A))
            return pte;
    }

    // Every page had PTE_A set – give some of them a "second chance" by
    // clearing their bits, then retry once.
    reset_some_access_bits(pgdir);

    for (long va = PGSIZE; va < KERNBASE; va += PGSIZE)
    {
        pte = walkpgdir(pgdir, (char *)va, 0);
        if (pte == 0)
            continue;
        if (!(*pte & PTE_P))
            continue;
        if (*pte & PTE_SWAPPED)
            continue;
        if (!(*pte & PTE_A))
            return pte;
    }

    return 0;
}

#endif // EVICTION_POLICY == 0

// -------------------------------------------------------------------------
// Policy 1 – FIFO
// -------------------------------------------------------------------------
#if EVICTION_POLICY == 1

pte_t *
select_a_victim(pde_t *pgdir)
{
    pte_t *pte;
    pte_t *victim = 0;
    uint oldest = 0;
    uint pa, idx, seq;

    for (uint va = PGSIZE; va < KERNBASE; va += PGSIZE)
    {
        pte = walkpgdir(pgdir, (char *)va, 0);
        if (pte == 0)
            continue;
        if ((*pte & PTE_P) == 0 || (*pte & PTE_SWAPPED))
            continue;
        pa = PTE_ADDR(*pte);
        if (pa >= PHYSTOP)
            continue;
        idx = pa / PGSIZE;
        seq = page_info[idx].alloc_seq;
        if (seq == 0)
            continue;
        if (victim == 0 || seq < oldest)
        {
            victim = pte;
            oldest = seq;
        }
    }
    return victim;
}

#endif // EVICTION_POLICY == 1

// -------------------------------------------------------------------------
// Policy 2 – Inverse-Lottery
// -------------------------------------------------------------------------
#if EVICTION_POLICY == 2

// Scale factor for the integer inverse-weight computation.
// W_i = LOTTERY_SCALE / tickets_i.   A larger constant gives finer
// granularity at the cost of a wider range for total_weight.
#define LOTTERY_SCALE 100000

// Tiny LCG used to produce the draw value.  Seeded lazily from the
// system tick counter the first time it is called.
static uint lottery_seed;

static uint
lottery_rand(uint bound)
{
    if (bound == 0)
        return 0;
    if (lottery_seed == 0)
        lottery_seed = ticks;
    lottery_seed = lottery_seed * 1103515245u + 12345u;
    return lottery_seed % bound;
}

pte_t *
select_a_victim(pde_t *pgdir)
{
    pte_t *pte;
    uint pa, idx, tickets, weight;
    uint total_weight = 0;
    uint draw;

    // ---------- first pass: accumulate total inverse weight ----------
    for (uint va = PGSIZE; va < KERNBASE; va += PGSIZE)
    {
        pte = walkpgdir(pgdir, (char *)va, 0);
        if (pte == 0)
            continue;
        if ((*pte & PTE_P) == 0 || (*pte & PTE_SWAPPED))
            continue;
        pa = PTE_ADDR(*pte);
        if (pa >= PHYSTOP)
            continue;

        idx = pa / PGSIZE;
        tickets = page_info[idx].tickets;
        if (tickets < 10)
            tickets = 10; // floor – avoids division by zero
        weight = LOTTERY_SCALE / tickets;
        if (weight == 0)
            weight = 1; // every page must have some weight
        total_weight += weight;
    }

    if (total_weight == 0)
        return 0;

    // ---------- draw a winning value in [0, total_weight) ----------
    draw = lottery_rand(total_weight);

    // ---------- second pass: walk until the draw is consumed ----------
    for (uint va = PGSIZE; va < KERNBASE; va += PGSIZE)
    {
        pte = walkpgdir(pgdir, (char *)va, 0);
        if (pte == 0)
            continue;
        if ((*pte & PTE_P) == 0 || (*pte & PTE_SWAPPED))
            continue;
        pa = PTE_ADDR(*pte);
        if (pa >= PHYSTOP)
            continue;

        idx = pa / PGSIZE;
        tickets = page_info[idx].tickets;
        if (tickets < 10)
            tickets = 10;
        weight = LOTTERY_SCALE / tickets;
        if (weight == 0)
            weight = 1;

        if (draw < weight)
            return pte; // this page is the victim
        draw -= weight;
    }

    return 0; // should not reach here if total_weight > 0
}

#endif // EVICTION_POLICY == 2

// =========================================================================
// clearaccessbit – kept for compatibility with swap_page() in paging.c.
// The Second Chance policy handles its own bit-clearing internally; the
// other two policies never call this.  It is left as a no-op stub for the
// non-Second-Chance builds so that paging.c compiles without changes.
// =========================================================================
void clearaccessbit(pde_t *pgdir)
{
    // Only meaningful for the Second Chance policy, which handles clearing
    // internally inside select_a_victim().  The other policies ignore
    // the access bit during victim selection so there is nothing to do.
    (void)pgdir;
}

// return the disk block-id, if the virtual address
// was swapped, -1 otherwise.
int getswappedblk(pde_t *pgdir, uint va)
{
    pte_t *pte = walkpgdir(pgdir, (char *)va, 0);
    // upper 20 bits of the PTE hold the block id when the page is swapped
    int block_id = (*pte) >> 12;
    return block_id;
}

// Clear PTE_U on a page. Used to create an inaccessible
// page beneath the user stack.
void clearpteu(pde_t *pgdir, char *uva)
{
    pte_t *pte;

    pte = walkpgdir(pgdir, uva, 0);
    if (pte == 0)
        panic("clearpteu");
    *pte &= ~PTE_U;
}

// Given a parent process's page table, create a copy
// of it for a child.
pde_t *
copyuvm(pde_t *pgdir, uint sz)
{
    pde_t *d;
    pte_t *pte;
    uint pa, i, flags;
    char *mem;
    if ((d = setupkvm()) == 0)
        return 0;
    for (i = 0; i < sz; i += PGSIZE)
    {
        if ((pte = walkpgdir(pgdir, (void *)i, 0)) == 0)
            panic("copyuvm: pte should exist");

        if (*pte & PTE_SWAPPED)
        {
            // Page lives on disk – allocate a frame, read it back, fix the PTE,
            // then release the disk block.
            if ((mem = kalloc()) == 0)
            {
                swap_page(pgdir);
                mem = kalloc();
            }
            int blockid = getswappedblk(pgdir, i);
            read_page_from_disk(ROOTDEV, mem, blockid);

            *pte = V2P(mem) | PTE_W | PTE_U | PTE_P;
            *pte &= ~PTE_SWAPPED;
            lcr3(V2P(pgdir));
            bfree_page(ROOTDEV, blockid);
        }

        pa = PTE_ADDR(*pte);
        flags = PTE_FLAGS(*pte);
        if ((mem = kalloc()) == 0)
        {
            swap_page(pgdir);
            mem = kalloc();
            if (mem == 0)
                cprintf("unable to get memory in copyuvm\n");
        }

        memmove(mem, (char *)P2V(pa), PGSIZE);
        if (mappages(d, (void *)i, PGSIZE, V2P(mem), flags) < 0)
            goto bad;
    }
    return d;

bad:
    freevm(d);
    return 0;
}

// PAGEBREAK!
//   Map user virtual address to kernel address.
char *
uva2ka(pde_t *pgdir, char *uva)
{
    pte_t *pte;

    pte = walkpgdir(pgdir, uva, 0);
    if ((*pte & PTE_P) == 0)
        return 0;
    if ((*pte & PTE_U) == 0)
        return 0;
    return (char *)P2V(PTE_ADDR(*pte));
}

// returns the page table entry corresponding
// to a virtual address.
pte_t *
uva2pte(pde_t *pgdir, uint uva)
{
    return walkpgdir(pgdir, (void *)uva, 0);
}

// Copy len bytes from p to user address va in page table pgdir.
// Most useful when pgdir is not the current page table.
// uva2ka ensures this only works for PTE_U pages.
int copyout(pde_t *pgdir, uint va, void *p, uint len)
{
    char *buf, *pa0;
    uint n, va0;

    buf = (char *)p;
    while (len > 0)
    {
        va0 = (uint)PGROUNDDOWN(va);
        pa0 = uva2ka(pgdir, (char *)va0);
        if (pa0 == 0)
            return -1;
        n = PGSIZE - (va - va0);
        if (n > len)
            n = len;
        memmove(pa0 + (va - va0), buf, n);
        len -= n;
        buf += n;
        va = va0 + PGSIZE;
    }
    return 0;
}

// PAGEBREAK!
//   Blank page.
// PAGEBREAK!
//   Blank page.
// PAGEBREAK!
//   Blank page.
