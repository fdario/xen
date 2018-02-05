/******************************************************************************
 * arch/x86/mm/xpti.c
 *
 * Xen Page Table Isolation support.
 *
 * Copyright (c) 2018 SUSE Linux GmbH (Juergen Gross)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#include <xen/cpu.h>
#include <xen/domain_page.h>
#include <xen/errno.h>
#include <xen/init.h>
#include <xen/keyhandler.h>
#include <xen/lib.h>
#include <xen/notifier.h>
#include <xen/sched.h>
#include <asm/bitops.h>
#include <asm/pv/mm.h>

/*
 * For each L4 page table of the guest we need a shadow for the hypervisor.
 *
 * Such a shadow is considered to be active when the guest has loaded its
 * %cr3 on any vcpu with the MFN of the L4 page the shadow is associated
 * with.
 *
 * The shadows are referenced via an array of struct xpti_l4pg. This array
 * is set up at domain creation time and sized by number of vcpus of the
 * domain (N_L4_PERVCPU * max_vcpus). The index into this array is used to
 * address the single xpti_l4pg instances.
 *
 * A xpti_l4pg associated to a guest's L4 page table is put in a linked list
 * anchored in the xpti_l4ref hash array. The index into this array is taken
 * from the lower bits of the guest's L4 page table MFN.
 * The hash lists are sorted by a LRU mechanism. Additionally all xpti_l4pg's
 * in any hash list but those currently being active are in a single big
 * LRU list.
 *
 * Whenever a guest L4 page table is being unpinned its associated shadow is
 * put in a free list to avoid the need to allocate a new shadow from the heap
 * when a new L4 page is being pinned. This free list is limited in its length
 * to (N_L4_FREE_PERVCPU * max_vcpus).
 *
 * New shadows are obtained from the following resources (first hit wins):
 * - from the free list
 * - if maximum number of shadows not yet reached allocation from the heap
 * - from the end of the global lru list
 *
 * At domain creation the free list is initialized with N_L4_MIN_PERVCPU per
 * vcpu free shadows in order to have a minimal working set technically not
 * requiring additional allocations.
 */

#define XPTI_DEBUG

#define N_L4_PERVCPU      64
#define N_L4_FREE_PERVCPU 16
#define N_L4_MIN_PERVCPU   2

#define L4_INVALID  ~0

#define max_l4(d)    ((d)->max_vcpus * N_L4_PERVCPU)
#define max_free(xd) (N_L4_FREE_PERVCPU * (xd)->domain->max_vcpus)
#define min_free(xd) (N_L4_MIN_PERVCPU * (xd)->domain->max_vcpus)

#ifdef XPTI_DEBUG
#define XPTI_CNT(what) xd->what++
#else
#define XPTI_CNT(what)
#endif

struct xpti_l4ref {
    unsigned idx;              /* First shadow */
};

struct xpti_l4pg {
    unsigned long guest_mfn;   /* MFN of guest L4 page */
    unsigned long xen_mfn;     /* MFN of associated shadow */
    unsigned ref_next;         /* Next shadow, anchored in xpti_l4ref */
    unsigned lru_next;         /* Global LRU list */
    unsigned lru_prev;
    unsigned active_cnt;       /* Number of vcpus the shadow is active on */
};

struct xpti_domain {
    struct xpti_l4ref *l4ref;  /* Hash array */
    struct xpti_l4pg *l4pg;    /* Shadow admin array */
    unsigned l4ref_size;       /* Hash size */
    unsigned n_alloc;          /* Number of allocated shadows */
    unsigned n_free;           /* Number of free shadows */
    unsigned lru_first;        /* LRU list of associated shadows */
    unsigned lru_last;
    unsigned free_first;       /* List of free shadows */
    unsigned unused_first;     /* List of unused slots */
    spinlock_t lock;           /* Protects all shadow lists */
    struct domain *domain;
    struct page_info *l3_shadow;
    struct tasklet tasklet;
#ifdef XPTI_DEBUG
    unsigned cnt_alloc;
    unsigned cnt_free;
    unsigned cnt_getfree;
    unsigned cnt_putfree;
    unsigned cnt_getforce;
    unsigned cnt_activate;
    unsigned cnt_deactivate;
    unsigned cnt_newl4;
    unsigned cnt_freel4;
#endif
};

static __read_mostly enum {
    XPTI_DEFAULT,
    XPTI_ON,
    XPTI_OFF,
    XPTI_NODOM0
} opt_xpti = XPTI_DEFAULT;

static bool xpti_l3_shadow = false;
static l3_pgentry_t *xpti_l3_shadows[11];

static int parse_xpti(const char *s)
{
    int rc = 0;

    switch ( parse_bool(s, NULL) )
    {
    case 0:
        opt_xpti = XPTI_OFF;
        break;
    case 1:
        opt_xpti = XPTI_ON;
        break;
    default:
        if ( !strcmp(s, "default") )
            opt_xpti = XPTI_DEFAULT;
        else if ( !strcmp(s, "nodom0") )
            opt_xpti = XPTI_NODOM0;
        else
            rc = -EINVAL;
        break;
    }

    return rc;
}

custom_runtime_param("xpti", parse_xpti);

static unsigned xpti_shadow_add(struct xpti_domain *xd, unsigned long mfn)
{
    unsigned new = xd->unused_first;
    struct xpti_l4pg *l4pg = xd->l4pg + new;

    if ( xd->n_alloc >= max_l4(xd->domain) )
        new = L4_INVALID;
    if ( new != L4_INVALID )
    {
        XPTI_CNT(cnt_alloc);
        xd->unused_first = l4pg->lru_next;
        l4pg->xen_mfn = mfn;
        xd->n_alloc++;
    }

    return new;
}

static void *xpti_shadow_free(struct xpti_domain *xd, unsigned free)
{
    struct xpti_l4pg *l4pg = xd->l4pg + free;
    void *virt;

    XPTI_CNT(cnt_free);
    ASSERT(xd->n_alloc);
    virt = mfn_to_virt(l4pg->xen_mfn);
    l4pg->lru_next = xd->unused_first;
    xd->unused_first = free;
    xd->n_alloc--;

    return virt;
}

static unsigned xpti_shadow_getfree(struct xpti_domain *xd)
{
    unsigned free = xd->free_first;
    struct xpti_l4pg *l4pg = xd->l4pg + free;

    if ( free != L4_INVALID )
    {
        XPTI_CNT(cnt_getfree);
        xd->free_first = l4pg->lru_next;
        ASSERT(xd->n_free);
        xd->n_free--;
        l4pg->lru_next = L4_INVALID;

        if ( !xd->n_free && xd->n_alloc < max_l4(xd->domain) &&
             !xd->domain->is_dying )
            tasklet_schedule(&xd->tasklet);
    }

    return free;
}

static void xpti_shadow_putfree(struct xpti_domain *xd, unsigned free)
{
    struct xpti_l4pg *l4pg = xd->l4pg + free;

    ASSERT(free != L4_INVALID);
    XPTI_CNT(cnt_putfree);
    l4pg->lru_prev = L4_INVALID;
    l4pg->lru_next = xd->free_first;
    xd->free_first = free;
    xd->n_free++;

    if ( xd->n_free > max_free(xd) && !xd->domain->is_dying )
        tasklet_schedule(&xd->tasklet);
}

static struct xpti_l4ref *xpti_get_hashentry_mfn(struct xpti_domain *xd,
                                                 unsigned long mfn)
{
    return xd->l4ref + (mfn & (xd->l4ref_size - 1));
}

static struct xpti_l4ref *xpti_get_hashentry(struct xpti_domain *xd,
                                             unsigned idx)
{
    struct xpti_l4pg *l4pg = xd->l4pg + idx;

    return xpti_get_hashentry_mfn(xd, l4pg->guest_mfn);
}

static unsigned xpti_shadow_from_hashlist(struct xpti_domain *xd,
                                          unsigned long mfn)
{
    struct xpti_l4ref *l4ref;
    unsigned ref_idx;

    l4ref = xpti_get_hashentry_mfn(xd, mfn);
    ref_idx = l4ref->idx;
    while ( ref_idx != L4_INVALID && xd->l4pg[ref_idx].guest_mfn != mfn )
        ref_idx = xd->l4pg[ref_idx].ref_next;

    return ref_idx;
}

static void xpti_shadow_deactivate(struct xpti_domain *xd, unsigned idx)
{
    struct xpti_l4pg *l4pg = xd->l4pg + idx;
    struct xpti_l4ref *l4ref;
    unsigned ref_idx;

    /* Decrement active count. If still > 0 we are done. */
    XPTI_CNT(cnt_deactivate);
    ASSERT(l4pg->active_cnt > 0);
    l4pg->active_cnt--;
    if ( l4pg->active_cnt )
        return;

    /* Put in hash list at first position for its hash entry. */
    l4ref = xpti_get_hashentry(xd, idx);
    ref_idx = l4ref->idx;
    ASSERT(ref_idx != L4_INVALID);
    /* Only need to do something if not already in front. */
    if ( ref_idx != idx )
    {
        /* Search for entry referencing our element. */
        while ( xd->l4pg[ref_idx].ref_next != idx )
              ref_idx = xd->l4pg[ref_idx].ref_next;

        /* Dequeue and put to front of list. */
        xd->l4pg[ref_idx].ref_next = l4pg->ref_next;
        l4pg->ref_next = l4ref->idx;
        l4ref->idx = idx;
    }

    /* Put into LRU list at first position. */
    l4pg->lru_next = xd->lru_first;
    l4pg->lru_prev = L4_INVALID;
    xd->lru_first = idx;
    if ( xd->lru_last == L4_INVALID )
        xd->lru_last = idx;
    else if ( l4pg->lru_next != L4_INVALID )
        xd->l4pg[l4pg->lru_next].lru_prev = idx;
}

static void xpti_shadow_lru_remove(struct xpti_domain *xd, unsigned idx)
{
    struct xpti_l4pg *l4pg = xd->l4pg + idx;
    unsigned prev = l4pg->lru_prev;
    unsigned next = l4pg->lru_next;

    if ( prev != L4_INVALID )
        xd->l4pg[prev].lru_next = next;
    else if ( xd->lru_first == idx )
        xd->lru_first = next;
    if ( next != L4_INVALID )
        xd->l4pg[next].lru_prev = prev;
    else if ( xd->lru_last == idx )
        xd->lru_last = prev;
    l4pg->lru_prev = L4_INVALID;
    l4pg->lru_next = L4_INVALID;
}

static void xpti_shadow_hash_remove(struct xpti_domain *xd, unsigned idx)
{
    struct xpti_l4pg *l4pg = xd->l4pg + idx;
    struct xpti_l4ref *l4ref;
    unsigned ref_idx;

    l4ref = xpti_get_hashentry(xd, idx);
    ref_idx = l4ref->idx;
    ASSERT(ref_idx != L4_INVALID);
    if ( ref_idx == idx )
    {
        l4ref->idx = l4pg->ref_next;
    }
    else
    {
        while ( xd->l4pg[ref_idx].ref_next != idx )
            ref_idx = xd->l4pg[ref_idx].ref_next;
        xd->l4pg[ref_idx].ref_next = l4pg->ref_next;
    }
}

static unsigned xpti_shadow_getforce(struct xpti_domain *xd)
{
    unsigned idx = xd->lru_last;

    XPTI_CNT(cnt_getforce);
    ASSERT(idx != L4_INVALID);
    ASSERT(!xd->l4pg[idx].active_cnt);

    xpti_shadow_hash_remove(xd, idx);
    xpti_shadow_lru_remove(xd, idx);

    return idx;
}

static void xpti_update_l4_entry(struct xpti_domain *xd, l4_pgentry_t *dest,
                                 l4_pgentry_t entry, unsigned slot)
{
    l3_pgentry_t *l3pg;

    switch ( slot )
    {
    case 257: /* ioremap area. */
    case 258: /* linear page table (guest table). */
    case 259: /* linear page table (shadow table). */
        dest[slot] = l4e_empty();
        break;
    case 260: /* per-domain mappings. */
        dest[slot] = l4e_from_page(xd->l3_shadow, __PAGE_HYPERVISOR);
        break;
    case 261 ... 271: /* hypervisor text and data, direct phys mapping. */
        l3pg = xpti_l3_shadows[slot - 261];
        dest[slot] = l3pg
                     ? l4e_from_mfn(_mfn(virt_to_mfn(l3pg)), __PAGE_HYPERVISOR)
                     : l4e_empty();
        break;
    case 256: /* read-only guest accessible m2p table. */
    default:
        dest[slot] = entry;
        break;
    }
}

static void xpti_init_xen_l4(struct xpti_domain *xd, struct xpti_l4pg *l4pg)
{
    unsigned i;
    l4_pgentry_t *src, *dest;

    src = map_domain_page(_mfn(l4pg->guest_mfn));
    dest = mfn_to_virt(l4pg->xen_mfn);
    for ( i = 0; i < L4_PAGETABLE_ENTRIES; i++ )
        xpti_update_l4_entry(xd, dest, src[i], i);
    unmap_domain_page(src);
}

static unsigned xpti_shadow_get(struct xpti_domain *xd, unsigned long mfn)
{
    unsigned idx;
    struct xpti_l4ref *l4ref;
    struct xpti_l4pg *l4pg;

    idx = xpti_shadow_from_hashlist(xd, mfn);
    if ( idx != L4_INVALID )
    {
        /* Remove from LRU list if currently not active. */
        if ( !xd->l4pg[idx].active_cnt )
            xpti_shadow_lru_remove(xd, idx);

        return idx;
    }

    XPTI_CNT(cnt_newl4);
    idx = xpti_shadow_getfree(xd);
    if ( idx == L4_INVALID )
        idx = xpti_shadow_getforce(xd);

    /* Set mfn and insert in hash list. */
    l4ref = xpti_get_hashentry_mfn(xd, mfn);
    l4pg = xd->l4pg + idx;
    l4pg->guest_mfn = mfn;
    l4pg->ref_next = l4ref->idx;
    l4ref->idx = idx;

    /* Fill the shadow page table entries. */
    xpti_init_xen_l4(xd, l4pg);

    return idx;
}

static unsigned xpti_shadow_activate(struct xpti_domain *xd, unsigned long mfn)
{
    unsigned idx;
    struct xpti_l4pg *l4pg;

    XPTI_CNT(cnt_activate);
    idx = xpti_shadow_get(xd, mfn);
    l4pg = xd->l4pg + idx;

    l4pg->active_cnt++;

    return idx;
}

void xpti_update_l4(const struct domain *d, unsigned long mfn, unsigned slot,
                    l4_pgentry_t e)
{
    struct xpti_domain *xd = d->arch.pv_domain.xpti;
    unsigned long flags;
    unsigned idx;
    l4_pgentry_t *l4;

    spin_lock_irqsave(&xd->lock, flags);

    idx = xpti_shadow_from_hashlist(xd, mfn);
    if ( idx != L4_INVALID )
    {
        l4 = mfn_to_virt(xd->l4pg[idx].xen_mfn);
        xpti_update_l4_entry(xd, l4, e, slot);
    }

    spin_unlock_irqrestore(&xd->lock, flags);
}

void xpti_make_cr3(struct vcpu *v, unsigned long mfn)
{
    struct xpti_domain *xd = v->domain->arch.pv_domain.xpti;
    struct cpu_info *cpu_info;
    unsigned long flags;
    unsigned old, new;

    cpu_info = (struct cpu_info *)v->arch.pv_vcpu.stack_regs;

    spin_lock_irqsave(&xd->lock, flags);

    old = v->arch.pv_vcpu.xen_cr3_shadow;

    /* First activate new shadow. */
    new = xpti_shadow_activate(xd, mfn);
    v->arch.pv_vcpu.xen_cr3_shadow = new;

    /* Deactivate old shadow if applicable. */
    if ( old != L4_INVALID )
        xpti_shadow_deactivate(xd, old);

    cpu_info->xen_cr3 = mfn << PAGE_SHIFT;
    cpu_info->guest_cr3 = xd->l4pg[new].xen_mfn << PAGE_SHIFT;

    spin_unlock_irqrestore(&xd->lock, flags);
}

void xpti_free_l4(struct domain *d, unsigned long mfn)
{
    struct xpti_domain *xd = d->arch.pv_domain.xpti;
    unsigned long flags;
    unsigned idx;

    spin_lock_irqsave(&xd->lock, flags);

    idx = xpti_shadow_from_hashlist(xd, mfn);
    if ( idx != L4_INVALID )
    {
        XPTI_CNT(cnt_freel4);
        /* Might still be active in a vcpu to be destroyed. */
        if ( !xd->l4pg[idx].active_cnt )
        {
            xpti_shadow_lru_remove(xd, idx);
            xpti_shadow_hash_remove(xd, idx);
            xpti_shadow_putfree(xd, idx);
        }
    }

    spin_unlock_irqrestore(&xd->lock, flags);
}

static void xpti_tasklet(unsigned long _xd)
{
    struct xpti_domain *xd = (struct xpti_domain *)_xd;
    void *virt;
    unsigned long flags;
    unsigned free;

    spin_lock_irqsave(&xd->lock, flags);

    while ( xd->n_free < min_free(xd) && xd->n_alloc < max_l4(xd->domain) )
    {
        spin_unlock_irqrestore(&xd->lock, flags);
        virt = alloc_xenheap_pages(0, MEMF_node(domain_to_node(xd->domain)));
        spin_lock_irqsave(&xd->lock, flags);
        if ( !virt )
            break;
        free = xpti_shadow_add(xd, virt_to_mfn(virt));
        if ( free == L4_INVALID )
        {
            spin_unlock_irqrestore(&xd->lock, flags);
            free_xenheap_page(virt);
            spin_lock_irqsave(&xd->lock, flags);
            break;
        }
        xpti_shadow_putfree(xd, free);
    }

    while ( xd->n_free > max_free(xd) )
    {
        free = xpti_shadow_getfree(xd);
        ASSERT(free != L4_INVALID);
        virt = xpti_shadow_free(xd, free);
        spin_unlock_irqrestore(&xd->lock, flags);
        free_xenheap_page(virt);
        spin_lock_irqsave(&xd->lock, flags);
    }

    spin_unlock_irqrestore(&xd->lock, flags);
}

void xpti_domain_destroy(struct domain *d)
{
    struct xpti_domain *xd = d->arch.pv_domain.xpti;
    unsigned idx;

    if ( !xd )
        return;

    tasklet_kill(&xd->tasklet);

    while ( xd->lru_first != L4_INVALID ) {
        idx = xd->lru_first;
        xpti_shadow_lru_remove(xd, idx);
        free_xenheap_page(xpti_shadow_free(xd, idx));
    }

    while ( xd->n_free ) {
        idx = xpti_shadow_getfree(xd);
        free_xenheap_page(xpti_shadow_free(xd, idx));
    }

    if ( xd->l3_shadow )
        free_domheap_page(xd->l3_shadow);
    xfree(xd->l4pg);
    xfree(xd->l4ref);
    xfree(xd);
    d->arch.pv_domain.xpti = NULL;
}

void xpti_vcpu_destroy(struct vcpu *v)
{
    struct xpti_domain *xd = v->domain->arch.pv_domain.xpti;
    unsigned long flags;

    if ( xd )
    {
        spin_lock_irqsave(&xd->lock, flags);

        if ( v->arch.pv_vcpu.xen_cr3_shadow != L4_INVALID )
        {
            xpti_shadow_deactivate(xd, v->arch.pv_vcpu.xen_cr3_shadow);
            v->arch.pv_vcpu.xen_cr3_shadow = L4_INVALID;
        }

        spin_unlock_irqrestore(&xd->lock, flags);

        free_xenheap_page(v->arch.pv_vcpu.stack_regs);
        v->arch.pv_vcpu.stack_regs = NULL;
        destroy_perdomain_mapping(v->domain, XPTI_START(v), STACK_PAGES);
    }
}

static int xpti_vcpu_init(struct vcpu *v)
{
    struct domain *d = v->domain;
    struct page_info *pg;
    void *ptr;
    struct cpu_info *info;
    struct desc_struct *gdt;
    unsigned long stack_bottom;
    int rc;

    /* Populate page tables. */
    rc = create_perdomain_mapping(d, XPTI_START(v), STACK_PAGES,
                                  NIL(l1_pgentry_t *), NULL);
    if ( rc )
        goto done;

    /* Map stacks. */
    rc = create_perdomain_mapping(d, XPTI_START(v), IST_MAX,
                                  NULL, NIL(struct page_info *));
    if ( rc )
        goto done;

    ptr = alloc_xenheap_page();
    if ( !ptr )
    {
        rc = -ENOMEM;
        goto done;
    }
    clear_page(ptr);
    rc = addmfn_to_perdomain_mapping(d, XPTI_START(v) + STACK_SIZE - PAGE_SIZE,
                                     _mfn(virt_to_mfn(ptr)));
    if ( rc )
        goto done;
    info = (struct cpu_info *)((unsigned long)ptr + PAGE_SIZE) - 1;
    info->flags = ON_VCPUSTACK;
    v->arch.pv_vcpu.stack_regs = &info->guest_cpu_user_regs;

    /* Map TSS. */
    rc = create_perdomain_mapping(d, XPTI_TSS(v), 1, NULL, &pg);
    if ( rc )
        goto done;
    info = (struct cpu_info *)(XPTI_START(v) + STACK_SIZE) - 1;
    stack_bottom = (unsigned long)&info->guest_cpu_user_regs.es;
    ptr = __map_domain_page(pg);
    tss_init(ptr, stack_bottom);
    unmap_domain_page(ptr);

    /* Map stub trampolines. */
    rc = create_perdomain_mapping(d, XPTI_TRAMPOLINE(v), 1, NULL, &pg);
    if ( rc )
        goto done;
    ptr = __map_domain_page(pg);
    write_stub_trampoline((unsigned char *)ptr, XPTI_TRAMPOLINE(v),
                          stack_bottom, (unsigned long)lstar_enter);
    write_stub_trampoline((unsigned char *)ptr + STUB_TRAMPOLINE_SIZE_PERVCPU,
                          XPTI_TRAMPOLINE(v) + STUB_TRAMPOLINE_SIZE_PERVCPU,
                          stack_bottom, (unsigned long)cstar_enter);
    unmap_domain_page(ptr);
    rc = modflags_perdomain_mapping(d, XPTI_TRAMPOLINE(v), 0,
                                    _PAGE_NX | _PAGE_RW | _PAGE_DIRTY);

    /* Map GDT. */
    BUILD_BUG_ON(NR_RESERVED_GDT_PAGES > 1);
    gdt = (struct desc_struct *)GDT_VIRT_START(v) + FIRST_RESERVED_GDT_ENTRY;
    rc = create_perdomain_mapping(v->domain, (unsigned long)gdt,
                                  NR_RESERVED_GDT_PAGES, NULL, &pg);
    if ( !rc )
    {
        gdt = __map_domain_page(pg);
        memcpy(gdt, boot_cpu_gdt_table, NR_RESERVED_GDT_BYTES);
        _set_tssldt_desc(gdt + TSS_ENTRY - FIRST_RESERVED_GDT_ENTRY,
                     XPTI_TSS(v),
                     offsetof(struct tss_struct, __cacheline_filler) - 1,
                     SYS_DESC_tss_avail);
        unmap_domain_page(gdt);
    }

    v->arch.pv_vcpu.xen_cr3_shadow = L4_INVALID;

 done:
    return rc;
}

static int xpti_add_mapping(unsigned long addr)
{
    unsigned int slot, flags, mapflags;
    unsigned long mfn;
    l3_pgentry_t *pl3e;
    l2_pgentry_t *pl2e;
    l1_pgentry_t *pl1e;

    slot = l4_table_offset(addr);
    pl3e = l4e_to_l3e(idle_pg_table[slot]);

    slot = l3_table_offset(addr);
    mapflags = l3e_get_flags(pl3e[slot]);
    ASSERT(mapflags & _PAGE_PRESENT);
    if ( mapflags & _PAGE_PSE )
    {
        mapflags &= ~_PAGE_PSE;
        mfn = l3e_get_pfn(pl3e[slot]) & ~((1UL << (2 * PAGETABLE_ORDER)) - 1);
        mfn |= PFN_DOWN(addr) & ((1UL << (2 * PAGETABLE_ORDER)) - 1);
    }
    else
    {
        pl2e = l3e_to_l2e(pl3e[slot]);
        slot = l2_table_offset(addr);
        mapflags = l2e_get_flags(pl2e[slot]);
        ASSERT(mapflags & _PAGE_PRESENT);
        if ( mapflags & _PAGE_PSE )
        {
            mapflags &= ~_PAGE_PSE;
            mfn = l2e_get_pfn(pl2e[slot]) & ~((1UL << PAGETABLE_ORDER) - 1);
            mfn |= PFN_DOWN(addr) & ((1UL << PAGETABLE_ORDER) - 1);
        }
        else
        {
            pl1e = l2e_to_l1e(pl2e[slot]);
            slot = l1_table_offset(addr);
            mapflags = l1e_get_flags(pl1e[slot]);
            ASSERT(mapflags & _PAGE_PRESENT);
            mfn = l1e_get_pfn(pl1e[slot]);
        }
    }

    slot = l4_table_offset(addr);
    ASSERT(slot >= 261 && slot <= 271);
    pl3e = xpti_l3_shadows[slot - 261];
    if ( !pl3e )
    {
        pl3e = alloc_xen_pagetable();
        if ( !pl3e )
            return -ENOMEM;
        clear_page(pl3e);
        xpti_l3_shadows[slot - 261] = pl3e;
    }

    slot = l3_table_offset(addr);
    flags = l3e_get_flags(pl3e[slot]);
    if ( !(flags & _PAGE_PRESENT) )
    {
        pl2e = alloc_xen_pagetable();
        if ( !pl2e )
            return -ENOMEM;
        clear_page(pl2e);
        pl3e[slot] = l3e_from_mfn(_mfn(virt_to_mfn(pl2e)), __PAGE_HYPERVISOR);
    }
    else
    {
        pl2e = l3e_to_l2e(pl3e[slot]);
    }

    slot = l2_table_offset(addr);
    flags = l2e_get_flags(pl2e[slot]);
    if ( !(flags & _PAGE_PRESENT) )
    {
        pl1e = alloc_xen_pagetable();
        if ( !pl1e )
            return -ENOMEM;
        clear_page(pl1e);
        pl2e[slot] = l2e_from_mfn(_mfn(virt_to_mfn(pl1e)), __PAGE_HYPERVISOR);
    }
    else
    {
        pl1e = l2e_to_l1e(pl2e[slot]);
    }

    slot = l1_table_offset(addr);
    pl1e[slot] = l1e_from_mfn(_mfn(mfn), mapflags);

    return 0;
}

static void xpti_rm_mapping(unsigned long addr)
{
    unsigned int slot, flags;
    l3_pgentry_t *pl3e;
    l2_pgentry_t *pl2e;
    l1_pgentry_t *pl1e;

    slot = l4_table_offset(addr);
    ASSERT(slot >= 261 && slot <= 271);
    pl3e = xpti_l3_shadows[slot - 261];
    if ( !pl3e )
        return;

    slot = l3_table_offset(addr);
    flags = l3e_get_flags(pl3e[slot]);
    if ( !(flags & _PAGE_PRESENT) )
        return;

    pl2e = l3e_to_l2e(pl3e[slot]);
    slot = l2_table_offset(addr);
    flags = l2e_get_flags(pl2e[slot]);
    if ( !(flags & _PAGE_PRESENT) )
        return;

    pl1e = l2e_to_l1e(pl2e[slot]);
    slot = l1_table_offset(addr);
    pl1e[slot] = l1e_empty();
}

int xpti_domain_init(struct domain *d)
{
    bool xpti = false;
    int ret = -ENOMEM;
    struct vcpu *v;
    struct xpti_domain *xd;
    void *virt;
    unsigned long addr;
    unsigned i, new;
    l3_pgentry_t *l3tab, *l3shadow;

    if ( !is_pv_domain(d) || is_pv_32bit_domain(d) )
        return 0;

    switch ( opt_xpti )
    {
    case XPTI_OFF:
        xpti = false;
        break;
    case XPTI_ON:
        xpti = true;
        break;
    case XPTI_NODOM0:
        xpti = boot_cpu_data.x86_vendor != X86_VENDOR_AMD &&
               d->domain_id != 0 && d->domain_id != hardware_domid;
        break;
    case XPTI_DEFAULT:
        xpti = boot_cpu_data.x86_vendor != X86_VENDOR_AMD;
        break;
    }

    if ( !xpti )
        return 0;

    xd = xzalloc(struct xpti_domain);
    if ( !xd )
        goto done;
    d->arch.pv_domain.xpti = xd;
    xd->domain = d;
    xd->lru_first = L4_INVALID;
    xd->lru_last = L4_INVALID;
    xd->free_first = L4_INVALID;

    if ( !xpti_l3_shadow )
    {
        xpti_l3_shadow = true;

        for_each_online_cpu ( i )
            if ( xpti_add_mapping((unsigned long)idt_tables[i]) )
                goto done;

        for ( addr = round_pgdown((unsigned long)&xpti_map_start);
              addr <= round_pgdown((unsigned long)&xpti_map_end - 1);
              addr += PAGE_SIZE )
            if ( xpti_add_mapping(addr) )
                goto done;

        for ( addr = round_pgdown((unsigned long)&xpti_map_start_compat);
              addr <= round_pgdown((unsigned long)&xpti_map_end_compat - 1);
              addr += PAGE_SIZE )
            if ( xpti_add_mapping(addr) )
                goto done;
    }

    spin_lock_init(&xd->lock);
    tasklet_init(&xd->tasklet, xpti_tasklet, (unsigned long)xd);

    xd->l4ref_size = 1 << (fls(max_l4(d)) - 1);
    xd->l4ref = xzalloc_array(struct xpti_l4ref, xd->l4ref_size);
    if ( !xd->l4ref )
        goto done;
    for ( i = 0; i < xd->l4ref_size; i++ )
        xd->l4ref[i].idx = L4_INVALID;

    xd->l4pg = xzalloc_array(struct xpti_l4pg, max_l4(d));
    if ( !xd->l4pg )
        goto done;
    for ( i = 0; i < max_l4(d) - 1; i++ )
    {
        xd->l4pg[i].lru_next = i + 1;
    }
    xd->l4pg[i].lru_next = L4_INVALID;
    xd->unused_first = 0;

    for ( i = 0; i < min_free(xd); i++ )
    {
        virt = alloc_xenheap_pages(0, MEMF_node(domain_to_node(d)));
        if ( !virt )
            goto done;
        new = xpti_shadow_add(xd, virt_to_mfn(virt));
        ASSERT(new != L4_INVALID);
        xpti_shadow_putfree(xd, new);
    }

    for_each_vcpu( d, v )
    {
        ret = xpti_vcpu_init(v);
        if ( ret )
            goto done;
    }

    xd->l3_shadow = alloc_domheap_page(d, MEMF_no_owner);
    if ( !xd->l3_shadow )
        goto done;
    l3tab = __map_domain_page(d->arch.perdomain_l3_pg);
    l3shadow = __map_domain_page(xd->l3_shadow);
    clear_page(l3shadow);
    l3shadow[0] = l3tab[0];          /* GDT/LDT shadow mapping. */
    l3shadow[3] = l3tab[3];          /* XPTI mappings. */
    unmap_domain_page(l3shadow);
    unmap_domain_page(l3tab);
    ret = 0;

    printk("Enabling Xen Pagetable protection (XPTI) for Domain %d\n",
           d->domain_id);

 done:
    return ret;
}

static void xpti_dump_domain_info(struct domain *d)
{
    struct xpti_domain *xd = d->arch.pv_domain.xpti;
    unsigned long flags;

    if ( !is_pv_domain(d) || !xd )
        return;

    spin_lock_irqsave(&xd->lock, flags);

    printk("Domain %d XPTI shadow pages: %u allocated, %u max, %u free\n",
           d->domain_id, xd->n_alloc, max_l4(d), xd->n_free);

#ifdef XPTI_DEBUG
    printk("  alloc: %d, free: %d, getfree: %d, putfree: %d, getforce: %d\n",
           xd->cnt_alloc, xd->cnt_free, xd->cnt_getfree, xd->cnt_putfree,
           xd->cnt_getforce);
    printk("  activate: %d, deactivate: %d, newl4: %d, freel4: %d\n",
           xd->cnt_activate, xd->cnt_deactivate, xd->cnt_newl4,
           xd->cnt_freel4);
#endif

    spin_unlock_irqrestore(&xd->lock, flags);
}

static void xpti_dump_info(unsigned char key)
{
    struct domain *d;
    char *opt;

    printk("'%c' pressed -> dumping XPTI info\n", key);

    switch ( opt_xpti )
    {
    case XPTI_DEFAULT:
        opt = "default";
        break;
    case XPTI_ON:
        opt = "on";
        break;
    case XPTI_OFF:
        opt = "off";
        break;
    case XPTI_NODOM0:
        opt = "nodom0";
        break;
    default:
        opt = "???";
        break;
    }

    printk("XPTI global setting: %s\n", opt);

    rcu_read_lock(&domlist_read_lock);

    for_each_domain ( d )
        xpti_dump_domain_info(d);

    rcu_read_unlock(&domlist_read_lock);
}

static int __init xpti_key_init(void)
{
    register_keyhandler('X', xpti_dump_info, "dump XPTI info", 1);
    return 0;
}
__initcall(xpti_key_init);

static int xpti_cpu_callback(struct notifier_block *nfb, unsigned long action,
                             void *hcpu)
{
    unsigned int cpu = (unsigned long)hcpu;
    int rc = 0;

    if ( !xpti_l3_shadow )
        return NOTIFY_DONE;

    switch ( action )
    {
    case CPU_DOWN_FAILED:
    case CPU_ONLINE:
        rc = xpti_add_mapping((unsigned long)idt_tables[cpu]);
        break;
    case CPU_DOWN_PREPARE:
        xpti_rm_mapping((unsigned long)idt_tables[cpu]);
        break;
    default:
        break;
    }

    return !rc ? NOTIFY_DONE : notifier_from_errno(rc);
}

static struct notifier_block xpti_cpu_nfb = {
    .notifier_call = xpti_cpu_callback
};

static int __init xpti_presmp_init(void)
{
    register_cpu_notifier(&xpti_cpu_nfb);
    return 0;
}
presmp_initcall(xpti_presmp_init);
