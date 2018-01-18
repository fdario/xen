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

#include <xen/domain_page.h>
#include <xen/errno.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>

struct xpti_domain {
    int pad;
};

static __read_mostly enum {
    XPTI_DEFAULT,
    XPTI_ON,
    XPTI_OFF,
    XPTI_NODOM0
} opt_xpti = XPTI_DEFAULT;

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

void xpti_domain_destroy(struct domain *d)
{
    xfree(d->arch.pv_domain.xpti);
    d->arch.pv_domain.xpti = NULL;
}

void xpti_vcpu_destroy(struct vcpu *v)
{
    if ( v->domain->arch.pv_domain.xpti )
    {
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

 done:
    return rc;
}

int xpti_domain_init(struct domain *d)
{
    bool xpti = false;
    int ret = 0;
    struct vcpu *v;

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

    d->arch.pv_domain.xpti = xmalloc(struct xpti_domain);
    if ( !d->arch.pv_domain.xpti )
    {
        ret = -ENOMEM;
        goto done;
    }

    for_each_vcpu( d, v )
    {
        ret = xpti_vcpu_init(v);
        if ( ret )
            goto done;
    }

    printk("Enabling Xen Pagetable protection (XPTI) for Domain %d\n",
           d->domain_id);

 done:
    return ret;
}
