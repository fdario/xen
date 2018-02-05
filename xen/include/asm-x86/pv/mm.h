/*
 * asm-x86/pv/mm.h
 *
 * Memory management interfaces for PV guests
 *
 * Copyright (C) 2017 Wei Liu <wei.liu2@citrix.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms and conditions of the GNU General Public
 * License, version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __X86_PV_MM_H__
#define __X86_PV_MM_H__

#ifdef CONFIG_PV

extern void *xpti_map_start;
extern void *xpti_map_end;
extern void *xpti_map_start_compat;
extern void *xpti_map_end_compat;

int pv_ro_page_fault(unsigned long addr, struct cpu_user_regs *regs);

long pv_set_gdt(struct vcpu *v, unsigned long *frames, unsigned int entries);
void pv_destroy_gdt(struct vcpu *v);

bool pv_map_ldt_shadow_page(unsigned int off);
bool pv_destroy_ldt(struct vcpu *v);

void xpti_vcpu_destroy(struct vcpu *v);
int xpti_domain_init(struct domain *d);
void xpti_domain_destroy(struct domain *d);
void xpti_make_cr3(struct vcpu *v, unsigned long mfn);
void xpti_free_l4(struct domain *d, unsigned long mfn);
void xpti_update_l4(const struct domain *d, unsigned long mfn, unsigned slot,
                    l4_pgentry_t e);

static inline bool is_domain_xpti_active(const struct domain *d)
{
    return is_pv_domain(d) && d->arch.pv_domain.xpti;
}

static inline bool is_vcpu_xpti_active(const struct vcpu *v)
{
    return is_domain_xpti_active(v->domain);
}

#else

#include <xen/errno.h>
#include <xen/lib.h>

static inline int pv_ro_page_fault(unsigned long addr,
                                   struct cpu_user_regs *regs)
{
    ASSERT_UNREACHABLE();
    return 0;
}

static inline long pv_set_gdt(struct vcpu *v, unsigned long *frames,
                              unsigned int entries)
{ ASSERT_UNREACHABLE(); return -EINVAL; }
static inline void pv_destroy_gdt(struct vcpu *v) { ASSERT_UNREACHABLE(); }

static inline bool pv_map_ldt_shadow_page(unsigned int off) { return false; }
static inline bool pv_destroy_ldt(struct vcpu *v)
{ ASSERT_UNREACHABLE(); return false; }

static inline void xpti_vcpu_init(struct vcpu *v) { }
static inline int xpti_domain_init(struct domain *d) { return 0; }
static inline void xpti_domain_destroy(struct domain *d) { }
static inline void xpti_make_cr3(struct vcpu *v, unsigned long mfn) { }
static inline void xpti_free_l4(struct domain *d, unsigned long mfn) { }
static inline void xpti_update_l4(const struct domain *d, unsigned long mfn,
                                  unsigned slot, l4_pgentry_t e) { }

static inline bool is_domain_xpti_active(const struct domain *d)
{ return false; }
static inline bool is_vcpu_xpti_active(const struct vcpu *v) { return false; }

#endif

#endif /* __X86_PV_MM_H__ */
