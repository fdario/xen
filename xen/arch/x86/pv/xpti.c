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

int xpti_domain_init(struct domain *d)
{
    bool xpti = false;
    int ret = 0;

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

    printk("Enabling Xen Pagetable protection (XPTI) for Domain %d\n",
           d->domain_id);

 done:
    return ret;
}
