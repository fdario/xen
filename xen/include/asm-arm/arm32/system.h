/* Portions taken from Linux arch arm */
#ifndef __ASM_ARM32_SYSTEM_H
#define __ASM_ARM32_SYSTEM_H

#include <asm/arm32/cmpxchg.h>

#include <xen/trace.h>

#define local_irq_disable() asm volatile ( "cpsid i @ local_irq_disable\n" : : : "cc" )
#define local_irq_enable()  asm volatile ( "cpsie i @ local_irq_enable\n" : : : "cc" )

#define local_save_flags(x)                                      \
({                                                               \
    BUILD_BUG_ON(sizeof(x) != sizeof(long));                     \
    asm volatile ( "mrs %0, cpsr     @ local_save_flags\n"       \
                  : "=r" (x) :: "memory", "cc" );                \
})
#define local_irq_save(x)                                        \
({                                                               \
    local_save_flags(x);                                         \
    local_irq_disable();                                         \
})
#define local_irq_restore(x)                                     \
({                                                               \
    BUILD_BUG_ON(sizeof(x) != sizeof(long));                     \
    asm volatile (                                               \
            "msr     cpsr_c, %0      @ local_irq_restore\n"      \
            :                                                    \
            : "r" (x)                                            \
            : "memory", "cc");                                   \
})

static inline int local_irq_is_enabled(void)
{
    unsigned long flags;
    local_save_flags(flags);
    return !(flags & PSR_IRQ_MASK);
}

#define local_fiq_enable()  __asm__("cpsie f   @ __stf\n" : : : "memory", "cc")
#define local_fiq_disable() __asm__("cpsid f   @ __clf\n" : : : "memory", "cc")

#define local_abort_enable() __asm__("cpsie a  @ __sta\n" : : : "memory", "cc")
#define local_abort_disable() __asm__("cpsid a @ __sta\n" : : : "memory", "cc")

/* We do not support tracing (at all) yet */
#define trace_irq_disable_ret()   do { } while ( 0 )
#define trace_irq_enable_ret()    do { } while ( 0 )
#define trace_irq_save_ret(_x)    do { } while ( 0 )
#define trace_irq_restore_ret(_x) do { } while ( 0 )
#define _local_irq_disable()      local_irq_disable()
#define _local_irq_enable()       local_irq_enable()
#define _local_irq_save(_x)       local_irq_save(_x)
#define _local_irq_restore(_x)    local_irq_restore(_x)

static inline int local_fiq_is_enabled(void)
{
    unsigned long flags;
    local_save_flags(flags);
    return !(flags & PSR_FIQ_MASK);
}

#endif
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
