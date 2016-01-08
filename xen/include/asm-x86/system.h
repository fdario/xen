#ifndef __ASM_SYSTEM_H
#define __ASM_SYSTEM_H

#include <xen/lib.h>
#include <xen/bitops.h>
#include <asm/processor.h>

#include <xen/trace.h>

#define read_sreg(name)                                         \
({  unsigned int __sel;                                         \
    asm volatile ( "mov %%" STR(name) ",%0" : "=r" (__sel) );   \
    __sel;                                                      \
})

#define wbinvd() \
    asm volatile ( "wbinvd" : : : "memory" )

#define clflush(a) \
    asm volatile ( "clflush (%0)" : : "r"(a) )

#define nop() \
    asm volatile ( "nop" )

#define xchg(ptr,v) \
    ((__typeof__(*(ptr)))__xchg((unsigned long)(v),(ptr),sizeof(*(ptr))))

struct __xchg_dummy { unsigned long a[100]; };
#define __xg(x) ((volatile struct __xchg_dummy *)(x))

#include <asm/x86_64/system.h>

/*
 * Note: no "lock" prefix even on SMP: xchg always implies lock anyway
 * Note 2: xchg has side effect, so that attribute volatile is necessary,
 *   but generally the primitive is invalid, *ptr is output argument. --ANK
 */
static always_inline unsigned long __xchg(
    unsigned long x, volatile void *ptr, int size)
{
    switch ( size )
    {
    case 1:
        asm volatile ( "xchgb %b0,%1"
                       : "=q" (x)
                       : "m" (*__xg(ptr)), "0" (x)
                       : "memory" );
        break;
    case 2:
        asm volatile ( "xchgw %w0,%1"
                       : "=r" (x)
                       : "m" (*__xg(ptr)), "0" (x)
                       : "memory" );
        break;
    case 4:
        asm volatile ( "xchgl %k0,%1"
                       : "=r" (x)
                       : "m" (*__xg(ptr)), "0" (x)
                       : "memory" );
        break;
    case 8:
        asm volatile ( "xchgq %0,%1"
                       : "=r" (x)
                       : "m" (*__xg(ptr)), "0" (x)
                       : "memory" );
        break;
    }
    return x;
}

/*
 * Atomic compare and exchange.  Compare OLD with MEM, if identical,
 * store NEW in MEM.  Return the initial value in MEM.  Success is
 * indicated by comparing RETURN with OLD.
 */

static always_inline unsigned long __cmpxchg(
    volatile void *ptr, unsigned long old, unsigned long new, int size)
{
    unsigned long prev;
    switch ( size )
    {
    case 1:
        asm volatile ( "lock; cmpxchgb %b1,%2"
                       : "=a" (prev)
                       : "q" (new), "m" (*__xg(ptr)),
                       "0" (old)
                       : "memory" );
        return prev;
    case 2:
        asm volatile ( "lock; cmpxchgw %w1,%2"
                       : "=a" (prev)
                       : "r" (new), "m" (*__xg(ptr)),
                       "0" (old)
                       : "memory" );
        return prev;
    case 4:
        asm volatile ( "lock; cmpxchgl %k1,%2"
                       : "=a" (prev)
                       : "r" (new), "m" (*__xg(ptr)),
                       "0" (old)
                       : "memory" );
        return prev;
    case 8:
        asm volatile ( "lock; cmpxchgq %1,%2"
                       : "=a" (prev)
                       : "r" (new), "m" (*__xg(ptr)),
                       "0" (old)
                       : "memory" );
        return prev;
    }
    return old;
}

#define cmpxchgptr(ptr,o,n) ({                                          \
    const __typeof__(**(ptr)) *__o = (o);                               \
    __typeof__(**(ptr)) *__n = (n);                                     \
    ((__typeof__(*(ptr)))__cmpxchg((ptr),(unsigned long)__o,            \
                                   (unsigned long)__n,sizeof(*(ptr)))); \
})

/*
 * Undefined symbol to cause link failure if a wrong size is used with
 * arch_fetch_and_add().
 */
extern unsigned long __bad_fetch_and_add_size(void);

static always_inline unsigned long __xadd(
    volatile void *ptr, unsigned long v, int size)
{
    switch ( size )
    {
    case 1:
        asm volatile ( "lock; xaddb %b0,%1"
                       : "+r" (v), "+m" (*__xg(ptr))
                       :: "memory");
        return v;
    case 2:
        asm volatile ( "lock; xaddw %w0,%1"
                       : "+r" (v), "+m" (*__xg(ptr))
                       :: "memory");
        return v;
    case 4:
        asm volatile ( "lock; xaddl %k0,%1"
                       : "+r" (v), "+m" (*__xg(ptr))
                       :: "memory");
        return v;
    case 8:
        asm volatile ( "lock; xaddq %q0,%1"
                       : "+r" (v), "+m" (*__xg(ptr))
                       :: "memory");

        return v;
    default:
        return __bad_fetch_and_add_size();
    }
}

/*
 * Atomically add @v to the 1, 2, 4, or 8 byte value at @ptr.  Returns
 * the previous value.
 *
 * This is a full memory barrier.
 */
#define arch_fetch_and_add(ptr, v) \
    ((typeof(*(ptr)))__xadd(ptr, (typeof(*(ptr)))(v), sizeof(*(ptr))))

/*
 * Both Intel and AMD agree that, from a programmer's viewpoint:
 *  Loads cannot be reordered relative to other loads.
 *  Stores cannot be reordered relative to other stores.
 * 
 * Intel64 Architecture Memory Ordering White Paper
 * <http://developer.intel.com/products/processor/manuals/318147.pdf>
 * 
 * AMD64 Architecture Programmer's Manual, Volume 2: System Programming
 * <http://www.amd.com/us-en/assets/content_type/\
 *  white_papers_and_tech_docs/24593.pdf>
 */
#define rmb()           barrier()
#define wmb()           barrier()

#define smp_mb()        mb()
#define smp_rmb()       rmb()
#define smp_wmb()       wmb()

#define set_mb(var, value) do { xchg(&var, value); } while (0)
#define set_wmb(var, value) do { var = value; wmb(); } while (0)

#define _local_irq_disable()      asm volatile ( "cli" : : : "memory" )
#define _local_irq_enable()       asm volatile ( "sti" : : : "memory" )

#define TRACE_IRQ_DISABLED //XXX
//#undef TRACE_IRQ_DISABLED //XXX

#ifndef TRACE_IRQ_DISABLED
#define local_irq_disable()       _local_irq_disable()
#define local_irq_enable()        _local_irq_enable()
#else
#define TRACE_RET_ADDR     ((unsigned long) __builtin_return_address(0))

#define trace_irq_disable() \
({ \
    uint64_t addr = TRACE_RET_ADDR;                                         \
    if ( likely(local_irq_is_enabled()) )                                   \
        TRACE_2D(TRC_HW_IRQ_DISABLE, (unsigned) (addr >> 32),               \
                 (unsigned) addr);                                          \
})
#define trace_irq_enable()                                                  \
({                                                                          \
    uint64_t addr = TRACE_RET_ADDR;                                         \
    TRACE_2D(TRC_HW_IRQ_ENABLE, (unsigned) (addr >> 32), (unsigned) addr);  \
})
#define trace_irq_save(x)                                                   \
({                                                                          \
    uint64_t addr = TRACE_RET_ADDR;                                         \
    TRACE_3D(TRC_HW_IRQ_SAVE, !!((x) & X86_EFLAGS_IF),                      \
             (unsigned) (addr >> 32), (unsigned) addr);                     \
})
#define trace_irq_restore(x)                                                \
({                                                                          \
    uint64_t addr = TRACE_RET_ADDR;                                         \
    TRACE_3D(TRC_HW_IRQ_RESTORE, !!((x) & X86_EFLAGS_IF),                   \
             (unsigned) (addr >> 32), (unsigned) addr);                     \
})

void local_irq_disable(void);
void local_irq_enable(void);

void local_irq_restore(unsigned long flags);
void local_irq_save_raw(unsigned long *flags);
#define local_irq_save(x)         local_irq_save_raw(&(x))
#endif

/* used in the idle loop; sti takes one instruction cycle to complete */
#define safe_halt()     asm volatile ( "sti; hlt" : : : "memory" )
/* used when interrupts are already enabled or to shutdown the processor */
#define halt()          asm volatile ( "hlt" : : : "memory" )

#define local_save_flags(x)                                      \
({                                                               \
    BUILD_BUG_ON(sizeof(x) != sizeof(long));                     \
    asm volatile ( "pushf" __OS " ; pop" __OS " %0" : "=g" (x)); \
})
#define _local_irq_save(x)                                       \
({                                                               \
    local_save_flags(x);                                         \
    _local_irq_disable();                                        \
})
#define _local_irq_restore(x)                                    \
({                                                               \
    BUILD_BUG_ON(sizeof(x) != sizeof(long));                     \
    asm volatile ( "pushfq\n\t"                                  \
                   "andq %0, (%%rsp)\n\t"                        \
                   "orq  %1, (%%rsp)\n\t"                        \
                   "popfq"                                       \
                   : : "i?r" ( ~X86_EFLAGS_IF ),                 \
                       "ri" ( (x) & X86_EFLAGS_IF ) );           \
})

#ifndef TRACE_IRQ_DISABLED
#define local_irq_save(x)            _local_irq_save(x)
#define local_irq_restore(x)         _local_irq_restore(x)
#define local_irq_save_notrace(x)    local_irq_save(x)
#define local_irq_restore_notrace(x) local_irq_restore(x)
#else
#define local_irq_save_notrace(x)    _local_irq_save(x)
#define local_irq_restore_notrace(x) _local_irq_restore(x)
#endif

static inline int local_irq_is_enabled(void)
{
    unsigned long flags;
    local_save_flags(flags);
    return !!(flags & X86_EFLAGS_IF);
}

#define BROKEN_ACPI_Sx          0x0001
#define BROKEN_INIT_AFTER_S1    0x0002

void trap_init(void);
void init_idt_traps(void);
void load_system_tables(void);
void percpu_traps_init(void);
void subarch_percpu_traps_init(void);

#endif
