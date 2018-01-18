/******************************************************************************
 * current.h
 * 
 * Information structure that lives at the bottom of the per-cpu Xen stack.
 */

#ifndef __X86_CURRENT_H__
#define __X86_CURRENT_H__

#include <xen/percpu.h>
#include <public/xen.h>
#include <asm/config.h>
#include <asm/page.h>

#ifndef __ASSEMBLY__
/*
 * Xen's physical cpu stacks are 8 pages (8-page aligned), arranged as:
 *
 * 7 - Primary stack (with a struct cpu_info at the top)
 * 6 - Primary stack
 * 5 - Optionally not preset (MEMORY_GUARD)
 * 4 - unused
 * 3 - Syscall trampolines
 * 2 - MCE IST stack
 * 1 - NMI IST stack
 * 0 - Double Fault IST stack
 */

/*
 * The vcpu stacks used for XPTI are arranged similar to the physical cpu
 * stacks with some modifications. The main difference are the primary stack
 * size (only 1 page) and usage of the unused mappings for TSS and IDT.
 *
 * 7 - Primary stack (with a struct cpu_info at the top)
 * 6 - unused
 * 5 - TSS
 * 4 - unused
 * 3 - Syscall trampolines
 * 2 - MCE IST stack
 * 1 - NMI IST stack
 * 0 - Double Fault IST stack
 */

/*
 * Identify which stack page the stack pointer is on.  Returns an index
 * as per the comment above.
 */
static inline unsigned int get_stack_page(unsigned long sp)
{
    return (sp & (STACK_SIZE-1)) >> PAGE_SHIFT;
}

struct vcpu;

struct cpu_info {
    struct cpu_user_regs guest_cpu_user_regs;
    union {
        /* per physical cpu mapping */
        struct {
            struct vcpu *current_vcpu;
            unsigned long per_cpu_offset;
            unsigned long cr4;

            /* See asm-x86/spec_ctrl_asm.h for usage. */
            unsigned int shadow_spec_ctrl;
            bool         use_shadow_spec_ctrl;
            uint8_t      bti_ist_info;
            unsigned long p_pad;
        };
        /* per vcpu mapping (xpti) */
        struct {
            unsigned long v_pad[4];
            unsigned long stack_bottom_cpu;
        };
    };
    unsigned int processor_id;  /* per physical cpu mapping only */
    unsigned int flags;
#endif /* !__ASSEMBLY__ */
#define ON_VCPUSTACK      0x00000001
#define VCPUSTACK_ACTIVE  0x00000002
#ifndef __ASSEMBLY__
    /* get_stack_bottom() must be 16-byte aligned */
};

static inline struct cpu_info *get_cpu_info(void)
{
#ifdef __clang__
    /* Clang complains that sp in the else case is not initialised. */
    unsigned long sp;
    asm ( "mov %%rsp, %0" : "=r" (sp) );
#else
    register unsigned long sp asm("rsp");
#endif

    return (struct cpu_info *)((sp | (STACK_SIZE - 1)) + 1) - 1;
}

#define get_current()         (get_cpu_info()->current_vcpu)
#define set_current(vcpu)     (get_cpu_info()->current_vcpu = (vcpu))
#define current               (get_current())

#define get_processor_id()    (get_cpu_info()->processor_id)
#define set_processor_id(id)  do {                                      \
    struct cpu_info *ci__ = get_cpu_info();                             \
    ci__->per_cpu_offset = __per_cpu_offset[ci__->processor_id = (id)]; \
    ci__->flags = 0;                                                    \
} while (0)

#define guest_cpu_user_regs() ({                                        \
    struct cpu_info *info = get_cpu_info();                             \
    if ( info->flags & VCPUSTACK_ACTIVE )                               \
        info = (struct cpu_info *)(XPTI_START(info->current_vcpu) +     \
                                   STACK_SIZE) - 1;                     \
    &info->guest_cpu_user_regs;                                         \
})

/*
 * Get the bottom-of-stack, as stored in the per-CPU TSS. This actually points
 * into the middle of cpu_info.guest_cpu_user_regs, at the section that
 * precisely corresponds to a CPU trap frame.
 */
#define get_stack_bottom()                      \
    ((unsigned long)&get_cpu_info()->guest_cpu_user_regs.es)

/*
 * Get the reasonable stack bounds for stack traces and stack dumps.  Stack
 * dumps have a slightly larger range to include exception frames in the
 * printed information.  The returned word is inside the interesting range.
 */
unsigned long get_stack_trace_bottom(unsigned long sp);
unsigned long get_stack_dump_bottom (unsigned long sp);

#ifdef CONFIG_LIVEPATCH
# define CHECK_FOR_LIVEPATCH_WORK "call check_for_livepatch_work;"
#else
# define CHECK_FOR_LIVEPATCH_WORK ""
#endif

#define reset_stack_and_jump(__fn)                                      \
    ({                                                                  \
        __asm__ __volatile__ (                                          \
            "mov %0,%%"__OP"sp;"                                        \
            "mov %1,%%r12;"                                             \
            CHECK_FOR_LIVEPATCH_WORK                                    \
            "jmp %c2"                                                   \
            : : "r" (get_cpu_info()), "r" (guest_cpu_user_regs()),      \
                "i" (__fn) : "memory" );                                \
        unreachable();                                                  \
    })

/*
 * Which VCPU's state is currently running on each CPU?
 * This is not necesasrily the same as 'current' as a CPU may be
 * executing a lazy state switch.
 */
DECLARE_PER_CPU(struct vcpu *, curr_vcpu);

#endif /* !__ASSEMBLY__ */

#endif /* __X86_CURRENT_H__ */
