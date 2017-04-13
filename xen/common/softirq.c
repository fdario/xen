/******************************************************************************
 * common/softirq.c
 * 
 * Softirqs in Xen are only executed in an outermost activation (e.g., never 
 * within an interrupt activation). This simplifies some things and generally 
 * seems a good thing.
 * 
 * Copyright (c) 2003, K A Fraser
 * Copyright (c) 1992, Linus Torvalds
 */

#include <xen/init.h>
#include <xen/mm.h>
#include <xen/preempt.h>
#include <xen/sched.h>
#include <xen/trace.h>
#include <xen/rcupdate.h>
#include <xen/softirq.h>

#ifndef __ARCH_IRQ_STAT
irq_cpustat_t irq_stat[NR_CPUS];
#endif

static softirq_handler softirq_handlers[NR_SOFTIRQS];

static DEFINE_PER_CPU(cpumask_t, batch_mask);
static DEFINE_PER_CPU(unsigned int, batching);

#ifdef CONFIG_TRACE_SOFTIRQS
#define trace_raise(n)        TRACE_1D(TRC_XEN_SIRQ_RAISE, n)
static inline void trace_raise_cpu(unsigned int nr, unsigned int cpu)
{
    struct {
        uint16_t cpu, nr;
    } d;

    if ( likely(!tb_init_done) )
        return;

    d.nr = nr;
    d.cpu = cpu;
    __trace_var(TRC_XEN_SIRQ_RAISE_CPU, 1, sizeof(d), &d);
}
static inline
void trace_raise_mask(unsigned int nr, const cpumask_t *m)
{
    struct {
        uint32_t nr;
        uint32_t mask[6];
    } d;

    if ( likely(!tb_init_done) )
        return;

    d.nr = nr;
    memset(d.mask, 0, sizeof(d.mask));
    memcpy(d.mask, m, min(sizeof(d.mask), sizeof(cpumask_t)));
    __trace_var(TRC_XEN_SIRQ_RAISE_MASK, 1, sizeof(d), &d);
}
static inline void trace_handler(unsigned int nr, unsigned int cpu)
{
    struct {
        uint16_t pending, nr;
    } d;

    if ( likely(!tb_init_done) )
        return;

    d.nr = nr;
    d.pending = (uint16_t)softirq_pending(cpu);
    __trace_var(TRC_XEN_SIRQ_HANDLER, 1, sizeof(d), &d);
}
#else /* !TRACE_SOFTIRQS */
#define trace_raise(n)            do {} while ( 0 )
#define trace_raise_cpu(n, c)     do {} while ( 0 )
#define trace_raise_mask(n, m) do {} while ( 0 )
#define trace_handler(n, c)       do {} while ( 0 )
#endif /* TRACE_SOFTIRQS */

static void __do_softirq(unsigned long ignore_mask)
{
    unsigned int i, cpu = smp_processor_id();
    unsigned long pending;

    for ( ; ; )
    {
        ASSERT(cpu == smp_processor_id());

        if ( rcu_pending(cpu) )
            rcu_check_callbacks(cpu);

        if ( ((pending = (softirq_pending(cpu) & ~ignore_mask)) == 0)
             || cpu_is_offline(cpu) )
            break;

        i = find_first_set_bit(pending);
        clear_bit(i, &softirq_pending(cpu));
        trace_handler(i, cpu);
        (*softirq_handlers[i])();
    }
}

void process_pending_softirqs(void)
{
    ASSERT(!in_irq() && local_irq_is_enabled());
    /* Do not enter scheduler as it can preempt the calling context. */
    __do_softirq(1ul<<SCHEDULE_SOFTIRQ);
}

asmlinkage void do_softirq(void)
{
    ASSERT_NOT_IN_ATOMIC();
    __do_softirq(0);
}

void open_softirq(int nr, softirq_handler handler)
{
    ASSERT(nr < NR_SOFTIRQS);
    softirq_handlers[nr] = handler;
}

void cpumask_raise_softirq(const cpumask_t *mask, unsigned int nr)
{
    unsigned int cpu, this_cpu = smp_processor_id();
    cpumask_t send_mask, *raise_mask;

    trace_raise_mask(nr, mask);

    if ( !per_cpu(batching, this_cpu) || in_irq() )
    {
        cpumask_clear(&send_mask);
        raise_mask = &send_mask;
    }
    else
        raise_mask = &per_cpu(batch_mask, this_cpu);

    for_each_cpu(cpu, mask)
        if ( !test_and_set_bit(nr, &softirq_pending(cpu)) &&
             cpu != this_cpu &&
             !arch_skip_send_event_check(cpu) )
            __cpumask_set_cpu(cpu, raise_mask);

    if ( raise_mask == &send_mask )
        smp_send_event_check_mask(raise_mask);
}

void cpu_raise_softirq(unsigned int cpu, unsigned int nr)
{
    unsigned int this_cpu = smp_processor_id();

    trace_raise_cpu(nr, cpu);

    if ( test_and_set_bit(nr, &softirq_pending(cpu))
         || (cpu == this_cpu)
         || arch_skip_send_event_check(cpu) )
        return;

    if ( !per_cpu(batching, this_cpu) || in_irq() )
        smp_send_event_check_cpu(cpu);
    else
        __cpumask_set_cpu(cpu, &per_cpu(batch_mask, this_cpu));
}

void cpu_raise_softirq_batch_begin(void)
{
    ++this_cpu(batching);
}

void cpu_raise_softirq_batch_finish(void)
{
    unsigned int cpu, this_cpu = smp_processor_id();
    cpumask_t *mask = &per_cpu(batch_mask, this_cpu);

    ASSERT(per_cpu(batching, this_cpu));
    for_each_cpu ( cpu, mask )
        if ( !softirq_pending(cpu) )
            __cpumask_clear_cpu(cpu, mask);
    smp_send_event_check_mask(mask);
    cpumask_clear(mask);
    --per_cpu(batching, this_cpu);
}

void raise_softirq(unsigned int nr)
{
    trace_raise(nr);
    set_bit(nr, &softirq_pending(smp_processor_id()));
}

void __init softirq_init(void)
{
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
