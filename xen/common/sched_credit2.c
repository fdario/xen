
/****************************************************************************
 * (C) 2009 - George Dunlap - Citrix Systems R&D UK, Ltd
 ****************************************************************************
 *
 *        File: common/sched_credit2.c
 *      Author: George Dunlap
 *
 * Description: Credit-based SMP CPU scheduler
 * Based on an earlier verson by Emmanuel Ackaouy.
 */

#include <xen/config.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/perfc.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <asm/div64.h>
#include <xen/errno.h>
#include <xen/trace.h>
#include <xen/cpu.h>
#include <xen/keyhandler.h>

/* Meant only for helping developers during debugging. */
/* #define d2printk printk */
#define d2printk(x...)


/*
 * Credit2 tracing events ("only" 512 available!). Check
 * include/public/trace.h for more details.
 */
#define TRC_CSCHED2_TICK             TRC_SCHED_CLASS_EVT(CSCHED2, 1)
#define TRC_CSCHED2_RUNQ_POS         TRC_SCHED_CLASS_EVT(CSCHED2, 2)
#define TRC_CSCHED2_CREDIT_BURN      TRC_SCHED_CLASS_EVT(CSCHED2, 3)
#define TRC_CSCHED2_CREDIT_ADD       TRC_SCHED_CLASS_EVT(CSCHED2, 4)
#define TRC_CSCHED2_TICKLE_CHECK     TRC_SCHED_CLASS_EVT(CSCHED2, 5)
#define TRC_CSCHED2_TICKLE           TRC_SCHED_CLASS_EVT(CSCHED2, 6)
#define TRC_CSCHED2_CREDIT_RESET     TRC_SCHED_CLASS_EVT(CSCHED2, 7)
#define TRC_CSCHED2_SCHED_TASKLET    TRC_SCHED_CLASS_EVT(CSCHED2, 8)
#define TRC_CSCHED2_UPDATE_LOAD      TRC_SCHED_CLASS_EVT(CSCHED2, 9)
#define TRC_CSCHED2_RUNQ_ASSIGN      TRC_SCHED_CLASS_EVT(CSCHED2, 10)
#define TRC_CSCHED2_UPDATE_VCPU_LOAD TRC_SCHED_CLASS_EVT(CSCHED2, 11)
#define TRC_CSCHED2_UPDATE_RUNQ_LOAD TRC_SCHED_CLASS_EVT(CSCHED2, 12)
#define TRC_CSCHED2_TICKLE_NEW       TRC_SCHED_CLASS_EVT(CSCHED2, 13)
#define TRC_CSCHED2_RUNQ_MAX_WEIGHT  TRC_SCHED_CLASS_EVT(CSCHED2, 14)
#define TRC_CSCHED2_MIGRATE          TRC_SCHED_CLASS_EVT(CSCHED2, 15)
#define TRC_CSCHED2_LOAD_CHECK       TRC_SCHED_CLASS_EVT(CSCHED2, 16)
#define TRC_CSCHED2_LOAD_BALANCE     TRC_SCHED_CLASS_EVT(CSCHED2, 17)
#define TRC_CSCHED2_PICKED_CPU       TRC_SCHED_CLASS_EVT(CSCHED2, 19)
#define TRC_CSCHED2_RUNQ_CANDIDATE   TRC_SCHED_CLASS_EVT(CSCHED2, 20)
#define TRC_CSCHED2_SCHEDULE         TRC_SCHED_CLASS_EVT(CSCHED2, 21)
#define TRC_CSCHED2_RATELIMIT        TRC_SCHED_CLASS_EVT(CSCHED2, 22)

/*
 * WARNING: This is still in an experimental phase.  Status and work can be found at the
 * credit2 wiki page:
 *  http://wiki.xen.org/wiki/Credit2_Scheduler_Development
 *
 * TODO:
 * + Hyperthreading
 *  - Look for non-busy core if possible
 *  - "Discount" time run on a thread with busy siblings
 * + Algorithm:
 *  - "Mixed work" problem: if a VM is playing audio (5%) but also burning cpu (e.g.,
 *    a flash animation in the background) can we schedule it with low enough latency
 *    so that audio doesn't skip?
 *  - Cap and reservation: How to implement with the current system?
 * + Optimizing
 *  - Profiling, making new algorithms, making math more efficient (no long division)
 */

/*
 * Design:
 *
 * VMs "burn" credits based on their weight; higher weight means
 * credits burn more slowly.  The highest weight vcpu burns credits at
 * a rate of 1 credit per nanosecond.  Others burn proportionally
 * more.
 *
 * vcpus are inserted into the runqueue by credit order.
 *
 * Credits are "reset" when the next vcpu in the runqueue is less than
 * or equal to zero.  At that point, everyone's credits are "clipped"
 * to a small value, and a fixed credit is added to everyone.
 */

/*
 * Locking:
 *
 * - runqueue lock
 *  + it is per-runqueue, so:
 *   * cpus in a runqueue take the runqueue lock, when using
 *     pcpu_schedule_lock() / vcpu_schedule_lock() (and friends),
 *   * a cpu may (try to) take a "remote" runqueue lock, e.g., for
 *     load balancing;
 *  + serializes runqueue operations (removing and inserting vcpus);
 *  + protects runqueue-wide data in csched2_runqueue_data;
 *  + protects vcpu parameters in csched2_vcpu for the vcpu in the
 *    runqueue.
 *
 * - Private scheduler lock
 *  + protects scheduler-wide data in csched2_private, such as:
 *   * the list of domains active in this scheduler,
 *   * what cpus and what runqueues are active and in what
 *     runqueue each cpu is;
 *  + serializes the operation of changing the weights of domains;
 *
 * - Type:
 *  + runqueue locks are 'regular' spinlocks;
 *  + the private scheduler lock can be an rwlock. In fact, data
 *    it protects is modified only during initialization, cpupool
 *    manipulation and when changing weights, and read in all
 *    other cases (e.g., during load balancing).
 *
 * Ordering:
 *  + tylock must be used when wanting to take a runqueue lock,
 *    if we already hold another one;
 *  + if taking both a runqueue lock and the private scheduler
 *    lock is, the latter must always be taken for first.
 */

/*
 * Basic constants
 */
/* Default weight: How much a new domain starts with */
#define CSCHED2_DEFAULT_WEIGHT       256
/* Min timer: Minimum length a timer will be set, to
 * achieve efficiency */
#define CSCHED2_MIN_TIMER            MICROSECS(500)
/* Amount of credit VMs begin with, and are reset to.
 * ATM, set so that highest-weight VMs can only run for 10ms
 * before a reset event. */
#define CSCHED2_CREDIT_INIT          MILLISECS(10)
/* Carryover: How much "extra" credit may be carried over after
 * a reset. */
#define CSCHED2_CARRYOVER_MAX        CSCHED2_MIN_TIMER
/* Stickiness: Cross-L2 migration resistance.  Should be less than
 * MIN_TIMER. */
#define CSCHED2_MIGRATE_RESIST       ((opt_migrate_resist)*MICROSECS(1))
/* How much to "compensate" a vcpu for L2 migration */
#define CSCHED2_MIGRATE_COMPENSATION MICROSECS(50)
/* How big of a bias we should have against a yielding vcpu */
#define CSCHED2_YIELD_BIAS           ((opt_yield_bias)*MICROSECS(1))
#define CSCHED2_YIELD_BIAS_MIN       CSCHED2_MIN_TIMER
/* Reset: Value below which credit will be reset. */
#define CSCHED2_CREDIT_RESET         0
/* Max timer: Maximum time a guest can be run for. */
#define CSCHED2_MAX_TIMER            MILLISECS(2)


#define CSCHED2_IDLE_CREDIT                 (-(1<<30))

/*
 * Flags
 */
/* CSFLAG_scheduled: Is this vcpu either running on, or context-switching off,
 * a physical cpu?
 * + Accessed only with runqueue lock held
 * + Set when chosen as next in csched2_schedule().
 * + Cleared after context switch has been saved in csched2_context_saved()
 * + Checked in vcpu_wake to see if we can add to the runqueue, or if we should
 *   set CSFLAG_delayed_runq_add
 * + Checked to be false in runq_insert.
 */
#define __CSFLAG_scheduled 1
#define CSFLAG_scheduled (1<<__CSFLAG_scheduled)
/* CSFLAG_delayed_runq_add: Do we need to add this to the runqueue once it'd done
 * being context switched out?
 * + Set when scheduling out in csched2_schedule() if prev is runnable
 * + Set in csched2_vcpu_wake if it finds CSFLAG_scheduled set
 * + Read in csched2_context_saved().  If set, it adds prev to the runqueue and
 *   clears the bit.
 */
#define __CSFLAG_delayed_runq_add 2
#define CSFLAG_delayed_runq_add (1<<__CSFLAG_delayed_runq_add)
/* CSFLAG_runq_migrate_request: This vcpu is being migrated as a result of a
 * credit2-initiated runq migrate request; migrate it to the runqueue indicated
 * in the svc struct. 
 */
#define __CSFLAG_runq_migrate_request 3
#define CSFLAG_runq_migrate_request (1<<__CSFLAG_runq_migrate_request)
/*
 * CSFLAG_vcpu_yield: this vcpu was running, and has called vcpu_yield(). The
 * scheduler is invoked to see if we can give the cpu to someone else, and
 * get back to the yielding vcpu in a while.
 */
#define __CSFLAG_vcpu_yield 4
#define CSFLAG_vcpu_yield (1<<__CSFLAG_vcpu_yield)

static unsigned int __read_mostly opt_migrate_resist = 500;
integer_param("sched_credit2_migrate_resist", opt_migrate_resist);

static unsigned int __read_mostly opt_yield_bias = 1000;
integer_param("sched_credit2_yield_bias", opt_yield_bias);

/*
 * Useful macros
 */
#define CSCHED2_PRIV(_ops)   \
    ((struct csched2_private *)((_ops)->sched_data))
#define CSCHED2_VCPU(_vcpu)  ((struct csched2_vcpu *) (_vcpu)->sched_priv)
#define CSCHED2_DOM(_dom)    ((struct csched2_dom *) (_dom)->sched_priv)
/* CPU to runq_id macro */
#define c2r(_ops, _cpu)     (CSCHED2_PRIV(_ops)->runq_map[(_cpu)])
/* CPU to runqueue struct macro */
#define RQD(_ops, _cpu)     (&CSCHED2_PRIV(_ops)->rqd[c2r(_ops, _cpu)])

/*
 * Load tracking and load balancing
 *
 * Load history of runqueues and vcpus is accounted for by using an
 * exponential weighted moving average algorithm. However, instead of using
 * fractions,we shift everything to left by the number of bits we want to
 * use for representing the fractional part (Q-format).
 *
 * We may also want to reduce the precision of time accounting, to
 * accommodate 'longer  windows'. So, if that is the case, we just need to
 * shift all time samples to the right.
 *
 * The details of the formulas used for load tracking are explained close to
 * __update_runq_load(). Let's just say here that, with full nanosecond time
 * granularity, a 30 bits wide 'decaying window' is ~1 second long.
 *
 * We want to consider the following equations:
 *
 *  avg[0] = load*P
 *  avg[i+1] = avg[i] + delta*load*P/W - delta*avg[i]/W,  0 <= delta <= W
 *
 * where W is the lenght of the window, P the multiplier for transitiong into
 * Q-format fixed point arithmetic and load is the instantaneous load of a
 * runqueue, which basically is the number of runnable vcpus there are on the
 * runqueue (for the meaning of the other terms, look at the doc comment to
 *  __update_runq_load()).
 *
 *  So, again, with full nanosecond granularity, and 1 second window, we have:
 *
 *  W = 2^30
 *  P = 2^18
 *
 * The maximum possible value for the average load, which we want to store in
 * s_time_t type variables (i.e., we have 63 bits available) is load*P. This
 * means that, with P 18 bits wide, load can occupy 45 bits. This in turn
 * means we can have 2^45 vcpus in each runqueue, before overflow occurs!
 *
 * However, it can happen that, at step j+1, if:
 *
 *  avg[j] = load*P
 *  delta = W
 *
 * then:
 *
 *  avg[j+i] = avg[j] + W*load*P/W - W*load*P/W
 *
 * So we must be able to deal with W*load*P. This means load can't be higher
 * than:
 *
 *  2^(63 - 30 - 18) = 2^15 = 32768
 *
 * So 32768 is the maximum number of vcpus the we can have in a runqueue,
 * at any given time, and still not have problems with the load tracking
 * calculations... and this is more than fine.
 *
 * As a matter of fact, since we are using microseconds granularity, we have
 * W=2^20. So, still with 18 fractional bits and a 1 second long window, there
 * may be 2^25 = 33554432 vcpus in a runq before we have to start thinking
 * about overflow.
 */

/* If >0, decreases the granularity of time samples used for load tracking. */
#define LOADAVG_GRANULARITY_SHIFT   (10)
/* Time window during which we still give value to previous load history. */
#define LOADAVG_WINDOW_SHIFT        (30)
/* 18 bits by default (and not less than 4) for decimals. */
#define LOADAVG_PRECISION_SHIFT     (18)
#define LOADAVG_PRECISION_SHIFT_MIN (4)

/*
 * Both the lenght of the window and the number of fractional bits can be
 * decided with boot parameters.
 *
 * The length of the window is always expressed in nanoseconds. The actual
 * value used by default is LOADAVG_WINDOW_SHIFT - LOADAVG_GRANULARITY_SHIFT.
 */
static unsigned int __read_mostly opt_load_window_shift = LOADAVG_WINDOW_SHIFT;
integer_param("credit2_load_window_shift", opt_load_window_shift);
static unsigned int __read_mostly opt_load_precision_shift = LOADAVG_PRECISION_SHIFT;
integer_param("credit2_load_precision_shift", opt_load_precision_shift);

static int __read_mostly opt_underload_balance_tolerance = 0;
integer_param("credit2_balance_under", opt_underload_balance_tolerance);
static int __read_mostly opt_overload_balance_tolerance = -3;
integer_param("credit2_balance_over", opt_overload_balance_tolerance);

/*
 * Runqueue organization.
 *
 * The various cpus are to be assigned each one to a runqueue, and we
 * want that to happen basing on topology. At the moment, it is possible
 * to choose to arrange runqueues to be:
 *
 * - per-core: meaning that there will be one runqueue per each physical
 *             core of the host. This will happen if the opt_runqueue
 *             parameter is set to 'core';
 *
 * - per-socket: meaning that there will be one runqueue per each physical
 *               socket (AKA package, which often, but not always, also
 *               matches a NUMA node) of the host; This will happen if
 *               the opt_runqueue parameter is set to 'socket';
 *
 * - per-node: meaning that there will be one runqueue per each physical
 *             NUMA node of the host. This will happen if the opt_runqueue
 *             parameter is set to 'node';
 *
 * - global: meaning that there will be only one runqueue to which all the
 *           (logical) processors of the host belong. This will happen if
 *           the opt_runqueue parameter is set to 'all'.
 *
 * Depending on the value of opt_runqueue, therefore, cpus that are part of
 * either the same physical core, the same physical socket, the same NUMA
 * node, or just all of them, will be put together to form runqueues.
 */
#define OPT_RUNQUEUE_CORE   0
#define OPT_RUNQUEUE_SOCKET 1
#define OPT_RUNQUEUE_NODE   2
#define OPT_RUNQUEUE_ALL    3
static const char *const opt_runqueue_str[] = {
    [OPT_RUNQUEUE_CORE] = "core",
    [OPT_RUNQUEUE_SOCKET] = "socket",
    [OPT_RUNQUEUE_NODE] = "node",
    [OPT_RUNQUEUE_ALL] = "all"
};
static int __read_mostly opt_runqueue = OPT_RUNQUEUE_CORE;

static void parse_credit2_runqueue(const char *s)
{
    unsigned int i;

    for ( i = 0; i < ARRAY_SIZE(opt_runqueue_str); i++ )
    {
        if ( !strcmp(s, opt_runqueue_str[i]) )
        {
            opt_runqueue = i;
            return;
        }
    }

    printk("WARNING, unrecognized value of credit2_runqueue option!\n");
}
custom_param("credit2_runqueue", parse_credit2_runqueue);

/*
 * Per-runqueue data
 */
struct csched2_runqueue_data {
    int id;

    spinlock_t lock;      /* Lock for this runqueue. */
    cpumask_t active;      /* CPUs enabled for this runqueue */

    struct list_head runq; /* Ordered list of runnable vms */
    struct list_head svc;  /* List of all vcpus assigned to this runqueue */
    unsigned int max_weight;

    cpumask_t idle,        /* Currently idle pcpus */
        smt_idle,          /* Fully idle-and-untickled cores (see below) */
        tickled;           /* Have been asked to go through schedule */
    int load;              /* Instantaneous load: Length of queue  + num non-idle threads */
    s_time_t load_last_update;  /* Last time average was updated */
    s_time_t avgload;           /* Decaying queue load */
    s_time_t b_avgload;         /* Decaying queue load modified by balancing */
};

/*
 * System-wide private data
 */
struct csched2_private {
    rwlock_t lock;
    cpumask_t initialized; /* CPU is initialized for this pool */
    
    struct list_head sdom; /* Used mostly for dump keyhandler. */

    int runq_map[NR_CPUS];
    cpumask_t active_queues; /* Queues which may have active cpus */
    struct csched2_runqueue_data rqd[NR_CPUS];

    unsigned int load_precision_shift;
    unsigned int load_window_shift;
    unsigned ratelimit_us; /* each cpupool can have its own ratelimit */
};

/*
 * Virtual CPU
 */
struct csched2_vcpu {
    struct list_head rqd_elem;         /* On the runqueue data list  */
    struct list_head runq_elem;        /* On the runqueue            */
    struct csched2_runqueue_data *rqd; /* Up-pointer to the runqueue */

    /* Up-pointers */
    struct csched2_dom *sdom;
    struct vcpu *vcpu;

    unsigned int weight;
    unsigned int residual;

    int credit;
    s_time_t start_time; /* When we were scheduled (used for credit) */
    unsigned flags;      /* 16 bits doesn't seem to play well with clear_bit() */
    int tickled_cpu;     /* cpu tickled for picking us up (-1 if none) */

    /* Individual contribution to load */
    s_time_t load_last_update;  /* Last time average was updated */
    s_time_t avgload;           /* Decaying queue load */

    struct csched2_runqueue_data *migrate_rqd; /* Pre-determined rqd to which to migrate */
};

/*
 * Domain
 */
struct csched2_dom {
    struct list_head sdom_elem;
    struct domain *dom;
    uint16_t weight;
    uint16_t nr_vcpus;
};

/*
 * Hyperthreading (SMT) support.
 *
 * We use a special per-runq mask (smt_idle) and update it according to the
 * following logic:
 *  - when _all_ the SMT sibling in a core are idle, all their corresponding
 *    bits are set in the smt_idle mask;
 *  - when even _just_one_ of the SMT siblings in a core is not idle, all the
 *    bits correspondings to it and to all its siblings are clear in the
 *    smt_idle mask.
 *
 * Once we have such a mask, it is easy to implement a policy that, either:
 *  - uses fully idle cores first: it is enough to try to schedule the vcpus
 *    on pcpus from smt_idle mask first. This is what happens if
 *    sched_smt_power_savings was not set at boot (default), and it maximizes
 *    true parallelism, and hence performance;
 *  - uses already busy cores first: it is enough to try to schedule the vcpus
 *    on pcpus that are idle, but are not in smt_idle. This is what happens if
 *    sched_smt_power_savings is set at boot, and it allows as more cores as
 *    possible to stay in low power states, minimizing power consumption.
 *
 * This logic is entirely implemented in runq_tickle(), and that is enough.
 * In fact, in this scheduler, placement of a vcpu on one of the pcpus of a
 * runq, _always_ happens by means of tickling:
 *  - when a vcpu wakes up, it calls csched2_vcpu_wake(), which calls
 *    runq_tickle();
 *  - when a migration is initiated in schedule.c, we call csched2_cpu_pick(),
 *    csched2_vcpu_migrate() (which calls migrate()) and csched2_vcpu_wake().
 *    csched2_cpu_pick() looks for the least loaded runq and return just any
 *    of its processors. Then, csched2_vcpu_migrate() just moves the vcpu to
 *    the chosen runq, and it is again runq_tickle(), called by
 *    csched2_vcpu_wake() that actually decides what pcpu to use within the
 *    chosen runq;
 *  - when a migration is initiated in sched_credit2.c, by calling  migrate()
 *    directly, that again temporarily use a random pcpu from the new runq,
 *    and then calls runq_tickle(), by itself.
 */

/*
 * If all the siblings of cpu (including cpu itself) are both idle and
 * untickled, set all their bits in mask.
 *
 * NB that rqd->smt_idle is different than rqd->idle.  rqd->idle
 * records pcpus that at are merely idle (i.e., at the moment do not
 * have a vcpu running on them).  But you have to manually filter out
 * which pcpus have been tickled in order to find cores that are not
 * going to be busy soon.  Filtering out tickled cpus pairwise is a
 * lot of extra pain; so for rqd->smt_idle, we explicitly make so that
 * the bits of a pcpu are set only if all the threads on its core are
 * both idle *and* untickled.
 *
 * This means changing the mask when either rqd->idle or rqd->tickled
 * changes.
 */
static inline
void smt_idle_mask_set(unsigned int cpu, const cpumask_t *idlers,
                       cpumask_t *mask)
{
    const cpumask_t *cpu_siblings = per_cpu(cpu_sibling_mask, cpu);

    if ( cpumask_subset(cpu_siblings, idlers) )
        cpumask_or(mask, mask, cpu_siblings);
}

/*
 * Clear the bits of all the siblings of cpu from mask.
 */
static inline
void smt_idle_mask_clear(unsigned int cpu, cpumask_t *mask)
{
    cpumask_andnot(mask, mask, per_cpu(cpu_sibling_mask, cpu));
}

/*
 * In csched2_cpu_pick(), it may not be possible to actually look at remote
 * runqueues (the trylock-s on their spinlocks can fail!). If that happens,
 * we pick, in order of decreasing preference:
 *  1) svc's current pcpu, if it is part of svc's soft affinity;
 *  2) a pcpu in svc's current runqueue that is also in svc's soft affinity;
 *  3) just one valid pcpu from svc's soft affinity;
 *  4) svc's current pcpu, if it is part of svc's hard affinity;
 *  5) a pcpu in svc's current runqueue that is also in svc's hard affinity;
 *  6) just one valid pcpu from svc's hard affinity
 *
 * Of course, 1, 2 and 3 makes sense only if svc has a soft affinity. Also
 * note that at least 6 is guaranteed to _always_ return at least one pcpu.
 */
static int get_fallback_cpu(struct csched2_vcpu *svc)
{
    int cpu;
    unsigned int bs;

    for_each_affinity_balance_step( bs )
    {
        if ( bs == BALANCE_SOFT_AFFINITY &&
             !has_soft_affinity(svc->vcpu, svc->vcpu->cpu_hard_affinity) )
            continue;

        affinity_balance_cpumask(svc->vcpu, bs, cpumask_scratch);

        /*
         * This is cases 1 or 4 (depending on bs): if v->processor is (still)
         * in our affinity, go for it, for cache betterness.
         */
        if ( likely(cpumask_test_cpu(svc->vcpu->processor,
                                     cpumask_scratch)) )
            return svc->vcpu->processor;

        /*
         * This is cases 2 or 5 (depending on bsp): v->processor isn't there
         * any longer, check if we at least can stay in our current runq.
         */
        cpumask_and(cpumask_scratch, cpumask_scratch,
                    &svc->rqd->active);
        cpu = cpumask_first(cpumask_scratch);
        if ( likely(cpu < nr_cpu_ids) )
            return cpu;

        /*
         * This is cases 3 or 6 (depending on bs): last stand, just one valid
         * pcpu from our soft affinity, if we have one and if there's any. In
         * fact, if we are doing soft-affinity, it is possible that we fail,
         * which means we stay in the loop and look for hard affinity. OTOH,
         * if we are at the hard-affinity balancing step, it's guaranteed that
         * there is at least one valid cpu, and therefore we are sure that we
         * return it, and never really exit the loop.
         */
        cpumask_and(cpumask_scratch, cpumask_scratch,
                    cpupool_domain_cpumask(svc->vcpu->domain));
        ASSERT(!cpumask_empty(cpumask_scratch) || bs == BALANCE_SOFT_AFFINITY);
        cpu = cpumask_first(cpumask_scratch);
        if ( likely(cpu < nr_cpu_ids) )
            return cpu;
    }
    BUG_ON(1);
    return -1;
}

/*
 * Time-to-credit, credit-to-time.
 * 
 * We keep track of the "residual" time to make sure that frequent short
 * schedules still get accounted for in the end.
 *
 * FIXME: Do pre-calculated division?
 */
static void t2c_update(struct csched2_runqueue_data *rqd, s_time_t time,
                          struct csched2_vcpu *svc)
{
    uint64_t val = time * rqd->max_weight + svc->residual;

    svc->residual = do_div(val, svc->weight);
    svc->credit -= val;
}

static s_time_t c2t(struct csched2_runqueue_data *rqd, s_time_t credit, struct csched2_vcpu *svc)
{
    return credit * svc->weight / rqd->max_weight;
}

/*
 * Runqueue related code
 */

static /*inline*/ int
__vcpu_on_runq(struct csched2_vcpu *svc)
{
    return !list_empty(&svc->runq_elem);
}

static /*inline*/ struct csched2_vcpu *
__runq_elem(struct list_head *elem)
{
    return list_entry(elem, struct csched2_vcpu, runq_elem);
}

/*
 * Track the runq load by gathering instantaneous load samples, and using
 * exponentially weighted moving average (EWMA) for the 'decaying'.
 *
 * We consider a window of lenght W=2^(prv->load_window_shift) nsecs
 * (which takes LOADAVG_GRANULARITY_SHIFT into account).
 *
 * If load is the instantaneous load, the formula for EWMA looks as follows,
 * for the i-eth sample:
 *
 *  avg[i] = a*load + (1 - a)*avg[i-1]
 *
 * where avg[i] is the new value of the average load, avg[i-1] is the value
 * of the average load calculated so far, and a is a coefficient less or
 * equal to 1.
 *
 * So, for us, it becomes:
 *
 *  avgload = a*load + (1 - a)*avgload
 *
 * For determining a, we consider _when_ we are doing the load update, wrt
 * the lenght of the window. We define delta as follows:
 *
 *  delta = t - load_last_update
 *
 * where t is current time (i.e., time at which we are both sampling and
 * updating the load average) and load_last_update is the last time we did
 * that.
 *
 * There are two possible situations:
 *
 * a) delta <= W
 *    this means that, during the last window of lenght W, the runeuque load
 *    was avgload for (W - detla) time, and load for delta time:
 *
 *                |----------- W ---------|
 *                |                       |
 *                |     load_last_update  t
 *     -------------------------|---------|---
 *                |             |         |
 *                \__W - delta__/\_delta__/
 *                |             |         |
 *                |___avgload___|__load___|
 *
 *    So, what about using delta/W as our smoothing coefficient a. If we do,
 *    here's what happens:
 *
 *     a = delta / W
 *     1 - a = 1 - (delta / W) = (W - delta) / W
 *
 *    Which matches the above description of what happened in the last
 *    window of lenght W.
 *
 *    Note that this also means that the weight that we assign to both the
 *    latest load sample, and to previous history, varies at each update.
 *    The longer the latest load sample has been in efect, within the last
 *    window, the higher it weights (and the lesser the previous history
 *    weights).
 *
 *    This is some sort of extension of plain EWMA to fit even better to our
 *    use case.
 *
 * b) delta > W
 *    this means more than a full window has passed since the last update:
 *
 *                |----------- W ---------|
 *                |                       |
 *       load_last_update                 t
 *     ----|------------------------------|---
 *         |                              |
 *         \_________________delta________/
 *
 *    Basically, it means the last load sample has been in effect for more
 *    than W time, and hence we should just use it, and forget everything
 *    before that.
 *
 *    This can be seen as a 'reset condition', occurring when, for whatever
 *    reason, load has not been updated for longer than we expected. (It is
 *    also how avgload is assigned its first value.)
 *
 * The formula for avgload then becomes:
 *
 *  avgload = (delta/W)*load + (W - delta)*avgload/W
 *  avgload = delta*load/W + W*avgload/W - delta*avgload/W
 *  avgload = avgload + delta*load/W - delta*avgload/W
 *
 * So, final form is:
 *
 *  avgload_0 = load
 *  avgload = avgload + delta*load/W - delta*avgload/W,  0<=delta<=W
 *
 * As a confirmation, let's look at the extremes, when delta is 0 (i.e.,
 * what happens if we  update the load twice, at the same time instant?):
 *
 *  avgload = avgload + 0*load/W - 0*avgload/W
 *  avgload = avgload
 *
 * and when delta is W (i.e., what happens if we update at the last
 * possible instant before the window 'expires'?):
 *
 *  avgload = avgload + W*load/W - W*avgload/W
 *  avgload = avgload + load - avgload
 *  avgload = load
 *
 * Which, in both cases, is what we expect.
 */
static void
__update_runq_load(const struct scheduler *ops,
                  struct csched2_runqueue_data *rqd, int change, s_time_t now)
{
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    s_time_t delta, load = rqd->load;
    unsigned int P, W;

    W = prv->load_window_shift;
    P = prv->load_precision_shift;
    now >>= LOADAVG_GRANULARITY_SHIFT;

    /*
     * To avoid using fractions, we shift to left by load_precision_shift,
     * and use the least last load_precision_shift bits as fractional part.
     * Looking back at the formula we want to use, we now have:
     *
     *  P = 2^(load_precision_shift)
     *  P*avgload = P*(avgload + delta*load/W - delta*avgload/W)
     *  P*avgload = P*avgload + delta*load*P/W - delta*P*avgload/W
     *
     * And if we are ok storing and using P*avgload, we can rewrite this as:
     *
     *  P*avgload = avgload'
     *  avgload' = avgload' + delta*P*load/W - delta*avgload'/W
     *
     * Coupled with, of course:
     *
     *  avgload_0' = P*load
     */

    if ( rqd->load_last_update + (1ULL << W)  < now )
    {
        rqd->avgload = load << P;
        rqd->b_avgload = load << P;
    }
    else
    {
        delta = now - rqd->load_last_update;
        if ( unlikely(delta < 0) )
        {
            d2printk("WARNING: %s: Time went backwards? now %"PRI_stime" llu %"PRI_stime"\n",
                     __func__, now, rqd->load_last_update);
            delta = 0;
        }

        /*
         * Note that, if we were to enforce (or check) some relationship
         * between P and W, we may save one shift. E.g., if we are sure
         * that P < W, we could write:
         *
         *  (delta * (load << P)) >> W
         *
         * as:
         *
         *  (delta * load) >> (W - P)
         */
        rqd->avgload = rqd->avgload +
                       ((delta * (load << P)) >> W) -
                       ((delta * rqd->avgload) >> W);
        rqd->b_avgload = rqd->b_avgload +
                         ((delta * (load << P)) >> W) -
                         ((delta * rqd->b_avgload) >> W);
    }
    rqd->load += change;
    rqd->load_last_update = now;

    /* Overflow, capable of making the load look negative, must not occur. */
    ASSERT(rqd->avgload >= 0 && rqd->b_avgload >= 0);

    if ( unlikely(tb_init_done) )
    {
        struct {
            uint64_t rq_avgload, b_avgload;
            unsigned rq_load:16, rq_id:8, shift:8;
        } d;
        d.rq_id = rqd->id;
        d.rq_load = rqd->load;
        d.rq_avgload = rqd->avgload;
        d.b_avgload = rqd->b_avgload;
        d.shift = P;
        __trace_var(TRC_CSCHED2_UPDATE_RUNQ_LOAD, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }
}

static void
__update_svc_load(const struct scheduler *ops,
                  struct csched2_vcpu *svc, int change, s_time_t now)
{
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    s_time_t delta, vcpu_load;
    unsigned int P, W;

    if ( change == -1 )
        vcpu_load = 1;
    else if ( change == 1 )
        vcpu_load = 0;
    else
        vcpu_load = vcpu_runnable(svc->vcpu);

    W = prv->load_window_shift;
    P = prv->load_precision_shift;
    now >>= LOADAVG_GRANULARITY_SHIFT;

    if ( svc->load_last_update + (1ULL << W) < now )
    {
        svc->avgload = vcpu_load << P;
    }
    else
    {
        delta = now - svc->load_last_update;
        if ( unlikely(delta < 0) )
        {
            d2printk("WARNING: %s: Time went backwards? now %"PRI_stime" llu %"PRI_stime"\n",
                     __func__, now, svc->load_last_update);
            delta = 0;
        }

        svc->avgload = svc->avgload +
                       ((delta * (vcpu_load << P)) >> W) -
                       ((delta * svc->avgload) >> W);
    }
    svc->load_last_update = now;

    /* Overflow, capable of making the load look negative, must not occur. */
    ASSERT(svc->avgload >= 0);

    if ( unlikely(tb_init_done) )
    {
        struct {
            uint64_t v_avgload;
            unsigned vcpu:16, dom:16;
            unsigned shift;
        } d;
        d.dom = svc->vcpu->domain->domain_id;
        d.vcpu = svc->vcpu->vcpu_id;
        d.v_avgload = svc->avgload;
        d.shift = P;
        __trace_var(TRC_CSCHED2_UPDATE_VCPU_LOAD, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }
}

static void
update_load(const struct scheduler *ops,
            struct csched2_runqueue_data *rqd,
            struct csched2_vcpu *svc, int change, s_time_t now)
{
    trace_var(TRC_CSCHED2_UPDATE_LOAD, 1, 0,  NULL);

    __update_runq_load(ops, rqd, change, now);
    if ( svc )
        __update_svc_load(ops, svc, change, now);
}

static int
__runq_insert(struct list_head *runq, struct csched2_vcpu *svc)
{
    struct list_head *iter;
    int pos = 0;

    ASSERT(&svc->rqd->runq == runq);
    ASSERT(!is_idle_vcpu(svc->vcpu));
    ASSERT(!svc->vcpu->is_running);
    ASSERT(!(svc->flags & CSFLAG_scheduled));

    list_for_each( iter, runq )
    {
        struct csched2_vcpu * iter_svc = __runq_elem(iter);

        if ( svc->credit > iter_svc->credit )
            break;

        pos++;
    }

    list_add_tail(&svc->runq_elem, iter);

    return pos;
}

static void
runq_insert(const struct scheduler *ops, struct csched2_vcpu *svc)
{
    unsigned int cpu = svc->vcpu->processor;
    struct list_head * runq = &RQD(ops, cpu)->runq;
    int pos = 0;

    ASSERT(spin_is_locked(per_cpu(schedule_data, cpu).schedule_lock));

    ASSERT(!__vcpu_on_runq(svc));
    ASSERT(c2r(ops, cpu) == c2r(ops, svc->vcpu->processor));

    pos = __runq_insert(runq, svc);

    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned vcpu:16, dom:16;
            unsigned pos;
        } d;
        d.dom = svc->vcpu->domain->domain_id;
        d.vcpu = svc->vcpu->vcpu_id;
        d.pos = pos;
        __trace_var(TRC_CSCHED2_RUNQ_POS, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }

    return;
}

static inline void
__runq_remove(struct csched2_vcpu *svc)
{
    ASSERT(__vcpu_on_runq(svc));
    list_del_init(&svc->runq_elem);
}

/*
 * During the soft-affinity step, only actually preempt someone if
 * he does not have soft-affinity with cpu (while we have).
 *
 * BEWARE that this uses cpumask_scratch, trowing away what's in there!
 */
static inline bool_t soft_aff_check_preempt(unsigned int bs, unsigned int cpu)
{
    struct csched2_vcpu * cur = CSCHED2_VCPU(curr_on_cpu(cpu));

    /*
     * If we're doing hard-affinity, always check whether to preempt cur.
     * If we're doing soft-affinity, but cur doesn't have one, check as well.
     */
    if ( bs == BALANCE_HARD_AFFINITY ||
         !has_soft_affinity(cur->vcpu, cur->vcpu->cpu_hard_affinity) )
        return 1;

    /*
     * We're doing soft-affinity, and we know that the current vcpu on cpu
     * has a soft affinity. We now want to know whether cpu itself is in
     * such affinity. In fact, since we now that new (in runq_tickle()) is:
     *  - if cpu is not in cur's soft-affinity, we should indeed check to
     *    see whether new should preempt cur. If that will be the case, that
     *    would be an improvement wrt respecting soft affinity;
     *  - if cpu is in cur's soft-affinity, we leave it alone and (in
     *    runq_tickle()) move on to another cpu. In fact, we don't want to
     *    be too harsh with someone which is running within its soft-affinity.
     *    This is safe because later, if we don't fine anyone else during the
     *    soft-affinity step, we will check cpu for preemption anyway, when
     *    doing hard-affinity.
     */
    affinity_balance_cpumask(cur->vcpu, BALANCE_SOFT_AFFINITY, cpumask_scratch);
    return !cpumask_test_cpu(cpu, cpumask_scratch);
}

void burn_credits(struct csched2_runqueue_data *rqd, struct csched2_vcpu *, s_time_t);

/*
 * Check what processor it is best to 'wake', for picking up a vcpu that has
 * just been put (back) in the runqueue. Logic is as follows:
 *  1. if there are idle processors in the runq, wake one of them;
 *  2. if there aren't idle processor, check the one were the vcpu was
 *     running before to see if we can preempt what's running there now
 *     (and hence doing just one migration);
 *  3. last stand: check all processors and see if the vcpu is in right
 *     of preempting any of the other vcpus running on them (this requires
 *     two migrations, and that's indeed why it is left as the last stand).
 *
 * Note that when we say 'idle processors' what we really mean is (pretty
 * much always) both _idle_ and _not_already_tickled_. In fact, if a
 * processor has been tickled, it will run csched2_schedule() shortly, and
 * pick up some work, so it would be wrong to consider it idle.
 */
static void
runq_tickle(const struct scheduler *ops, struct csched2_vcpu *new, s_time_t now)
{
    int i, ipid = -1;
    s_time_t lowest = (1<<30);
    unsigned int bs, cpu = new->vcpu->processor;
    struct csched2_runqueue_data *rqd = RQD(ops, cpu);
    cpumask_t mask;
    struct csched2_vcpu * cur;

    ASSERT(new->rqd == rqd);

    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned vcpu:16, dom:16;
            unsigned processor, credit;
        } d;
        d.dom = new->vcpu->domain->domain_id;
        d.vcpu = new->vcpu->vcpu_id;
        d.processor = new->vcpu->processor;
        d.credit = new->credit;
        __trace_var(TRC_CSCHED2_TICKLE_NEW, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }

    for_each_affinity_balance_step( bs )
    {
        /*
         * First things first: if we are at the first (soft affinity) step,
         * but new doesn't have a soft affinity, skip this step.
         */
        if ( bs == BALANCE_SOFT_AFFINITY &&
             !has_soft_affinity(new->vcpu, new->vcpu->cpu_hard_affinity) )
            continue;

        affinity_balance_cpumask(new->vcpu, bs, cpumask_scratch);

        /*
         * First of all, consider idle cpus, checking if we can just
         * re-use the pcpu where we were running before.
         *
         * If there are cores where all the siblings are idle, consider
         * them first, honoring whatever the spreading-vs-consolidation
         * SMT policy wants us to do.
         */
        if ( unlikely(sched_smt_power_savings) )
            cpumask_andnot(&mask, &rqd->idle, &rqd->smt_idle);
        else
            cpumask_copy(&mask, &rqd->smt_idle);
        cpumask_and(&mask, &mask, cpumask_scratch);
        i = cpumask_test_or_cycle(cpu, &mask);
        if ( i < nr_cpu_ids )
        {
            SCHED_STAT_CRANK(tickled_idle_cpu);
            ipid = i;
            goto tickle;
        }

        /*
         * If there are no fully idle cores, check all idlers, after
         * having filtered out pcpus that have been tickled but haven't
         * gone through the scheduler yet.
         */
        cpumask_andnot(&mask, &rqd->idle, &rqd->tickled);
        cpumask_and(&mask, &mask, cpumask_scratch);
        i = cpumask_test_or_cycle(cpu, &mask);
        if ( i < nr_cpu_ids )
        {
            SCHED_STAT_CRANK(tickled_idle_cpu);
            ipid = i;
            goto tickle;
        }

        /*
         * Otherwise, look for the non-idle (and non-tickled) processors with
         * the lowest credit, among the ones new is allowed to run on. Again,
         * the cpu were it was running on would be the best candidate.
         */
        cpumask_andnot(&mask, &rqd->active, &rqd->idle);
        cpumask_andnot(&mask, &mask, &rqd->tickled);
        cpumask_and(&mask, &mask, cpumask_scratch);
        if ( cpumask_test_cpu(cpu, &mask) )
        {
            cur = CSCHED2_VCPU(curr_on_cpu(cpu));

            if ( soft_aff_check_preempt(bs, cpu) )
            {
                burn_credits(rqd, cur, now);

                if ( unlikely(tb_init_done) )
                {
                    struct {
                        unsigned vcpu:16, dom:16;
                        unsigned cpu, credit;
                    } d;
                    d.dom = cur->vcpu->domain->domain_id;
                    d.vcpu = cur->vcpu->vcpu_id;
                    d.credit = cur->credit;
                    d.cpu = cpu;
                    __trace_var(TRC_CSCHED2_TICKLE_CHECK, 1,
                                sizeof(d),
                                (unsigned char *)&d);
                }

                if ( cur->credit < new->credit )
                {
                    SCHED_STAT_CRANK(tickled_busy_cpu);
                    ipid = cpu;
                    goto tickle;
                }
            }
        }

        for_each_cpu(i, &mask)
        {
            /* Already looked at this one above */
            if ( i == cpu )
                continue;

            cur = CSCHED2_VCPU(curr_on_cpu(i));
            ASSERT(!is_idle_vcpu(cur->vcpu));

            if ( soft_aff_check_preempt(bs, i) )
            {
                /* Update credits for current to see if we want to preempt. */
                burn_credits(rqd, cur, now);

                if ( unlikely(tb_init_done) )
                {
                    struct {
                        unsigned vcpu:16, dom:16;
                        unsigned cpu, credit;
                    } d;
                    d.dom = cur->vcpu->domain->domain_id;
                    d.vcpu = cur->vcpu->vcpu_id;
                    d.credit = cur->credit;
                    d.cpu = i;
                    __trace_var(TRC_CSCHED2_TICKLE_CHECK, 1,
                                sizeof(d),
                                (unsigned char *)&d);
                }

                if ( cur->credit < lowest )
                {
                    ipid = i;
                    lowest = cur->credit;
                }
            }
        }

        /*
         * Only switch to another processor if the credit difference is
         * greater than the migrate resistance.
         */
        if ( ipid != -1 && lowest + CSCHED2_MIGRATE_RESIST <= new->credit )
        {
            SCHED_STAT_CRANK(tickled_busy_cpu);
            goto tickle;
        }
    }

    SCHED_STAT_CRANK(tickled_no_cpu);
    return;
 tickle:
    BUG_ON(ipid == -1);

    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned cpu:16, pad:16;
        } d;
        d.cpu = ipid; d.pad = 0;
        __trace_var(TRC_CSCHED2_TICKLE, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }
    __cpumask_set_cpu(ipid, &rqd->tickled);
    smt_idle_mask_clear(ipid, &rqd->smt_idle);
    cpu_raise_softirq(ipid, SCHEDULE_SOFTIRQ);

    if ( unlikely(new->tickled_cpu != -1) )
        SCHED_STAT_CRANK(tickled_cpu_overwritten);
    new->tickled_cpu = ipid;
}

/*
 * Credit-related code
 */
static void reset_credit(const struct scheduler *ops, int cpu, s_time_t now,
                         struct csched2_vcpu *snext)
{
    struct csched2_runqueue_data *rqd = RQD(ops, cpu);
    struct list_head *iter;
    int m;

    /*
     * Under normal circumstances, snext->credit should never be less
     * than -CSCHED2_MIN_TIMER.  However, under some circumstances, a
     * vcpu with low credits may be allowed to run long enough that
     * its credits are actually less than -CSCHED2_CREDIT_INIT.
     * (Instances have been observed, for example, where a vcpu with
     * 200us of credit was allowed to run for 11ms, giving it -10.8ms
     * of credit.  Thus it was still negative even after the reset.)
     *
     * If this is the case for snext, we simply want to keep moving
     * everyone up until it is in the black again.  This fair because
     * none of the other vcpus want to run at the moment.
     *
     * Rather than looping, however, we just calculate a multiplier,
     * avoiding an integer division and multiplication in the common
     * case.
     */
    m = 1;
    if ( snext->credit < -CSCHED2_CREDIT_INIT )
        m += (-snext->credit) / CSCHED2_CREDIT_INIT;

    list_for_each( iter, &rqd->svc )
    {
        struct csched2_vcpu * svc;
        int start_credit;

        svc = list_entry(iter, struct csched2_vcpu, rqd_elem);

        ASSERT(!is_idle_vcpu(svc->vcpu));
        ASSERT(svc->rqd == rqd);

        start_credit = svc->credit;

        /* And add INIT * m, avoiding integer multiplication in the
         * common case. */
        if ( likely(m==1) )
            svc->credit += CSCHED2_CREDIT_INIT;
        else
            svc->credit += m * CSCHED2_CREDIT_INIT;

        /* "Clip" credits to max carryover */
        if ( svc->credit > CSCHED2_CREDIT_INIT + CSCHED2_CARRYOVER_MAX )
            svc->credit = CSCHED2_CREDIT_INIT + CSCHED2_CARRYOVER_MAX;

        svc->start_time = now;

        if ( unlikely(tb_init_done) )
        {
            struct {
                unsigned vcpu:16, dom:16;
                unsigned credit_start, credit_end;
                unsigned multiplier;
            } d;
            d.dom = svc->vcpu->domain->domain_id;
            d.vcpu = svc->vcpu->vcpu_id;
            d.credit_start = start_credit;
            d.credit_end = svc->credit;
            d.multiplier = m;
            __trace_var(TRC_CSCHED2_CREDIT_RESET, 1,
                        sizeof(d),
                        (unsigned char *)&d);
        }
    }

    SCHED_STAT_CRANK(credit_reset);

    /* No need to resort runqueue, as everyone's order should be the same. */
}

void burn_credits(struct csched2_runqueue_data *rqd,
                  struct csched2_vcpu *svc, s_time_t now)
{
    s_time_t delta;

    ASSERT(svc == CSCHED2_VCPU(curr_on_cpu(svc->vcpu->processor)));

    if ( unlikely(is_idle_vcpu(svc->vcpu)) )
    {
        ASSERT(svc->credit == CSCHED2_IDLE_CREDIT);
        return;
    }

    delta = now - svc->start_time;

    if ( likely(delta > 0) )
    {
        SCHED_STAT_CRANK(burn_credits_t2c);
        t2c_update(rqd, delta, svc);
        svc->start_time = now;
    }
    else if ( delta < 0 )
    {
        d2printk("WARNING: %s: Time went backwards? now %"PRI_stime" start_time %"PRI_stime"\n",
                 __func__, now, svc->start_time);
    }

    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned vcpu:16, dom:16;
            unsigned credit, cpu;
            int delta;
        } d;
        d.dom = svc->vcpu->domain->domain_id;
        d.vcpu = svc->vcpu->vcpu_id;
        d.credit = svc->credit;
        d.cpu = svc->vcpu->processor;
        d.delta = delta;
        __trace_var(TRC_CSCHED2_CREDIT_BURN, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }
}

/* Find the domain with the highest weight. */
static void update_max_weight(struct csched2_runqueue_data *rqd, int new_weight,
                              int old_weight)
{
    /* Try to avoid brute-force search:
     * - If new_weight is larger, max_weigth <- new_weight
     * - If old_weight != max_weight, someone else is still max_weight
     *   (No action required)
     * - If old_weight == max_weight, brute-force search for max weight
     */
    if ( new_weight > rqd->max_weight )
    {
        rqd->max_weight = new_weight;
        SCHED_STAT_CRANK(upd_max_weight_quick);
    }
    else if ( old_weight == rqd->max_weight )
    {
        struct list_head *iter;
        int max_weight = 1;

        list_for_each( iter, &rqd->svc )
        {
            struct csched2_vcpu * svc = list_entry(iter, struct csched2_vcpu, rqd_elem);

            if ( svc->weight > max_weight )
                max_weight = svc->weight;
        }

        rqd->max_weight = max_weight;
        SCHED_STAT_CRANK(upd_max_weight_full);
    }

    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned rqi:16, max_weight:16;
        } d;
        d.rqi = rqd->id;
        d.max_weight = rqd->max_weight;
        __trace_var(TRC_CSCHED2_RUNQ_MAX_WEIGHT, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }
}

#ifndef NDEBUG
static /*inline*/ void
__csched2_vcpu_check(struct vcpu *vc)
{
    struct csched2_vcpu * const svc = CSCHED2_VCPU(vc);
    struct csched2_dom * const sdom = svc->sdom;

    BUG_ON( svc->vcpu != vc );
    BUG_ON( sdom != CSCHED2_DOM(vc->domain) );
    if ( sdom )
    {
        BUG_ON( is_idle_vcpu(vc) );
        BUG_ON( sdom->dom != vc->domain );
    }
    else
    {
        BUG_ON( !is_idle_vcpu(vc) );
    }
    SCHED_STAT_CRANK(vcpu_check);
}
#define CSCHED2_VCPU_CHECK(_vc)  (__csched2_vcpu_check(_vc))
#else
#define CSCHED2_VCPU_CHECK(_vc)
#endif

static void *
csched2_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
    struct csched2_vcpu *svc;

    /* Allocate per-VCPU info */
    svc = xzalloc(struct csched2_vcpu);
    if ( svc == NULL )
        return NULL;

    INIT_LIST_HEAD(&svc->rqd_elem);
    INIT_LIST_HEAD(&svc->runq_elem);

    svc->sdom = dd;
    svc->vcpu = vc;
    svc->flags = 0U;

    if ( ! is_idle_vcpu(vc) )
    {
        ASSERT(svc->sdom != NULL);
        svc->credit = CSCHED2_CREDIT_INIT;
        svc->weight = svc->sdom->weight;
        svc->tickled_cpu = -1;
        /* Starting load of 50% */
        svc->avgload = 1ULL << (CSCHED2_PRIV(ops)->load_precision_shift - 1);
        svc->load_last_update = NOW() >> LOADAVG_GRANULARITY_SHIFT;
    }
    else
    {
        ASSERT(svc->sdom == NULL);
        svc->tickled_cpu = svc->vcpu->vcpu_id;
        svc->credit = CSCHED2_IDLE_CREDIT;
        svc->weight = 0;
    }

    SCHED_STAT_CRANK(vcpu_alloc);

    return svc;
}

/* Add and remove from runqueue assignment (not active run queue) */
static void
__runq_assign(struct csched2_vcpu *svc, struct csched2_runqueue_data *rqd)
{

    svc->rqd = rqd;
    list_add_tail(&svc->rqd_elem, &svc->rqd->svc);

    update_max_weight(svc->rqd, svc->weight, 0);

    /* Expected new load based on adding this vcpu */
    rqd->b_avgload += svc->avgload;

    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned vcpu:16, dom:16;
            unsigned rqi:16;
        } d;
        d.dom = svc->vcpu->domain->domain_id;
        d.vcpu = svc->vcpu->vcpu_id;
        d.rqi=rqd->id;
        __trace_var(TRC_CSCHED2_RUNQ_ASSIGN, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }

}

static void
runq_assign(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched2_vcpu *svc = vc->sched_priv;

    ASSERT(svc->rqd == NULL);

    __runq_assign(svc, RQD(ops, vc->processor));
}

static void
__runq_deassign(struct csched2_vcpu *svc)
{
    struct csched2_runqueue_data *rqd = svc->rqd;

    ASSERT(!__vcpu_on_runq(svc));
    ASSERT(!(svc->flags & CSFLAG_scheduled));

    list_del_init(&svc->rqd_elem);
    update_max_weight(rqd, 0, svc->weight);

    /* Expected new load based on removing this vcpu */
    rqd->b_avgload = max_t(s_time_t, rqd->b_avgload - svc->avgload, 0);

    svc->rqd = NULL;
}

static void
runq_deassign(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched2_vcpu *svc = vc->sched_priv;

    ASSERT(svc->rqd == RQD(ops, vc->processor));

    __runq_deassign(svc);
}

static void
csched2_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched2_vcpu * const svc = CSCHED2_VCPU(vc);

    ASSERT(!is_idle_vcpu(vc));
    SCHED_STAT_CRANK(vcpu_sleep);

    if ( curr_on_cpu(vc->processor) == vc )
        cpu_raise_softirq(vc->processor, SCHEDULE_SOFTIRQ);
    else if ( __vcpu_on_runq(svc) )
    {
        ASSERT(svc->rqd == RQD(ops, vc->processor));
        update_load(ops, svc->rqd, svc, -1, NOW());
        __runq_remove(svc);
    }
    else if ( svc->flags & CSFLAG_delayed_runq_add )
        __clear_bit(__CSFLAG_delayed_runq_add, &svc->flags);
}

static void
csched2_vcpu_wake(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched2_vcpu * const svc = CSCHED2_VCPU(vc);
    unsigned int cpu = vc->processor;
    s_time_t now;

    ASSERT(spin_is_locked(per_cpu(schedule_data, cpu).schedule_lock));

    ASSERT(!is_idle_vcpu(vc));

    if ( unlikely(curr_on_cpu(cpu) == vc) )
    {
        SCHED_STAT_CRANK(vcpu_wake_running);
        goto out;
    }

    if ( unlikely(__vcpu_on_runq(svc)) )
    {
        SCHED_STAT_CRANK(vcpu_wake_onrunq);
        goto out;
    }

    if ( likely(vcpu_runnable(vc)) )
        SCHED_STAT_CRANK(vcpu_wake_runnable);
    else
        SCHED_STAT_CRANK(vcpu_wake_not_runnable);

    /* If the context hasn't been saved for this vcpu yet, we can't put it on
     * another runqueue.  Instead, we set a flag so that it will be put on the runqueue
     * after the context has been saved. */
    if ( unlikely(svc->flags & CSFLAG_scheduled) )
    {
        __set_bit(__CSFLAG_delayed_runq_add, &svc->flags);
        goto out;
    }

    /* Add into the new runqueue if necessary */
    if ( svc->rqd == NULL )
        runq_assign(ops, vc);
    else
        ASSERT(RQD(ops, vc->processor) == svc->rqd );

    now = NOW();

    update_load(ops, svc->rqd, svc, 1, now);
        
    /* Put the VCPU on the runq */
    runq_insert(ops, svc);
    runq_tickle(ops, svc, now);

out:
    return;
}

static void
csched2_vcpu_yield(const struct scheduler *ops, struct vcpu *v)
{
    struct csched2_vcpu * const svc = CSCHED2_VCPU(v);

    __set_bit(__CSFLAG_vcpu_yield, &svc->flags);
}

static void
csched2_context_saved(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched2_vcpu * const svc = CSCHED2_VCPU(vc);
    spinlock_t *lock = vcpu_schedule_lock_irq(vc);
    s_time_t now = NOW();

    BUG_ON( !is_idle_vcpu(vc) && svc->rqd != RQD(ops, vc->processor));
    ASSERT(is_idle_vcpu(vc) || svc->rqd == RQD(ops, vc->processor));

    /* This vcpu is now eligible to be put on the runqueue again */
    __clear_bit(__CSFLAG_scheduled, &svc->flags);

    /* If someone wants it on the runqueue, put it there. */
    /*
     * NB: We can get rid of CSFLAG_scheduled by checking for
     * vc->is_running and __vcpu_on_runq(svc) here.  However,
     * since we're accessing the flags cacheline anyway,
     * it seems a bit pointless; especially as we have plenty of
     * bits free.
     */
    if ( __test_and_clear_bit(__CSFLAG_delayed_runq_add, &svc->flags)
         && likely(vcpu_runnable(vc)) )
    {
        ASSERT(!__vcpu_on_runq(svc));

        runq_insert(ops, svc);
        runq_tickle(ops, svc, now);
    }
    else if ( !is_idle_vcpu(vc) )
        update_load(ops, svc->rqd, svc, -1, now);

    vcpu_schedule_unlock_irq(lock, vc);
}

#define MAX_LOAD (STIME_MAX)
static int
csched2_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    int i, min_rqi = -1, min_s_rqi = -1, new_cpu;
    struct csched2_vcpu *svc = CSCHED2_VCPU(vc);
    s_time_t min_avgload = MAX_LOAD, min_s_avgload = MAX_LOAD;
    bool_t has_soft;

    ASSERT(!cpumask_empty(&prv->active_queues));

    /* Locking:
     * - Runqueue lock of vc->processor is already locked
     * - Need to grab prv lock to make sure active runqueues don't
     *   change
     * - Need to grab locks for other runqueues while checking
     *   avgload
     * Locking constraint is:
     * - Lock prv before runqueue locks
     * - Trylock between runqueue locks (no ordering)
     *
     * Since one of the runqueue locks is already held, we can't
     * just grab the prv lock.  Instead, we'll have to trylock, and
     * do something else reasonable if we fail.
     */
    ASSERT(spin_is_locked(per_cpu(schedule_data, vc->processor).schedule_lock));

    if ( !read_trylock(&prv->lock) )
    {
        /* We may be here because someone requested us to migrate. */
        __clear_bit(__CSFLAG_runq_migrate_request, &svc->flags);
        new_cpu = get_fallback_cpu(svc);
        /*
         * Tracing of runq and its load won't be accurate, since we could
         * not get the lock, but at least we will output the chosen pcpu.
         */
        goto out;
    }

    /*
     * First check to see if we're here because someone else suggested a place
     * for us to move.
     */
    if ( __test_and_clear_bit(__CSFLAG_runq_migrate_request, &svc->flags) )
    {
        if ( unlikely(svc->migrate_rqd->id < 0) )
        {
            printk(XENLOG_WARNING "%s: target runqueue disappeared!\n",
                   __func__);
        }
        else
        {
            /*
             * If we've been asked to move to migrate_rqd, we should just do
             * that, which we actually do by returning one cpu from that runq.
             * There is no need to take care of soft affinity, as that will
             * happen in runq_tickle().
             */
            cpumask_and(cpumask_scratch, vc->cpu_hard_affinity,
                        &svc->migrate_rqd->active);
            new_cpu = cpumask_any(cpumask_scratch);
            if ( new_cpu < nr_cpu_ids )
                goto out_up;
        }
        /* Fall-through to normal cpu pick */
    }

    has_soft = has_soft_affinity(vc, vc->cpu_hard_affinity);
    if ( has_soft )
        affinity_balance_cpumask(vc, BALANCE_SOFT_AFFINITY, cpumask_scratch);

    /*
     * What we want is:
     *  - if we have soft affinity, the runqueue with the lowest average
     *    load, among the ones that contain cpus in our soft affinity; this
     *    represents the best runq on which we would want to run.
     *  - the runqueue with the lowest average load among the ones that
     *    contains cpus in our hard affinity; this represent the best runq
     *    on which we can run.
     *
     * Find both runqueues in one pass.
     */
    for_each_cpu(i, &prv->active_queues)
    {
        struct csched2_runqueue_data *rqd;
        s_time_t rqd_avgload = MAX_LOAD;

        rqd = prv->rqd + i;

        /*
         * If checking a different runqueue, grab the lock, check hard
         * affinity, read the avg, and then release the lock.
         *
         * If on our own runqueue, don't grab or release the lock;
         * but subtract our own load from the runqueue load to simulate
         * impartiality.
         *
         * Note that, if svc's hard affinity has changed, this is the
         * first time when we see such change, so it is indeed possible
         * that none of the cpus in svc's current runqueue is in our
         * (new) hard affinity!
         */
        if ( rqd == svc->rqd )
        {
            if ( cpumask_intersects(vc->cpu_hard_affinity, &rqd->active) )
                rqd_avgload = max_t(s_time_t, rqd->b_avgload - svc->avgload, 0);
        }
        else if ( spin_trylock(&rqd->lock) )
        {
            if ( cpumask_intersects(vc->cpu_hard_affinity, &rqd->active) )
                rqd_avgload = rqd->b_avgload;

            spin_unlock(&rqd->lock);
        }

        if ( has_soft &&
             rqd_avgload < min_s_avgload &&
             cpumask_intersects(cpumask_scratch, &rqd->active) )
        {
            min_s_avgload = rqd_avgload;
            min_s_rqi = i;
        }
        if ( rqd_avgload < min_avgload )
        {
            min_avgload = rqd_avgload;
            min_rqi = i;
        }
    }

    if ( has_soft && min_s_rqi != -1 )
    {
        /*
         * We have soft affinity, and we have a candidate runq, so go for it.
         *
         * Note that, since has_soft is true, cpumask_scratch holds the proper
         * soft-affinity mask.
         */
        cpumask_and(cpumask_scratch, cpumask_scratch,
                    &prv->rqd[min_s_rqi].active);
    }
    else if ( min_rqi != -1 )
    {
        /*
         * Either we don't have soft affinity, or we do, but we did not find
         * any suitable runq. But we did find one when considering hard
         * affinity, so go for it.
         */
        cpumask_and(cpumask_scratch, vc->cpu_hard_affinity,
                    &prv->rqd[min_rqi].active);
    }
    else
    {
        /*
         * We didn't find anyone at all (most likely because of spinlock
         * contention).
         */
        new_cpu = get_fallback_cpu(svc);
        min_rqi = c2r(ops, new_cpu);
        min_avgload = prv->rqd[min_rqi].b_avgload;
        goto out_up;
    }

    cpumask_and(cpumask_scratch, vc->cpu_hard_affinity,
                &prv->rqd[min_rqi].active);
    new_cpu = cpumask_any(cpumask_scratch);
    BUG_ON(new_cpu >= nr_cpu_ids);

 out_up:
    read_unlock(&prv->lock);
 out:
    if ( unlikely(tb_init_done) )
    {
        struct {
            uint64_t b_avgload;
            unsigned vcpu:16, dom:16;
            unsigned rq_id:16, new_cpu:16;
        } d;
        d.dom = vc->domain->domain_id;
        d.vcpu = vc->vcpu_id;
        d.rq_id = min_rqi;
        d.b_avgload = min_avgload;
        d.new_cpu = new_cpu;
        __trace_var(TRC_CSCHED2_PICKED_CPU, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }

    return new_cpu;
}

/* Working state of the load-balancing algorithm. */
typedef struct {
    /* NB: Modified by consider(). */
    s_time_t load_delta;
    struct csched2_vcpu * best_push_svc, *best_pull_svc;
    /* NB: Read by consider() (and the various consider_foo() functions). */
    struct csched2_runqueue_data *lrqd;
    struct csched2_runqueue_data *orqd;
    bool_t push_has_soft_aff, pull_has_soft_aff;
    s_time_t push_soft_aff_load, pull_soft_aff_load;
} balance_state_t;

static inline s_time_t consider_load(balance_state_t *st,
                                     struct csched2_vcpu *push_svc,
                                     struct csched2_vcpu *pull_svc)
{
    s_time_t l_load, o_load, delta;

    l_load = st->lrqd->b_avgload;
    o_load = st->orqd->b_avgload;
    if ( push_svc )
    {
        /* What happens to the load on both if we push? */
        l_load -= push_svc->avgload;
        o_load += push_svc->avgload;
    }
    if ( pull_svc )
    {
        /* What happens to the load on both if we pull? */
        l_load += pull_svc->avgload;
        o_load -= pull_svc->avgload;
    }

    delta = l_load - o_load;
    if ( delta < 0 )
        delta = -delta;

    return delta;
}

/*
 * Load balancing and soft-affinity.
 *
 * When trying to figure out whether or not it's best to move a vcpu from
 * one runqueue to another, we must keep soft-affinity in mind. Intuitively
 * we would want to know the following:
 *  - 'how much' affinity does the vcpu have with its current runq?
 *  - 'how much' affinity will it have with its new runq?
 *
 * But we certainly need to be more precise about how much it is that 'how
 * much'! Let's start with some definitions:
 *
 *  - let v be a vcpu, running in runq I, with soft-affinity to vi
 *    pcpus of runq I, and soft affinity with vj pcpus of runq J;
 *  - let k be another vcpu, running in runq J, with soft-affinity to kj
 *    pcpus of runq J, and with ki pcpus of runq I;
 *  - let runq I have Ci pcpus, and runq J Cj pcpus;
 *  - let vcpu v have an average load of lv, and k an average load of lk;
 *  - let runq I have an average load of Li, and J an average load of Lj.
 *
 * We also define the following::
 *
 *  - lvi = lv * (vi / Ci)  as the 'perceived load' of v, when running
 *                          in runq i;
 *  - lvj = lv * (vj / Cj)  as the 'perceived load' of v, it running
 *                          in runq j;
 *  - the same for k, mutatis mutandis.
 *
 * Idea is that vi/Ci (i.e., the ratio of the number of cpus of a runq that
 * a vcpu has soft-affinity with, over the total number of cpus of the runq
 * itself) can be seen as the 'degree of soft-affinity' of v to runq I (and
 * vj/Cj the one of v to J). In other words, we define the degree of soft
 * affinity of a vcpu to a runq as what fraction of pcpus of the runq itself
 * the vcpu has soft-affinity with. Then, we multiply this 'degree of
 * soft-affinity' by the vcpu load, and call the result the 'perceived load'.
 *
 * Basically, if a soft-affinity is defined, the work done by a vcpu on a
 * runq to which it has higher degree of soft-affinity, is considered
 * 'lighter' than the same work done by the same vcpu on a runq to which it
 * has smaller degree of soft-affinity (degree of soft affinity is <= 1). In
 * fact, if soft-affinity is used to achieve NUMA-aware scheduling, the higher
 * the degree of soft-affinity of the vcpu to a runq, the greater the probability
 * of accessing local memory, when running on such runq. And that is certainly\
 * 'lighter' than having to fetch memory from remote NUMA nodes.
 *
 * SoXX, evaluating pushing v from I to J would mean removing (from I) a
 * perceived load of lv*(vi/Ci) and adding (to J) a perceived load of
 * lv*(vj/Cj), which we (looking at things from the point of view of I,
 * which is what balance_load() does) can call D_push:
 *
 *  - D_push = -lv * (vi / Ci) + lv * (vj / Cj) =
 *           = lv * (vj/Cj - vi/Ci)
 *
 * On the other hand, pulling k from J to I would entail a D_pull:
 *
 *  - D_pull = lk * (ki / Ci) - lk * (kj / Cj) =
 *           = lk * (ki/Ci - kj/Cj)
 *
 * Note that if v (k) has soft-afinity with all the cpus of both I and J,
 * D_push (D_pull) will be 0, and the same is true in case it has no soft
 * affinity at all with any of the cpus of I and J. Note also that both
 * D_push and D_pull can be positive or negative (there's no abs() around
 * in this case!) depending on the relationship between the degrees of soft
 * affinity of the vcpu to I and J.
 *
 * If there is no soft-affinity, load_balance() (actually, consider()) acts
 * as follows:
 *
 *  - D = abs(Li - Lj)
 *  - consider pushing v from I to J:
 *     - D' = abs(Li - lv - (Lj + lv))   (from now, abs(x) == |x|)
 *     - if (D' < D) { push }
 *  - consider pulling k from J to I:
 *     - D' = |Li + lk - (Lj - lk)|
 *     - if (D' < D) { pull }
 *  - consider both push and pull:
 *     - D' = |Li - lv + lk - (Lj + lv - lk)|
 *     - if (D' < D) { push; pull }
 *
 * In order to make soft-affinity part of the process, we use D_push and
 * D_pull, so that, the final behavior will look like this:
 *
 *  - D = abs(Li - Lj)
 *  - consider pushing v from I to J:
 *     - D' = |Li - lv - (Lj + lv)|
 *     - D_push = lv * (vj/Cj - vi/Ci)
 *     - if (D' + D_push < D) { push }
 *  - consider pulling k from J to I:
 *     - D' = |Li + lk - (Lj - lk)|
 *       D_pull = lk * (ki/Ci - kj/Cj)
 *     - if (D' < D) { pull }
 *  - consider both push and pull:
 *     - D' = |Li - lv + lk - (Lj + lv - lk)|
 *     - D_push = lv * (vj/Cj - vi/Ci)
 *       D_pull = lk * (ki/Ci - kj/Cj)
 *     - if (D' + D_push + D_pull < D) { push; pull }
 *
 * So, for instance, the complete formula, in case of a push, with soft
 * affinity being considered looks like this:
 *
 *  - D'' = D' + D_push =
 *        = |Li - lv - (Lj + lv)| + lv*(vj/Cj - vi/Ci)
 *
 * which highlights how soft-affinity being considered acts as a *modifier*
 * of the "normal" results obtained by just using the actual vcpus loads.
 * This approach is modular, in the sense that it only takes implementing
 * another function that returns another modifier, to make the load balancer
 * consider some other factor or characteristics of the vcpus.
 *
 * Finally there is the scope for actually using a scaling factor, to limit
 * the influence that soft-affinity will actually have on baseline results
 * from consider_load(). Basically, that means that instead of D_push and/or
 * D_pull, we'll be adding D_push/S and/or D_pull/S (with S the scaling
 * factor). Check prep_soft_aff_load() for details on this.
 */

static inline s_time_t consider_soft_affinity(balance_state_t *st,
                                              struct csched2_vcpu *push_svc,
                                              struct csched2_vcpu *pull_svc)
{
    s_time_t push_load = push_svc ? st->push_soft_aff_load : 0;
    s_time_t pull_load = pull_svc ? st->pull_soft_aff_load : 0;

    /*
     * This is potentially called many times, and a few of them, on the same
     * vcpu(s). Therefore, all the expensive operations (e.g., the cpumask
     * manipulations) are done in balance_load(), in the attempt of amortizing
     * the cost, and all that remains to be done here is return the proper
     * combination of results.
     */
    return push_load + pull_load;
}

static void consider(balance_state_t *st,
                     struct csched2_vcpu *push_svc,
                     struct csched2_vcpu *pull_svc)
{
    s_time_t delta, delta_soft_aff;

    /*
     * The idea here is:
     *  - consider_load() is the basic step. It compares what would happen
     *    if the requested combination of pushes and pulls is done, using
     *    the actual load of the vcpus being considered;
     *  - subsequent steps return a *modifier* for the result obtained
     *    above, which is then applied, before drawing conclusions.
     */
    delta = consider_load(st, push_svc, pull_svc);
    delta_soft_aff = consider_soft_affinity(st, push_svc, pull_svc);

    delta += delta_soft_aff;

    if ( delta < st->load_delta )
    {
        st->load_delta = delta;
        st->best_push_svc = push_svc;
        st->best_pull_svc = pull_svc;
    }
}


static void migrate(const struct scheduler *ops,
                    struct csched2_vcpu *svc, 
                    struct csched2_runqueue_data *trqd, 
                    s_time_t now)
{
    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned vcpu:16, dom:16;
            unsigned rqi:16, trqi:16;
        } d;
        d.dom = svc->vcpu->domain->domain_id;
        d.vcpu = svc->vcpu->vcpu_id;
        d.rqi = svc->rqd->id;
        d.trqi = trqd->id;
        __trace_var(TRC_CSCHED2_MIGRATE, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }

    if ( svc->flags & CSFLAG_scheduled )
    {
        /* It's running; mark it to migrate. */
        svc->migrate_rqd = trqd;
        __set_bit(_VPF_migrating, &svc->vcpu->pause_flags);
        __set_bit(__CSFLAG_runq_migrate_request, &svc->flags);
        cpu_raise_softirq(svc->vcpu->processor, SCHEDULE_SOFTIRQ);
        SCHED_STAT_CRANK(migrate_requested);
    }
    else
    {
        int on_runq = 0;
        /* It's not running; just move it */
        if ( __vcpu_on_runq(svc) )
        {
            __runq_remove(svc);
            update_load(ops, svc->rqd, NULL, -1, now);
            on_runq = 1;
        }
        __runq_deassign(svc);

        cpumask_and(cpumask_scratch, svc->vcpu->cpu_hard_affinity,
                    &trqd->active);
        svc->vcpu->processor = cpumask_any(cpumask_scratch);
        ASSERT(svc->vcpu->processor < nr_cpu_ids);

        __runq_assign(svc, trqd);
        if ( on_runq )
        {
            update_load(ops, svc->rqd, NULL, 1, now);
            runq_insert(ops, svc);
            runq_tickle(ops, svc, now);
            SCHED_STAT_CRANK(migrate_on_runq);
        }
        else
            SCHED_STAT_CRANK(migrate_no_runq);
    }
}

/*
 * It makes sense considering migrating svc to rqd, if:
 *  - svc is not already flagged to migrate,
 *  - if svc is allowed to run on at least one of the pcpus of rqd.
 */
static bool_t vcpu_is_migrateable(struct csched2_vcpu *svc,
                                  struct csched2_runqueue_data *rqd)
{
    return !(svc->flags & CSFLAG_runq_migrate_request) &&
           cpumask_intersects(svc->vcpu->cpu_hard_affinity, &rqd->active);
}

/*
 * Compute the load modifiers to be used in consider() and store them in st.
 *
 * aff_shift gives the chance to change how much soft-affinity should affect
 * load balancing, i.e., the scaling factor introduced above.
 *
 * It's a shift width and using for it the same value as prec_shift will make
 * the scaling factor 1. Using smaller values, will exponentially decrease
 * the impact of soft-affinity. E.g., aff_shift=prec_shift-1 would make the
 * scaling factor be 2 (S=2 in the formulas above), which in turn means
 * D_push and D_pull will be halved, and so on and so forth.
 */
static inline s_time_t prep_soft_aff_load(const struct csched2_vcpu *svc,
                                          const balance_state_t *st,
                                          unsigned int nr_cpus_lrq,
                                          unsigned int nr_cpus_orq,
                                          unsigned int prec_shift,
                                          unsigned int aff_shift,
                                          bool_t is_push)
{
    s_time_t l_weight, o_weight;
    s_time_t load, weight;
    cpumask_t aff_mask;

    ASSERT(aff_shift <= prec_shift);

    /* Compute the degree of soft-affinity of svc to both lrq and orq. */
    affinity_balance_cpumask(svc->vcpu, BALANCE_SOFT_AFFINITY, &aff_mask);
    cpumask_and(cpumask_scratch, &aff_mask, &st->lrqd->active);
    l_weight = (cpumask_weight(cpumask_scratch) << aff_shift) / nr_cpus_lrq;
    cpumask_and(cpumask_scratch, &aff_mask, &st->orqd->active);
    o_weight = (cpumask_weight(cpumask_scratch) << aff_shift) / nr_cpus_orq;

    if ( l_weight >= o_weight )
    {
        weight = l_weight - o_weight;
        load = 1;
    }
    else
    {
        weight = o_weight - l_weight;
        load = -1;
    }

    if ( !is_push )
        weight = -weight;

    load *= (svc->avgload * weight) >> prec_shift;

    /*
     * Remember that what we are after is actually a modifier. So, for
     * instance, let our vcpu be v, with load 30%, let's consider pushing
     * it from runq I, to which it has a degree of soft-affinity of 3/8, to
     * J, to which it has a degree of soft-affinity of 6/8, and let's say
     * load of I is 40% and load of J is 15%
     *
     * Plain load calculations (i.e., no soft affinity involved) are as
     * follows:
     *  - D = abs(40 - 15) = 25
     *  - consider_load:
     *    - D' = abs(40 - 30 - (15 + 30)) = abs(10 - 45) = 35
     *  - D' > D ==> don't push
     *
     * And this indeed will be the result returned by consider_load(). Now
     * we need D_push (let's assume a scaling factor of 1). Following the
     * code above:
     *
     *  - l_weight = 3/8 = 0.375
     *  - o_weight = 6/8 = 0.75
     *  - weight = 0.75 - 0.375 = 0.375, load = -1
     *  - load = -1 * 30 * 0.375 = -11.25
     *
     * which, once back in consider() would mean:
     *  - D = 25
     *  - D' = 35
     *  - D_push = -11.25
     *  - D' + D_push = 35 - 11.25 = 23.75
     *  - D ' + D_push < D ==> *push*
     *
     * which means considering soft-affinity changed the original load
     * balancer decision, and seems to makes sense, considering that we'd be
     * moving v from a runq where it only has affinity with 3 vcpus (out of
     * 8) to one where it has twice as much of that.
     */
    ASSERT(load >= 0 ? svc->avgload >= load : svc->avgload >= -load);
    return load;
}

static void balance_load(const struct scheduler *ops, int cpu, s_time_t now)
{
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    int i, max_delta_rqi = -1;
    struct list_head *push_iter, *pull_iter;
    unsigned int nr_cpus_lrq, nr_cpus_orq;
    bool_t inner_loop_done_once = 0;

    balance_state_t st = { .best_push_svc = NULL, .best_pull_svc = NULL };

    /*
     * Basic algorithm: Push, pull, or swap.
     * - Find the runqueue with the furthest load distance
     * - Find a pair that makes the difference the least (where one
     * on either side may be empty).
     */

    ASSERT(spin_is_locked(per_cpu(schedule_data, cpu).schedule_lock));
    st.lrqd = RQD(ops, cpu);

    __update_runq_load(ops, st.lrqd, 0, now);

retry:
    if ( !read_trylock(&prv->lock) )
        return;

    st.load_delta = 0;

    for_each_cpu(i, &prv->active_queues)
    {
        s_time_t delta;
        
        st.orqd = prv->rqd + i;

        if ( st.orqd == st.lrqd
             || !spin_trylock(&st.orqd->lock) )
            continue;

        __update_runq_load(ops, st.orqd, 0, now);
    
        delta = st.lrqd->b_avgload - st.orqd->b_avgload;
        if ( delta < 0 )
            delta = -delta;

        if ( delta > st.load_delta )
        {
            st.load_delta = delta;
            max_delta_rqi = i;
        }

        spin_unlock(&st.orqd->lock);
    }

    /* Minimize holding the private scheduler lock. */
    read_unlock(&prv->lock);
    if ( max_delta_rqi == -1 )
        goto out;

    {
        s_time_t load_max;
        int cpus_max;

        load_max = st.lrqd->b_avgload;
        if ( st.orqd->b_avgload > load_max )
            load_max = st.orqd->b_avgload;

        cpus_max = nr_cpus_lrq = cpumask_weight(&st.lrqd->active);
        nr_cpus_orq = cpumask_weight(&st.orqd->active);
        cpus_max = nr_cpus_orq > cpus_max ? nr_cpus_orq : cpus_max;

        if ( unlikely(tb_init_done) )
        {
            struct {
                unsigned lrq_id:16, orq_id:16;
                unsigned load_delta;
            } d;
            d.lrq_id = st.lrqd->id;
            d.orq_id = st.orqd->id;
            d.load_delta = st.load_delta;
            __trace_var(TRC_CSCHED2_LOAD_CHECK, 1,
                        sizeof(d),
                        (unsigned char *)&d);
        }

        /*
         * If we're under 100% capacaty, only shift if load difference
         * is > 1.  otherwise, shift if under 12.5%
         */
        if ( load_max < ((s_time_t)cpus_max << prv->load_precision_shift) )
        {
            if ( st.load_delta < (1ULL << (prv->load_precision_shift +
                                           opt_underload_balance_tolerance)) )
                 goto out;
        }
        else
            if ( st.load_delta < (1ULL << (prv->load_precision_shift +
                                           opt_overload_balance_tolerance)) )
                goto out;
    }
             
    /* Try to grab the other runqueue lock; if it's been taken in the
     * meantime, try the process over again.  This can't deadlock
     * because if it doesn't get any other rqd locks, it will simply
     * give up and return. */
    st.orqd = prv->rqd + max_delta_rqi;
    if ( !spin_trylock(&st.orqd->lock) )
        goto retry;

    /* Make sure the runqueue hasn't been deactivated since we released prv->lock */
    if ( unlikely(st.orqd->id < 0) )
        goto out_up;

    if ( unlikely(tb_init_done) )
    {
        struct {
            uint64_t lb_avgload, ob_avgload;
            unsigned lrq_id:16, orq_id:16;
        } d;
        d.lrq_id = st.lrqd->id;
        d.lb_avgload = st.lrqd->b_avgload;
        d.orq_id = st.orqd->id;
        d.ob_avgload = st.orqd->b_avgload;
        __trace_var(TRC_CSCHED2_LOAD_BALANCE, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }

    /* Look for "swap" which gives the best load average
     * FIXME: O(n^2)! */

    /* Reuse load delta (as we're trying to minimize it) */
    st.load_delta = st.load_delta;

    list_for_each( push_iter, &st.lrqd->svc )
    {
        struct csched2_vcpu * push_svc = list_entry(push_iter, struct csched2_vcpu, rqd_elem);

        __update_svc_load(ops, push_svc, 0, now);

        if ( !vcpu_is_migrateable(push_svc, st.orqd) )
            continue;

        /*
         * Soft affinity related code being present here, is the price we pay
         * to performance. In fact, ideally, this would all go into
         * consider_soft_aff(). However, that would mean doing all it's done
         * here (i.e., cpumask stuff in has_soft_affinity() and both cpumask
         * stuff and medium heavy math in prep_soft_aff_load()), for the *same*
         * vcpu, 1+nr_vcpus_in_orq times!
         *
         * So, yes, it's less beautiful and modular than it could have been,
         * but this is an hot path, and we can't afford being that beautiful
         * and modular.
         */
        if ( has_soft_affinity(push_svc->vcpu,
                               push_svc->vcpu->cpu_hard_affinity) )
        {
            st.push_has_soft_aff = 1;
            st.push_soft_aff_load = prep_soft_aff_load(push_svc, &st,
                                                       nr_cpus_lrq,
                                                       nr_cpus_orq,
                                                       prv->load_precision_shift,
                                                       prv->load_precision_shift - 0,
                                                       1 /* is a push */);
        }
        else
        {
            st.push_has_soft_aff = 0;
            st.push_soft_aff_load = 0;
        }

        /* Consider push only. */
        consider(&st, push_svc, NULL);

        list_for_each( pull_iter, &st.orqd->svc )
        {
            struct csched2_vcpu * pull_svc = list_entry(pull_iter, struct csched2_vcpu, rqd_elem);
            
            if ( !inner_loop_done_once )
                __update_svc_load(ops, pull_svc, 0, now);

            if ( !vcpu_is_migrateable(pull_svc, st.lrqd) )
                continue;

            /*
             * Same argument as above about modularity. For pulls, it would
             * be a little less of a problem having stuff done in
             * consider_soft_aff(), as we'd be "only" doing the same ops on
             * the same vcpus twice, but:
             *  - doing them twice is still worse than doing them once;
             *  - since push side is like this, it's better to be consistent.
             */
            if ( has_soft_affinity(pull_svc->vcpu,
                                   pull_svc->vcpu->cpu_hard_affinity) )
            {
                st.pull_has_soft_aff = 1;
                st.pull_soft_aff_load = prep_soft_aff_load(pull_svc, &st,
                                                           nr_cpus_lrq,
                                                           nr_cpus_orq,
                                                           prv->load_precision_shift,
                                                           prv->load_precision_shift - 0,
                                                           0 /* is a pull */);
            }
            else
            {
                st.pull_has_soft_aff = 0;
                st.pull_soft_aff_load = 0;
            }

            /* Consider pull only. */
            if ( !inner_loop_done_once )
                consider(&st, NULL, pull_svc);

            /* Consider both push and pull. */
            consider(&st, push_svc, pull_svc);
        }
        inner_loop_done_once = 1;
    }

    /* OK, now we have some candidates; do the moving */
    if ( st.best_push_svc )
        migrate(ops, st.best_push_svc, st.orqd, now);
    if ( st.best_pull_svc )
        migrate(ops, st.best_pull_svc, st.lrqd, now);

 out_up:
    spin_unlock(&st.orqd->lock);
 out:
    return;
}

static void
csched2_vcpu_migrate(
    const struct scheduler *ops, struct vcpu *vc, unsigned int new_cpu)
{
    struct csched2_vcpu * const svc = CSCHED2_VCPU(vc);
    struct csched2_runqueue_data *trqd;

    /* Check if new_cpu is valid */
    ASSERT(cpumask_test_cpu(new_cpu, &CSCHED2_PRIV(ops)->initialized));
    ASSERT(cpumask_test_cpu(new_cpu, vc->cpu_hard_affinity));

    trqd = RQD(ops, new_cpu);

    /*
     * Do the actual movement toward new_cpu, and update vc->processor.
     * If we are changing runqueue, migrate() takes care of everything.
     * If we are not changing runqueue, we need to update vc->processor
     * here. In fact, if, for instance, we are here because the vcpu's
     * hard affinity changed, we don't want to risk leaving vc->processor
     * pointing to a pcpu where we can't run any longer.
     */
    if ( trqd != svc->rqd )
        migrate(ops, svc, trqd, NOW());
    else
        vc->processor = new_cpu;
}

static int
csched2_dom_cntl(
    const struct scheduler *ops,
    struct domain *d,
    struct xen_domctl_scheduler_op *op)
{
    struct csched2_dom * const sdom = CSCHED2_DOM(d);
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    unsigned long flags;
    int rc = 0;

    /*
     * Locking:
     *  - we must take the private lock for accessing the weights of the
     *    vcpus of d,
     *  - in the putinfo case, we also need the runqueue lock(s), for
     *    updating the max waight of the runqueue(s).
     */
    switch ( op->cmd )
    {
    case XEN_DOMCTL_SCHEDOP_getinfo:
        read_lock_irqsave(&prv->lock, flags);
        op->u.credit2.weight = sdom->weight;
        read_unlock_irqrestore(&prv->lock, flags);
        break;
    case XEN_DOMCTL_SCHEDOP_putinfo:
        if ( op->u.credit2.weight != 0 )
        {
            struct vcpu *v;
            int old_weight;

            write_lock_irqsave(&prv->lock, flags);

            old_weight = sdom->weight;

            sdom->weight = op->u.credit2.weight;

            /* Update weights for vcpus, and max_weight for runqueues on which they reside */
            for_each_vcpu ( d, v )
            {
                struct csched2_vcpu *svc = CSCHED2_VCPU(v);
                spinlock_t *lock = vcpu_schedule_lock(svc->vcpu);

                ASSERT(svc->rqd == RQD(ops, svc->vcpu->processor));

                svc->weight = sdom->weight;
                update_max_weight(svc->rqd, svc->weight, old_weight);

                vcpu_schedule_unlock(lock, svc->vcpu);
            }

            write_unlock_irqrestore(&prv->lock, flags);
        }
        break;
    default:
        rc = -EINVAL;
        break;
    }


    return rc;
}

static int csched2_sys_cntl(const struct scheduler *ops,
                            struct xen_sysctl_scheduler_op *sc)
{
    xen_sysctl_credit2_schedule_t *params = &sc->u.sched_credit2;
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    unsigned long flags;

    switch (sc->cmd )
    {
    case XEN_SYSCTL_SCHEDOP_putinfo:
        if ( params->ratelimit_us &&
             (params->ratelimit_us > XEN_SYSCTL_SCHED_RATELIMIT_MAX ||
              params->ratelimit_us < XEN_SYSCTL_SCHED_RATELIMIT_MIN ))
            return -EINVAL;

        write_lock_irqsave(&prv->lock, flags);
        if ( !prv->ratelimit_us && params->ratelimit_us )
            printk(XENLOG_INFO "Enabling context switch rate limiting\n");
        else if ( prv->ratelimit_us && !params->ratelimit_us )
            printk(XENLOG_INFO "Disabling context switch rate limiting\n");
        prv->ratelimit_us = params->ratelimit_us;
        write_unlock_irqrestore(&prv->lock, flags);

    /* FALLTHRU */
    case XEN_SYSCTL_SCHEDOP_getinfo:
        params->ratelimit_us = prv->ratelimit_us;
        break;
    }

    return 0;
}

static void *
csched2_alloc_domdata(const struct scheduler *ops, struct domain *dom)
{
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    struct csched2_dom *sdom;
    unsigned long flags;

    sdom = xzalloc(struct csched2_dom);
    if ( sdom == NULL )
        return NULL;

    /* Initialize credit and weight */
    INIT_LIST_HEAD(&sdom->sdom_elem);
    sdom->dom = dom;
    sdom->weight = CSCHED2_DEFAULT_WEIGHT;
    sdom->nr_vcpus = 0;

    write_lock_irqsave(&prv->lock, flags);

    list_add_tail(&sdom->sdom_elem, &CSCHED2_PRIV(ops)->sdom);

    write_unlock_irqrestore(&prv->lock, flags);

    return (void *)sdom;
}

static int
csched2_dom_init(const struct scheduler *ops, struct domain *dom)
{
    struct csched2_dom *sdom;

    if ( is_idle_domain(dom) )
        return 0;

    sdom = csched2_alloc_domdata(ops, dom);
    if ( sdom == NULL )
        return -ENOMEM;

    dom->sched_priv = sdom;

    return 0;
}

static void
csched2_free_domdata(const struct scheduler *ops, void *data)
{
    unsigned long flags;
    struct csched2_dom *sdom = data;
    struct csched2_private *prv = CSCHED2_PRIV(ops);

    write_lock_irqsave(&prv->lock, flags);

    list_del_init(&sdom->sdom_elem);

    write_unlock_irqrestore(&prv->lock, flags);

    xfree(data);
}

static void
csched2_dom_destroy(const struct scheduler *ops, struct domain *dom)
{
    ASSERT(CSCHED2_DOM(dom)->nr_vcpus == 0);

    csched2_free_domdata(ops, CSCHED2_DOM(dom));
}

static void
csched2_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched2_vcpu *svc = vc->sched_priv;
    struct csched2_dom * const sdom = svc->sdom;
    spinlock_t *lock;

    ASSERT(!is_idle_vcpu(vc));
    ASSERT(list_empty(&svc->runq_elem));

    /* csched2_cpu_pick() expects the pcpu lock to be held */
    lock = vcpu_schedule_lock_irq(vc);

    vc->processor = csched2_cpu_pick(ops, vc);

    spin_unlock_irq(lock);

    lock = vcpu_schedule_lock_irq(vc);

    /* Add vcpu to runqueue of initial processor */
    runq_assign(ops, vc);

    vcpu_schedule_unlock_irq(lock, vc);

    sdom->nr_vcpus++;

    SCHED_STAT_CRANK(vcpu_insert);

    CSCHED2_VCPU_CHECK(vc);
}

static void
csched2_free_vdata(const struct scheduler *ops, void *priv)
{
    struct csched2_vcpu *svc = priv;

    xfree(svc);
}

static void
csched2_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched2_vcpu * const svc = CSCHED2_VCPU(vc);
    spinlock_t *lock;

    ASSERT(!is_idle_vcpu(vc));
    ASSERT(list_empty(&svc->runq_elem));

    SCHED_STAT_CRANK(vcpu_remove);

    /* Remove from runqueue */
    lock = vcpu_schedule_lock_irq(vc);

    runq_deassign(ops, vc);

    vcpu_schedule_unlock_irq(lock, vc);

    svc->sdom->nr_vcpus--;
}

/* How long should we let this vcpu run for? */
static s_time_t
csched2_runtime(const struct scheduler *ops, int cpu,
                struct csched2_vcpu *snext, s_time_t now)
{
    s_time_t time, min_time;
    int rt_credit; /* Proposed runtime measured in credits */
    struct csched2_runqueue_data *rqd = RQD(ops, cpu);
    struct list_head *runq = &rqd->runq;
    struct csched2_private *prv = CSCHED2_PRIV(ops);

    /*
     * If we're idle, just stay so. Others (or external events)
     * will poke us when necessary.
     */
    if ( is_idle_vcpu(snext->vcpu) )
        return -1;

    /* General algorithm:
     * 1) Run until snext's credit will be 0
     * 2) But if someone is waiting, run until snext's credit is equal
     * to his
     * 3) But never run longer than MAX_TIMER or shorter than MIN_TIMER or
     * the ratelimit time.
     */

    /* Calculate mintime */
    min_time = CSCHED2_MIN_TIMER;
    if ( prv->ratelimit_us )
    {
        s_time_t ratelimit_min = MICROSECS(prv->ratelimit_us);
        if ( snext->vcpu->is_running )
            ratelimit_min = snext->vcpu->runstate.state_entry_time +
                            MICROSECS(prv->ratelimit_us) - now;
        if ( ratelimit_min > min_time )
            min_time = ratelimit_min;
    }

    /* 1) Basic time: Run until credit is 0. */
    rt_credit = snext->credit;

    /* 2) If there's someone waiting whose credit is positive,
     * run until your credit ~= his */
    if ( ! list_empty(runq) )
    {
        struct csched2_vcpu *swait = __runq_elem(runq->next);

        if ( ! is_idle_vcpu(swait->vcpu)
             && swait->credit > 0 )
        {
            rt_credit = snext->credit - swait->credit;
        }
    }

    /*
     * The next guy on the runqueue may actually have a higher credit,
     * if we've tried to avoid migrating him from a different cpu.
     * Setting time=0 will ensure the minimum timeslice is chosen.
     *
     * FIXME: See if we can eliminate this conversion if we know time
     * will be outside (MIN,MAX).  Probably requires pre-calculating
     * credit values of MIN,MAX per vcpu, since each vcpu burns credit
     * at a different rate.
     */
    if (rt_credit > 0)
        time = c2t(rqd, rt_credit, snext);
    else
        time = 0;

    /* 3) But never run longer than MAX_TIMER or less than MIN_TIMER or
     * the rate_limit time. */
    if ( time < min_time)
    {
        time = min_time;
        SCHED_STAT_CRANK(runtime_min_timer);
    }
    else if (time > CSCHED2_MAX_TIMER)
    {
        time = CSCHED2_MAX_TIMER;
        SCHED_STAT_CRANK(runtime_max_timer);
    }

    return time;
}

void __dump_execstate(void *unused);

/*
 * Find a candidate.
 */
static struct csched2_vcpu *
runq_candidate(struct csched2_runqueue_data *rqd,
               struct csched2_vcpu *scurr,
               int cpu, s_time_t now,
               unsigned int *pos)
{
    struct list_head *iter;
    struct csched2_vcpu *snext = NULL;
    struct csched2_private *prv = CSCHED2_PRIV(per_cpu(scheduler, cpu));
    /*
     * The way we actually take yields into account is like this:
     * if scurr is yielding, when comparing its credits with other vcpus in
     * the runqueue, act like those other vcpus had yield_bias more credits.
     */
    int yield_bias = 0;
    bool_t cpu_in_soft_aff = 1;

    if ( unlikely(is_idle_vcpu(scurr->vcpu)) )
    {
        snext = scurr;
        goto check_runq;
    }

    if ( __test_and_clear_bit(__CSFLAG_vcpu_yield, &scurr->flags) )
        yield_bias = CSCHED2_YIELD_BIAS;

    /*
     * Return the current vcpu if it has executed for less than ratelimit.
     * Adjuststment for the selected vcpu's credit and decision
     * for how long it will run will be taken in csched2_runtime.
     *
     * Note that, if scurr is yielding, we don't let rate limiting kick in.
     * In fact, it may be the case that scurr is about to spin, and there's
     * no point forcing it to do so until rate limiting expires.
     *
     * To check whether we are yielding, it's enough to look at yield_bias
     * (as CSCHED2_YIELD_BIAS can't be zero). Also, note that the yield flag
     * has been cleared already above.
     */
    if ( !yield_bias &&
         prv->ratelimit_us &&
         vcpu_runnable(scurr->vcpu) &&
         (now - scurr->vcpu->runstate.state_entry_time) <
          MICROSECS(prv->ratelimit_us) )
    {
        if ( unlikely(tb_init_done) )
        {
            struct {
                unsigned vcpu:16, dom:16;
                unsigned runtime;
            } d;
            d.dom = scurr->vcpu->domain->domain_id;
            d.vcpu = scurr->vcpu->vcpu_id;
            d.runtime = now - scurr->vcpu->runstate.state_entry_time;
            __trace_var(TRC_CSCHED2_RATELIMIT, 1,
                        sizeof(d),
                        (unsigned char *)&d);
        }
        return scurr;
    }

    if ( has_soft_affinity(scurr->vcpu, scurr->vcpu->cpu_hard_affinity) )
    {
        affinity_balance_cpumask(scurr->vcpu, BALANCE_SOFT_AFFINITY,
                                 cpumask_scratch);
        cpu_in_soft_aff = cpumask_test_cpu(cpu, cpumask_scratch);
        /* Idle and not-tickled cpus from scurr's soft-affinity. */
        cpumask_and(cpumask_scratch, cpumask_scratch, &rqd->idle);
        cpumask_andnot(cpumask_scratch, cpumask_scratch, &rqd->tickled);
    }

    /*
     * If scurr is runnable, and this cpu is in its soft-affinity, default to
     * it. We also default to it, even if cpu is not in its soft-affinity, if
     * there aren't any idle and not tickled cpu in its soft-affinity. In
     * fact, we don't want to risk leaving scurr in the runq and this cpu idle
     * only because it running outside of its soft-affinity.
     *
     * On the other hand, if cpu is not in scurr's soft-affinity, and there
     * looks to be better options, go for them. That happens by defaulting to
     * idle here, which means scurr will be preempted, put back in runq, and
     * one of those idle and not tickled cpus from its soft affinity will be
     * tickled to pick it up.
     *
     * If scurr does not have a valid soft-affinity, we allow it to continue
     * run here (that's why cpu_in_soft_aff is initialized to 1).
     *
     * Of course, we also default to idle also if scurr is not runnable.
     */
    if ( vcpu_runnable(scurr->vcpu) &&
         (cpu_in_soft_aff || cpumask_empty(cpumask_scratch)) )
        snext = scurr;
    else
        snext = CSCHED2_VCPU(idle_vcpu[cpu]);

 check_runq:
    list_for_each( iter, &rqd->runq )
    {
        struct csched2_vcpu * svc = list_entry(iter, struct csched2_vcpu, runq_elem);
        int svc_credit = svc->credit + yield_bias;

        /* Only consider vcpus that are allowed to run on this processor. */
        if ( !cpumask_test_cpu(cpu, svc->vcpu->cpu_hard_affinity) )
        {
            (*pos)++;
            continue;
        }

        /*
         * If a vcpu is meant to be picked up by another processor, and such
         * processor has not scheduled yet, leave it in the runqueue for him.
         */
        if ( svc->tickled_cpu != -1 && svc->tickled_cpu != cpu &&
             cpumask_test_cpu(svc->tickled_cpu, &rqd->tickled) )
        {
            (*pos)++;
            SCHED_STAT_CRANK(deferred_to_tickled_cpu);
            continue;
        }

        /*
         * If this is on a different processor, don't pull it unless
         * its credit is at least CSCHED2_MIGRATE_RESIST higher.
         */
        if ( svc->vcpu->processor != cpu
             && snext->credit + CSCHED2_MIGRATE_RESIST > svc_credit )
        {
            (*pos)++;
            SCHED_STAT_CRANK(migrate_resisted);
            continue;
        }

        /*
         * If the next one on the list has more credit than current
         * (or idle, if current is not runnable), choose it.
         */
        if ( svc_credit > snext->credit )
            snext = svc;

        /* In any case, if we got this far, break. */
        break;
    }

    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned vcpu:16, dom:16;
            unsigned tickled_cpu, position;
        } d;
        d.dom = snext->vcpu->domain->domain_id;
        d.vcpu = snext->vcpu->vcpu_id;
        d.tickled_cpu = snext->tickled_cpu;
        d.position = *pos;
        __trace_var(TRC_CSCHED2_RUNQ_CANDIDATE, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }

    if ( unlikely(snext->tickled_cpu != -1 && snext->tickled_cpu != cpu) )
        SCHED_STAT_CRANK(tickled_cpu_overridden);

    return snext;
}

/*
 * This function is in the critical path. It is designed to be simple and
 * fast for the common case.
 */
static struct task_slice
csched2_schedule(
    const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
    const int cpu = smp_processor_id();
    struct csched2_runqueue_data *rqd;
    struct csched2_vcpu * const scurr = CSCHED2_VCPU(current);
    struct csched2_vcpu *snext = NULL;
    unsigned int snext_pos = 0;
    struct task_slice ret;
    bool_t tickled;

    SCHED_STAT_CRANK(schedule);
    CSCHED2_VCPU_CHECK(current);

    BUG_ON(!cpumask_test_cpu(cpu, &CSCHED2_PRIV(ops)->initialized));

    rqd = RQD(ops, cpu);
    BUG_ON(!cpumask_test_cpu(cpu, &rqd->active));

    ASSERT(spin_is_locked(per_cpu(schedule_data, cpu).schedule_lock));

    BUG_ON(!is_idle_vcpu(scurr->vcpu) && scurr->rqd != rqd);

    /* Clear "tickled" bit now that we've been scheduled */
    tickled = cpumask_test_cpu(cpu, &rqd->tickled);
    if ( tickled )
    {
        __cpumask_clear_cpu(cpu, &rqd->tickled);
        cpumask_andnot(cpumask_scratch, &rqd->idle, &rqd->tickled);
        smt_idle_mask_set(cpu, cpumask_scratch, &rqd->smt_idle);
    }

    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned cpu:16, rq_id:16;
            unsigned tasklet:8, idle:8, smt_idle:8, tickled:8;
        } d;
        d.cpu = cpu;
        d.rq_id = c2r(ops, cpu);
        d.tasklet = tasklet_work_scheduled;
        d.idle = is_idle_vcpu(current);
        d.smt_idle = cpumask_test_cpu(cpu, &rqd->smt_idle);
        d.tickled = tickled;
        __trace_var(TRC_CSCHED2_SCHEDULE, 1,
                    sizeof(d),
                    (unsigned char *)&d);
    }

    /* Update credits */
    burn_credits(rqd, scurr, now);

    /*
     * Select next runnable local VCPU (ie top of local runq).
     *
     * If the current vcpu is runnable, and has higher credit than
     * the next guy on the queue (or there is noone else), we want to
     * run him again.
     *
     * If there's tasklet work to do, we want to chose the idle vcpu
     * for this processor, and mark the current for delayed runqueue
     * add.
     *
     * If the current vcpu is runnable, and there's another runnable
     * candidate, we want to mark current for delayed runqueue add,
     * and remove the next guy from the queue.
     *
     * If the current vcpu is not runnable, we want to chose the idle
     * vcpu for this processor.
     */
    if ( tasklet_work_scheduled )
    {
        __clear_bit(__CSFLAG_vcpu_yield, &scurr->flags);
        trace_var(TRC_CSCHED2_SCHED_TASKLET, 1, 0, NULL);
        snext = CSCHED2_VCPU(idle_vcpu[cpu]);
    }
    else
        snext = runq_candidate(rqd, scurr, cpu, now, &snext_pos);

    /* If switching from a non-idle runnable vcpu, put it
     * back on the runqueue. */
    if ( snext != scurr
         && !is_idle_vcpu(scurr->vcpu)
         && vcpu_runnable(current) )
        __set_bit(__CSFLAG_delayed_runq_add, &scurr->flags);

    ret.migrated = 0;

    /* Accounting for non-idle tasks */
    if ( !is_idle_vcpu(snext->vcpu) )
    {
        /* If switching, remove this from the runqueue and mark it scheduled */
        if ( snext != scurr )
        {
            ASSERT(snext->rqd == rqd);
            ASSERT(!snext->vcpu->is_running);

            __runq_remove(snext);
            __set_bit(__CSFLAG_scheduled, &snext->flags);
        }

        /*
         * The reset condition is "has a scheduler epoch come to an end?".
         * The way this is enforced is checking whether the vcpu at the top
         * of the runqueue has negative credits. This means the epochs have
         * variable lenght, as in one epoch expores when:
         *  1) the vcpu at the top of the runqueue has executed for
         *     around 10 ms (with default parameters);
         *  2) no other vcpu with higher credits wants to run.
         *
         * Here, where we want to check for reset, we need to make sure the
         * proper vcpu is being used. In fact, runqueue_candidate() may have
         * not returned the first vcpu in the runqueue, for various reasons
         * (e.g., affinity). Only trigger a reset when it does.
         */
        if ( snext_pos == 0 && snext->credit <= CSCHED2_CREDIT_RESET )
        {
            reset_credit(ops, cpu, now, snext);
            balance_load(ops, cpu, now);
        }

        /* Clear the idle mask if necessary */
        if ( cpumask_test_cpu(cpu, &rqd->idle) )
        {
            __cpumask_clear_cpu(cpu, &rqd->idle);
            smt_idle_mask_clear(cpu, &rqd->smt_idle);
        }

        snext->start_time = now;
        snext->tickled_cpu = -1;

        /* Safe because lock for old processor is held */
        if ( snext->vcpu->processor != cpu )
        {
            snext->credit += CSCHED2_MIGRATE_COMPENSATION;
            snext->vcpu->processor = cpu;
            SCHED_STAT_CRANK(migrated);
            ret.migrated = 1;
        }
    }
    else
    {
        /*
         * Update the idle mask if necessary. Note that, if we're scheduling
         * idle in order to carry on some tasklet work, we want to play busy!
         */
        if ( tasklet_work_scheduled )
        {
            if ( cpumask_test_cpu(cpu, &rqd->idle) )
            {
                __cpumask_clear_cpu(cpu, &rqd->idle);
                smt_idle_mask_clear(cpu, &rqd->smt_idle);
            }
        }
        else if ( !cpumask_test_cpu(cpu, &rqd->idle) )
        {
            __cpumask_set_cpu(cpu, &rqd->idle);
            cpumask_andnot(cpumask_scratch, &rqd->idle, &rqd->tickled);
            smt_idle_mask_set(cpu, cpumask_scratch, &rqd->smt_idle);
        }
        /* Make sure avgload gets updated periodically even
         * if there's no activity */
        update_load(ops, rqd, NULL, 0, now);
    }

    /*
     * Return task to run next...
     */
    ret.time = csched2_runtime(ops, cpu, snext, now);
    ret.task = snext->vcpu;

    CSCHED2_VCPU_CHECK(ret.task);
    return ret;
}

static void
csched2_dump_vcpu(struct csched2_private *prv, struct csched2_vcpu *svc)
{
    printk("[%i.%i] flags=%x cpu=%i",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,
            svc->flags,
            svc->vcpu->processor);

    printk(" credit=%" PRIi32" [w=%u]", svc->credit, svc->weight);

    printk(" load=%"PRI_stime" (~%"PRI_stime"%%)", svc->avgload,
           (svc->avgload * 100) >> prv->load_precision_shift);

    printk("\n");
}

static void
csched2_dump_pcpu(const struct scheduler *ops, int cpu)
{
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    struct list_head *runq, *iter;
    struct csched2_vcpu *svc;
    unsigned long flags;
    spinlock_t *lock;
    int loop;
#define cpustr keyhandler_scratch

    /*
     * We need both locks:
     * - csched2_dump_vcpu() wants to access domains' weights,
     *   which are protected by the private scheduler lock;
     * - we scan through the runqueue, so we need the proper runqueue
     *   lock (the one of the runqueue this cpu is associated to).
     */
    read_lock_irqsave(&prv->lock, flags);
    lock = per_cpu(schedule_data, cpu).schedule_lock;
    spin_lock(lock);

    runq = &RQD(ops, cpu)->runq;

    cpumask_scnprintf(cpustr, sizeof(cpustr), per_cpu(cpu_sibling_mask, cpu));
    printk(" sibling=%s, ", cpustr);
    cpumask_scnprintf(cpustr, sizeof(cpustr), per_cpu(cpu_core_mask, cpu));
    printk("core=%s\n", cpustr);

    /* current VCPU */
    svc = CSCHED2_VCPU(curr_on_cpu(cpu));
    if ( svc )
    {
        printk("\trun: ");
        csched2_dump_vcpu(prv, svc);
    }

    loop = 0;
    list_for_each( iter, runq )
    {
        svc = __runq_elem(iter);
        if ( svc )
        {
            printk("\t%3d: ", ++loop);
            csched2_dump_vcpu(prv, svc);
        }
    }

    spin_unlock(lock);
    read_unlock_irqrestore(&prv->lock, flags);
#undef cpustr
}

static void
csched2_dump(const struct scheduler *ops)
{
    struct list_head *iter_sdom;
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    unsigned long flags;
    int i, loop;
#define cpustr keyhandler_scratch

    /*
     * We need the private scheduler lock as we access global
     * scheduler data and (below) the list of active domains.
     */
    read_lock_irqsave(&prv->lock, flags);

    printk("Active queues: %d\n"
           "\tdefault-weight     = %d\n",
           cpumask_weight(&prv->active_queues),
           CSCHED2_DEFAULT_WEIGHT);
    for_each_cpu(i, &prv->active_queues)
    {
        s_time_t fraction;

        fraction = (prv->rqd[i].avgload * 100) >> prv->load_precision_shift;

        cpulist_scnprintf(cpustr, sizeof(cpustr), &prv->rqd[i].active);
        printk("Runqueue %d:\n"
               "\tncpus              = %u\n"
               "\tcpus               = %s\n"
               "\tmax_weight         = %d\n"
               "\tinstload           = %d\n"
               "\taveload            = %"PRI_stime" (~%"PRI_stime"%%)\n",
               i,
               cpumask_weight(&prv->rqd[i].active),
               cpustr,
               prv->rqd[i].max_weight,
               prv->rqd[i].load,
               prv->rqd[i].avgload,
               fraction);

        cpumask_scnprintf(cpustr, sizeof(cpustr), &prv->rqd[i].idle);
        printk("\tidlers: %s\n", cpustr);
        cpumask_scnprintf(cpustr, sizeof(cpustr), &prv->rqd[i].tickled);
        printk("\ttickled: %s\n", cpustr);
        cpumask_scnprintf(cpustr, sizeof(cpustr), &prv->rqd[i].smt_idle);
        printk("\tfully idle cores: %s\n", cpustr);
    }

    printk("Domain info:\n");
    loop = 0;
    list_for_each( iter_sdom, &prv->sdom )
    {
        struct csched2_dom *sdom;
        struct vcpu *v;

        sdom = list_entry(iter_sdom, struct csched2_dom, sdom_elem);

        printk("\tDomain: %d w %d v %d\n",
               sdom->dom->domain_id,
               sdom->weight,
               sdom->nr_vcpus);

        for_each_vcpu( sdom->dom, v )
        {
            struct csched2_vcpu * const svc = CSCHED2_VCPU(v);
            spinlock_t *lock;

            lock = vcpu_schedule_lock(svc->vcpu);

            printk("\t%3d: ", ++loop);
            csched2_dump_vcpu(prv, svc);

            vcpu_schedule_unlock(lock, svc->vcpu);
        }
    }

    read_unlock_irqrestore(&prv->lock, flags);
#undef cpustr
}

static void activate_runqueue(struct csched2_private *prv, int rqi)
{
    struct csched2_runqueue_data *rqd;

    rqd = prv->rqd + rqi;

    BUG_ON(!cpumask_empty(&rqd->active));

    rqd->max_weight = 1;
    rqd->id = rqi;
    INIT_LIST_HEAD(&rqd->svc);
    INIT_LIST_HEAD(&rqd->runq);
    spin_lock_init(&rqd->lock);

    __cpumask_set_cpu(rqi, &prv->active_queues);
}

static void deactivate_runqueue(struct csched2_private *prv, int rqi)
{
    struct csched2_runqueue_data *rqd;

    rqd = prv->rqd + rqi;

    BUG_ON(!cpumask_empty(&rqd->active));
    
    rqd->id = -1;

    __cpumask_clear_cpu(rqi, &prv->active_queues);
}

static inline bool_t same_node(unsigned int cpua, unsigned int cpub)
{
    return cpu_to_node(cpua) == cpu_to_node(cpub);
}

static inline bool_t same_socket(unsigned int cpua, unsigned int cpub)
{
    return cpu_to_socket(cpua) == cpu_to_socket(cpub);
}

static inline bool_t same_core(unsigned int cpua, unsigned int cpub)
{
    return same_socket(cpua, cpub) &&
           cpu_to_core(cpua) == cpu_to_core(cpub);
}

static unsigned int
cpu_to_runqueue(struct csched2_private *prv, unsigned int cpu)
{
    struct csched2_runqueue_data *rqd;
    unsigned int rqi;

    for ( rqi = 0; rqi < nr_cpu_ids; rqi++ )
    {
        unsigned int peer_cpu;

        /*
         * As soon as we come across an uninitialized runqueue, use it.
         * In fact, either:
         *  - we are initializing the first cpu, and we assign it to
         *    runqueue 0. This is handy, especially if we are dealing
         *    with the boot cpu (if credit2 is the default scheduler),
         *    as we would not be able to use cpu_to_socket() and similar
         *    helpers anyway (they're result of which is not reliable yet);
         *  - we have gone through all the active runqueues, and have not
         *    found anyone whose cpus' topology matches the one we are
         *    dealing with, so activating a new runqueue is what we want.
         */
        if ( prv->rqd[rqi].id == -1 )
            break;

        rqd = prv->rqd + rqi;
        BUG_ON(cpumask_empty(&rqd->active));

        peer_cpu = cpumask_first(&rqd->active);
        BUG_ON(cpu_to_socket(cpu) == XEN_INVALID_SOCKET_ID ||
               cpu_to_socket(peer_cpu) == XEN_INVALID_SOCKET_ID);

        if ( opt_runqueue == OPT_RUNQUEUE_ALL ||
             (opt_runqueue == OPT_RUNQUEUE_CORE && same_core(peer_cpu, cpu)) ||
             (opt_runqueue == OPT_RUNQUEUE_SOCKET && same_socket(peer_cpu, cpu)) ||
             (opt_runqueue == OPT_RUNQUEUE_NODE && same_node(peer_cpu, cpu)) )
            break;
    }

    /* We really expect to be able to assign each cpu to a runqueue. */
    BUG_ON(rqi >= nr_cpu_ids);

    return rqi;
}

/* Returns the ID of the runqueue the cpu is assigned to. */
static unsigned
init_pdata(struct csched2_private *prv, unsigned int cpu)
{
    unsigned rqi;
    struct csched2_runqueue_data *rqd;

    ASSERT(rw_is_write_locked(&prv->lock));
    ASSERT(!cpumask_test_cpu(cpu, &prv->initialized));

    /* Figure out which runqueue to put it in */
    rqi = cpu_to_runqueue(prv, cpu);

    rqd = prv->rqd + rqi;

    printk(XENLOG_INFO "Adding cpu %d to runqueue %d\n", cpu, rqi);
    if ( ! cpumask_test_cpu(rqi, &prv->active_queues) )
    {
        printk(XENLOG_INFO " First cpu on runqueue, activating\n");
        activate_runqueue(prv, rqi);
    }
    
    /* Set the runqueue map */
    prv->runq_map[cpu] = rqi;
    
    __cpumask_set_cpu(cpu, &rqd->idle);
    __cpumask_set_cpu(cpu, &rqd->active);
    __cpumask_set_cpu(cpu, &prv->initialized);
    __cpumask_set_cpu(cpu, &rqd->smt_idle);

    return rqi;
}

static void
csched2_init_pdata(const struct scheduler *ops, void *pdata, int cpu)
{
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    spinlock_t *old_lock;
    unsigned long flags;
    unsigned rqi;

    /*
     * pdata contains what alloc_pdata returned. But since we don't (need to)
     * implement alloc_pdata, either that's NULL, or something is very wrong!
     */
    ASSERT(!pdata);

    write_lock_irqsave(&prv->lock, flags);
    old_lock = pcpu_schedule_lock(cpu);

    rqi = init_pdata(prv, cpu);
    /* Move the scheduler lock to the new runq lock. */
    per_cpu(schedule_data, cpu).schedule_lock = &prv->rqd[rqi].lock;

    /* _Not_ pcpu_schedule_unlock(): schedule_lock may have changed! */
    spin_unlock(old_lock);
    write_unlock_irqrestore(&prv->lock, flags);
}

/* Change the scheduler of cpu to us (Credit2). */
static void
csched2_switch_sched(struct scheduler *new_ops, unsigned int cpu,
                     void *pdata, void *vdata)
{
    struct csched2_private *prv = CSCHED2_PRIV(new_ops);
    struct csched2_vcpu *svc = vdata;
    unsigned rqi;

    ASSERT(!pdata && svc && is_idle_vcpu(svc->vcpu));

    /*
     * We own one runqueue lock already (from schedule_cpu_switch()). This
     * looks like it violates this scheduler's locking rules, but it does
     * not, as what we own is the lock of another scheduler, that hence has
     * no particular (ordering) relationship with our private global lock.
     * And owning exactly that one (the lock of the old scheduler of this
     * cpu) is what is necessary to prevent races.
     */
    ASSERT(!local_irq_is_enabled());
    write_lock(&prv->lock);

    idle_vcpu[cpu]->sched_priv = vdata;

    rqi = init_pdata(prv, cpu);

    /*
     * Now that we know what runqueue we'll go in, double check what's said
     * above: the lock we already hold is not the one of this runqueue of
     * this scheduler, and so it's safe to have taken it /before/ our
     * private global lock.
     */
    ASSERT(per_cpu(schedule_data, cpu).schedule_lock != &prv->rqd[rqi].lock);

    per_cpu(scheduler, cpu) = new_ops;
    per_cpu(schedule_data, cpu).sched_priv = NULL; /* no pdata */

    /*
     * (Re?)route the lock to the per pCPU lock as /last/ thing. In fact,
     * if it is free (and it can be) we want that anyone that manages
     * taking it, find all the initializations we've done above in place.
     */
    smp_mb();
    per_cpu(schedule_data, cpu).schedule_lock = &prv->rqd[rqi].lock;

    write_unlock(&prv->lock);
}

static void
csched2_deinit_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    unsigned long flags;
    struct csched2_private *prv = CSCHED2_PRIV(ops);
    struct csched2_runqueue_data *rqd;
    int rqi;

    write_lock_irqsave(&prv->lock, flags);

    /*
     * alloc_pdata is not implemented, so pcpu must be NULL. On the other
     * hand, init_pdata must have been called for this pCPU.
     */
    ASSERT(!pcpu && cpumask_test_cpu(cpu, &prv->initialized));
    
    /* Find the old runqueue and remove this cpu from it */
    rqi = prv->runq_map[cpu];

    rqd = prv->rqd + rqi;

    /* No need to save IRQs here, they're already disabled */
    spin_lock(&rqd->lock);

    printk(XENLOG_INFO "Removing cpu %d from runqueue %d\n", cpu, rqi);

    __cpumask_clear_cpu(cpu, &rqd->idle);
    __cpumask_clear_cpu(cpu, &rqd->smt_idle);
    __cpumask_clear_cpu(cpu, &rqd->active);

    if ( cpumask_empty(&rqd->active) )
    {
        printk(XENLOG_INFO " No cpus left on runqueue, disabling\n");
        deactivate_runqueue(prv, rqi);
    }

    spin_unlock(&rqd->lock);

    __cpumask_clear_cpu(cpu, &prv->initialized);

    write_unlock_irqrestore(&prv->lock, flags);

    return;
}

static int
csched2_init(struct scheduler *ops)
{
    int i;
    struct csched2_private *prv;

    printk("Initializing Credit2 scheduler\n");
    printk(" WARNING: This is experimental software in development.\n" \
           " Use at your own risk.\n");

    printk(XENLOG_INFO " load_precision_shift: %d\n"
           XENLOG_INFO " load_window_shift: %d\n"
           XENLOG_INFO " underload_balance_tolerance: %d\n"
           XENLOG_INFO " overload_balance_tolerance: %d\n"
           XENLOG_INFO " runqueues arrangement: %s\n",
           opt_load_precision_shift,
           opt_load_window_shift,
           opt_underload_balance_tolerance,
           opt_overload_balance_tolerance,
           opt_runqueue_str[opt_runqueue]);

    if ( opt_load_precision_shift < LOADAVG_PRECISION_SHIFT_MIN )
    {
        printk("WARNING: %s: opt_load_precision_shift %d below min %d, resetting\n",
               __func__, opt_load_precision_shift, LOADAVG_PRECISION_SHIFT_MIN);
        opt_load_precision_shift = LOADAVG_PRECISION_SHIFT_MIN;
    }

    if ( opt_load_window_shift <= LOADAVG_GRANULARITY_SHIFT )
    {
        printk("WARNING: %s: opt_load_window_shift %d too short, resetting\n",
               __func__, opt_load_window_shift);
        opt_load_window_shift = LOADAVG_WINDOW_SHIFT;
    }
    printk(XENLOG_INFO "load tracking window lenght %llu ns\n",
           1ULL << opt_load_window_shift);

    if ( opt_yield_bias < CSCHED2_YIELD_BIAS_MIN )
    {
        printk("WARNING: %s: opt_yield_bias %d too small, resetting\n",
               __func__, opt_yield_bias);
        opt_yield_bias = 1000; /* 1 ms */
    }
    printk(XENLOG_INFO "yield bias value %d us\n", opt_yield_bias);

    /* Basically no CPU information is available at this point; just
     * set up basic structures, and a callback when the CPU info is
     * available. */

    prv = xzalloc(struct csched2_private);
    if ( prv == NULL )
        return -ENOMEM;
    ops->sched_data = prv;

    rwlock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->sdom);

    /* But un-initialize all runqueues */
    for ( i = 0; i < nr_cpu_ids; i++ )
    {
        prv->runq_map[i] = -1;
        prv->rqd[i].id = -1;
    }
    /* initialize ratelimit */
    prv->ratelimit_us = sched_ratelimit_us;

    prv->load_precision_shift = opt_load_precision_shift;
    prv->load_window_shift = opt_load_window_shift - LOADAVG_GRANULARITY_SHIFT;
    ASSERT(opt_load_window_shift > 0);

    return 0;
}

static void
csched2_deinit(struct scheduler *ops)
{
    struct csched2_private *prv;

    prv = CSCHED2_PRIV(ops);
    ops->sched_data = NULL;
    xfree(prv);
}

static const struct scheduler sched_credit2_def = {
    .name           = "SMP Credit Scheduler rev2",
    .opt_name       = "credit2",
    .sched_id       = XEN_SCHEDULER_CREDIT2,
    .sched_data     = NULL,

    .init_domain    = csched2_dom_init,
    .destroy_domain = csched2_dom_destroy,

    .insert_vcpu    = csched2_vcpu_insert,
    .remove_vcpu    = csched2_vcpu_remove,

    .sleep          = csched2_vcpu_sleep,
    .wake           = csched2_vcpu_wake,
    .yield          = csched2_vcpu_yield,

    .adjust         = csched2_dom_cntl,
    .adjust_global  = csched2_sys_cntl,

    .pick_cpu       = csched2_cpu_pick,
    .migrate        = csched2_vcpu_migrate,
    .do_schedule    = csched2_schedule,
    .context_saved  = csched2_context_saved,

    .dump_cpu_state = csched2_dump_pcpu,
    .dump_settings  = csched2_dump,
    .init           = csched2_init,
    .deinit         = csched2_deinit,
    .alloc_vdata    = csched2_alloc_vdata,
    .free_vdata     = csched2_free_vdata,
    .init_pdata     = csched2_init_pdata,
    .deinit_pdata   = csched2_deinit_pdata,
    .switch_sched   = csched2_switch_sched,
    .alloc_domdata  = csched2_alloc_domdata,
    .free_domdata   = csched2_free_domdata,
};

REGISTER_SCHEDULER(sched_credit2_def);
