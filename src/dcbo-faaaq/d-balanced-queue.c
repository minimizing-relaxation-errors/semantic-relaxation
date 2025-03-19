#include "d-balanced-queue.h"

#ifdef RELAXATION_LINEARIZATION_TIMESTAMP
#include "relaxation_linearization_timestamps.c"
#endif

#ifdef RELAXATION_LINEARIZATION_TIMESTAMP
__thread uint64_t enq_start_timestamp;
__thread uint64_t enq_end_timestamp;
__thread uint64_t deq_start_timestamp;
__thread uint64_t deq_end_timestamp;
#define ENQ_START_TIMESTAMP (enq_start_timestamp = get_timestamp());
#define ENQ_END_TIMESTAMP (enq_end_timestamp = get_timestamp());
#define DEQ_START_TIMESTAMP (deq_start_timestamp = get_timestamp());
#define DEQ_END_TIMESTAMP (deq_end_timestamp = get_timestamp());
#else
#define ENQ_START_TIMESTAMP
#define ENQ_END_TIMESTAMP
#define DEQ_START_TIMESTAMP
#define DEQ_END_TIMESTAMP
#endif
// Internal thread local count for double-collect
// Don't have in header as it would double-instantiate both here and in the test file
__thread uint64_t *double_collect_counts;
__thread ssmem_allocator_t *alloc;

int enqueue(mqueue_t *set, skey_t key, sval_t val)
{
    ENQ_START_TIMESTAMP;
#ifdef LENGTH_HEURISTIC
#define ENQ_HEURISTIC(q) PARTIAL_LENGTH(q)
#else
#define ENQ_HEURISTIC(q) PARTIAL_ENQ_COUNT(q)
#endif

    uint32_t opt_index = random_index(set);
    uint64_t opt = ENQ_HEURISTIC(&set->queues[opt_index]);
    for (int i = 1; i < set->d; i++)
    {
        uint32_t index = random_index(set);
        uint64_t index_val = ENQ_HEURISTIC(&set->queues[index]);
        if (index_val < opt)
        {
            opt_index = index;
            opt = index_val;
        }
    }
    ENQ_END_TIMESTAMP;
#ifdef RELAXATION_LINEARIZATION_TIMESTAMP
    add_relaxed_put(val, enq_start_timestamp, enq_end_timestamp);
#endif
    return PARTIAL_ENQUEUE(&set->queues[opt_index], key, val);
}

sval_t dequeue(mqueue_t *set)
{
    DEQ_START_TIMESTAMP;
#ifdef LENGTH_HEURISTIC
#define DEQ_HEURISTIC(q) -PARTIAL_LENGTH(q)
#else
#define DEQ_HEURISTIC(q) PARTIAL_DEQ_COUNT(q)
#endif

    uint32_t opt_index = random_index(set);
    int64_t opt = DEQ_HEURISTIC(&set->queues[opt_index]);
    for (int i = 1; i < set->d; i++)
    {
        uint32_t index = random_index(set);
        int64_t index_val = DEQ_HEURISTIC(&set->queues[index]);
        if (index_val < opt)
        {
            opt_index = index;
            opt = index_val;
        }
    }

    sval_t v = PARTIAL_DEQUEUE(&(set->queues[opt_index]));
    if (v != EMPTY)
    {
        DEQ_END_TIMESTAMP;
#ifdef RELAXATION_LINEARIZATION_TIMESTAMP
        add_relaxed_get(v, deq_start_timestamp, deq_end_timestamp);
#endif
        return v;
    }

    return double_collect(set, opt_index + 1);
}

sval_t double_collect(mqueue_t *set, uint32_t start_index)
{
    uint32_t index;
    uint64_t throwaway;

start:
    // Loop through all, collecting their tail versions and then try to dequeue if not empty
    for (uint32_t i = 0; i < set->width; i++)
    {
        index = (start_index + i) % set->width; // TODO: Optimize away modulo

        double_collect_counts[index] = PARTIAL_TAIL_VERSION(&set->queues[index]);
        sval_t v = PARTIAL_DEQUEUE(&(set->queues[index]));
        if (v != EMPTY)
        {
            DEQ_END_TIMESTAMP;
#ifdef RELAXATION_LINEARIZATION_TIMESTAMP
            add_relaxed_get(v, deq_start_timestamp, deq_end_timestamp);
#endif
            return v;
        }
    }

    // Return empty if all counts are the same and the queues are still empty, otherwise restart
    for (uint32_t i = 0; i < set->width; i++)
    {
        index = (start_index + i) % set->width;
        if (double_collect_counts[index] != PARTIAL_TAIL_VERSION(&(set->queues[index])))
        {
            start_index = index;
            goto start;
        }
    }

    return EMPTY;
}

mqueue_t *create_queue(uint32_t n_partial, uint32_t d, int nbr_threads)
{
    // Allocate n_partial MS
    mqueue_t *set;

    // Create an allocator for the main thread to more easily allocate the first queue node
    ssalloc_init();
#if GC == 1
    if (alloc == NULL)
    {
        alloc = (ssmem_allocator_t *)malloc(sizeof(ssmem_allocator_t));
        assert(alloc != NULL);
        ssmem_alloc_init_fs_size(alloc, SSMEM_DEFAULT_MEM_SIZE, SSMEM_GC_FREE_SET_SIZE, nbr_threads);
    }
#endif

    if ((set = (mqueue_t *)ssalloc_aligned(CACHE_LINE_SIZE, sizeof(mqueue_t))) == NULL)
    {
        perror("malloc");
        exit(1);
    }
    set->queues = ssalloc_aligned(CACHE_LINE_SIZE, n_partial * sizeof(PARTIAL_T)); // ssalloc(width);
    set->width = n_partial;
    set->d = d;

    uint32_t i;
    for (i = 0; i < set->width; i++)
    {
        INIT_PARTIAL(&(set->queues[i]), nbr_threads);
    }

    return set;
}

size_t queue_size(mqueue_t *set)
{
    uint64_t total = 0;
    for (int i = 0; i < set->width; i++)
    {
        total += PARTIAL_LENGTH(&set->queues[i]);
    }
    return total;
}

uint32_t random_index(mqueue_t *set)
{
    return (my_random(&(seeds[0]), &(seeds[1]), &(seeds[2])) % (set->width));
}

// Set up thread local variables for the queue
mqueue_t *d_balanced_register(mqueue_t *set, int thread_id)
{
    ssalloc_init();
#if GC == 1
    if (alloc == NULL)
    {
        alloc = (ssmem_allocator_t *)malloc(sizeof(ssmem_allocator_t));
        assert(alloc != NULL);
        ssmem_alloc_init_fs_size(alloc, SSMEM_DEFAULT_MEM_SIZE, SSMEM_GC_FREE_SET_SIZE, thread_id);
    }
#endif

    double_collect_counts = malloc(set->width * sizeof(uint64_t));
#ifdef RELAXATION_TIMER_ANALYSIS
    init_relaxation_analysis_local(thread_id);
#endif
    return set;
}