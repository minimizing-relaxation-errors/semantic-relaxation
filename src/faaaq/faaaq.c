#include "faaaq.h"

#ifdef RELAXATION_ANALYSIS
#include "relaxation_analysis_queue.c"
#elif RELAXATION_TIMER_ANALYSIS
#include "relaxation_analysis_timestamps.c"
#elif RELAXATION_LINEARIZATION_TIMESTAMP
#include "relaxation_linearization_timestamps.c"
#endif

// Want timers at FAA increments and not with the normal CAE
#ifdef RELAXATION_TIMER_ANALYSIS
uint64_t enq_timestamp, deq_timestamp;
#define ENQ_TIMESTAMP (enq_timestamp = get_timestamp());
#define DEQ_TIMESTAMP (deq_timestamp = get_timestamp());
#define DEQ_START_TIMESTAMP
#define DEQ_END_TIMESTAMP
#define ENQ_START_TIMESTAMP
#define ENQ_END_TIMESTAMP
#elif RELAXATION_LINEARIZATION_TIMESTAMP
__thread uint64_t enq_start_timestamp;
__thread uint64_t enq_end_timestamp;
__thread uint64_t deq_start_timestamp;
__thread uint64_t deq_end_timestamp;
#define ENQ_START_TIMESTAMP (enq_start_timestamp = get_timestamp());
#define ENQ_END_TIMESTAMP (enq_end_timestamp = get_timestamp());
#define DEQ_START_TIMESTAMP (deq_start_timestamp = get_timestamp());
#define DEQ_END_TIMESTAMP (deq_end_timestamp = get_timestamp());
#define ENQ_TIMESTAMP
#define DEQ_TIMESTAMP
#else
#define ENQ_TIMESTAMP
#define DEQ_TIMESTAMP
#define ENQ_START_TIMESTAMP
#define ENQ_END_TIMESTAMP
#define DEQ_START_TIMESTAMP
#define DEQ_END_TIMESTAMP
#endif

__thread ssmem_allocator_t *alloc;

// TODO: use ssmem_alloc when using GC (this uses ssmem instead of ssalloc)
segment_t *create_segment(skey_t key, sval_t val, segment_t *next, uint64_t node_idx)
{
#if GC == 1
    segment_t *segment = (segment_t *)ssmem_alloc(alloc, sizeof(segment_t) + BUFFER_SIZE * sizeof(sval_t));
#else
    segment_t *segment = (segment_t *)ssalloc(sizeof(segment_t) + BUFFER_SIZE * sizeof(sval_t));
#endif
    segment->next = NULL;
    segment->deq_idx = 0;
    segment->enq_idx = 1;
    segment->node_idx = node_idx;

    segment->items[0] = val;
    memset((void *)&segment->items[1], 0, (BUFFER_SIZE - 1) * sizeof(sval_t));
    return segment;
}

static int enq_cae(volatile sval_t *item_loc, sval_t new_value)
{
    sval_t expected = EMPTY;
#ifdef RELAXATION_TIMER_ANALYSIS
    // Use timers to track relaxation instead of locks
    if (CAE(item_loc, &expected, &new_value))
    {
        // Save this count in a local array of (timestamp, )
        add_relaxed_put(new_value, enq_timestamp);
        return true;
    }
    return false;
#elif RELAXATION_LINEARIZATION_TIMESTAMP
    // Use timers to track relaxation instead of locks
    if (CAE(item_loc, &expected, &new_value))
    {
        // Save this count in a local array of (timestamp, )
        add_relaxed_put(new_value, enq_start_timestamp, enq_end_timestamp);
        return true;
    }
    return false;
#elif RELAXATION_ANALYSIS
    lock_relaxation_lists();

    if (CAE(item_loc, &expected, &new_value))
    {
        *item_loc = gen_relaxation_count();
        add_linear(*item_loc, 0);
        unlock_relaxation_lists();
        return true;
    }
    else
    {
        unlock_relaxation_lists();
        return false;
    }
#else
    return CAE(item_loc, &expected, &new_value);
#endif
}

static sval_t deq_swp(volatile sval_t *item_loc)
{
#ifdef RELAXATION_TIMER_ANALYSIS
    // Use timers to track relaxation instead of locks
    sval_t item = SWAP_U64(item_loc, TAKEN);
    if (item != EMPTY)
    {
        add_relaxed_get(item, deq_timestamp);
    }
    return item;
#elif RELAXATION_LINEARIZATION_TIMESTAMP
    sval_t item = SWAP_U64(item_loc, TAKEN);
    return item;
#elif RELAXATION_ANALYSIS
    lock_relaxation_lists();
    sval_t item = SWAP_U64(item_loc, TAKEN);
    if (item != EMPTY)
    {
        remove_linear(item);
    }
    unlock_relaxation_lists();
    return item;
#else
    return SWAP_U64(item_loc, TAKEN);
#endif
}

int faaaq_enqueue(faaaq_t *q, skey_t key, sval_t val)
{
    while (true)
    {
        ENQ_START_TIMESTAMP;
        segment_t *tail = q->tail;
        // Linearization point
        uint64_t idx = FAI_U64(&tail->enq_idx);
        ENQ_TIMESTAMP;
        if (idx > BUFFER_SIZE - 1)
        {
            if (tail != q->tail)
                continue;
            segment_t *next = tail->next;
            if (next == NULL)
            {
                // Create segment (node)
                segment_t *new_segment = create_segment(key, val, NULL, tail->node_idx + 1);
                segment_t *null_segment = NULL;
                if (CAE(&tail->next, &null_segment, &new_segment))
                {
                    CAE(&q->tail, &tail, &new_segment);
                    ENQ_END_TIMESTAMP;
#ifdef RELAXATION_TIMER_ANALYSIS
                    add_relaxed_put(val, enq_timestamp);
#elif RELAXATION_LINEARIZATION_TIMESTAMP
                    add_relaxed_put(val, enq_start_timestamp, enq_end_timestamp);
#endif

                    return 1;
                }
#if GC == 1
                ssmem_free(alloc, (void *)new_segment);
#endif
            }
            else
            {
                CAE(&q->tail, &tail, &next);
            }
            continue;
        }
        ENQ_END_TIMESTAMP;
        if (enq_cae(&tail->items[idx], val))
        {
            return 1;
        }
    }
}

sval_t faaaq_dequeue(faaaq_t *q, uint64_t *double_collect_count)
{
    while (true)
    { // ta en timestamp här när vi börjar
        DEQ_START_TIMESTAMP;
        segment_t *head = q->head;
        if (head->deq_idx >= head->enq_idx && head->next == NULL)
            break;
        // Linearization point
        uint64_t idx = FAI_U64(&head->deq_idx);
        DEQ_TIMESTAMP; // IDE: timestamp macro
        if (idx > BUFFER_SIZE - 1)
        {
            segment_t *next = head->next;
            if (next == NULL)
                break;
            if (CAE(&q->head, &head, &next))
            {
#if GC == 1
                ssmem_free(alloc, (void *)head);
#endif
            }
            continue;
        }
        sval_t item = deq_swp(&head->items[idx]);
        if (item != EMPTY)
        { // och en timestamp innan return
            DEQ_END_TIMESTAMP;
#ifdef RELAXATION_LINEARIZATION_TIMESTAMP
            add_relaxed_get(item, deq_start_timestamp, deq_end_timestamp);
#endif
            return item;
        }
    }
    return 0;
}

void init_faaaq_queue(faaaq_t *q, int thread_id)
{
#if GC == 1
    if (alloc == NULL)
    {
        alloc = (ssmem_allocator_t *)malloc(sizeof(ssmem_allocator_t));
        assert(alloc != NULL);
        ssmem_alloc_init_fs_size(alloc, SSMEM_DEFAULT_MEM_SIZE, SSMEM_GC_FREE_SET_SIZE, thread_id);
    }
#endif

#if GC == 1
    segment_t *segment = (segment_t *)ssmem_alloc(alloc, sizeof(segment_t) + BUFFER_SIZE * sizeof(sval_t));
#else
    segment_t *segment = (segment_t *)ssalloc(sizeof(segment_t) + BUFFER_SIZE * sizeof(sval_t));
#endif
    segment->next = NULL;
    segment->deq_idx = 0;
    segment->enq_idx = 0;
    segment->node_idx = 0;
    memset((void *)&segment->items[0], 0, BUFFER_SIZE * sizeof(sval_t));

    q->head = segment;
    q->tail = segment;
}

faaaq_t *create_faaaq_queue(int thread_id)
{
    ssalloc_init();
    faaaq_t *q = (faaaq_t *)ssalloc_aligned(CACHE_LINE_SIZE, sizeof(faaaq_t));
    init_faaaq_queue(q, thread_id);
    return q;
}

size_t faaaq_queue_size(faaaq_t *q)
{
    size_t size = 0;
    segment_t *node = q->head;
    while (node)
    {
        for (int idx = 0; idx < BUFFER_SIZE; idx += 1)
        {
            if (node->items[idx] != EMPTY && node->items[idx] != TAKEN)
            {
                size += 1;
            }
        }
        node = node->next;
    }
    return size;
}

uint64_t faaaq_enq_count(faaaq_t *q)
{
    segment_t *tail = q->tail;
    uint64_t idx = tail->enq_idx;
    if (idx > BUFFER_SIZE - 1)
        idx = BUFFER_SIZE;
    return idx + BUFFER_SIZE * tail->node_idx;
}

uint64_t faaaq_deq_count(faaaq_t *q)
{
    segment_t *head = q->head;
    uint64_t idx = head->deq_idx;
    if (idx > BUFFER_SIZE - 1)
        idx = BUFFER_SIZE;
    return idx + BUFFER_SIZE * head->node_idx;
}

faaaq_t *queue_register(faaaq_t *set, int thread_id)
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

    return set;
}