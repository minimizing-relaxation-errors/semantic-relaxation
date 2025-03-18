#ifndef D_BALANCED_FAAAQ_H
#define D_BALANCED_FAAAQ_H

#include <assert.h>
#include <getopt.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <stdint.h>
#include "common.h"

#include "lock_if.h"
#include "ssmem.h"
#include "utils.h"

#ifdef RELAXATION_TIMER_ANALYSIS
#include "relaxation_analysis_timestamps.h"
#elif RELAXATION_LINEARIZATION_TIMESTAMP
#include "relaxation_linearization_timestamps.h"
#elif RELAXATION_ANALYSIS
#include "relaxation_analysis_queue.h"
#endif
#define DS_ADD(s,k,v)       faaaq_enqueue(s,k,v)
#define DS_REMOVE(s)        faaaq_dequeue(s, NULL)
#define DS_SIZE(s)          faaaq_queue_size(s)
#define DS_REGISTER(s,i)    queue_register(s,i)
#define DS_NEW(i)           create_faaaq_queue(i)

#define DS_TYPE             faaaq_t
#define DS_HANDLE           faaaq_t*
#define DS_NODE             sval_t

#define EMPTY						((sval_t)0)
// Internally used macros
#define TAKEN				        ((sval_t) -1)
#define BUFFER_SIZE 		        ((uint64_t) 1024)

/* Type definitions */

typedef ALIGNED(CACHE_LINE_SIZE) struct segment
{
	ALIGNED(CACHE_LINE_SIZE) volatile uint64_t enq_idx;
    ALIGNED(CACHE_LINE_SIZE) volatile uint64_t deq_idx;
    ALIGNED(CACHE_LINE_SIZE) struct segment *volatile next;
    uint64_t node_idx;
	volatile sval_t items[];
} segment_t;

typedef ALIGNED(CACHE_LINE_SIZE) struct faaaq
{
    segment_t * volatile head;
    segment_t * volatile tail;
	uint8_t padding[CACHE_LINE_SIZE - 2*sizeof(segment_t*)];
} faaaq_t;


/*Global variables*/

/*Thread local variables*/
extern __thread ssmem_allocator_t* alloc;
extern __thread int thread_id;

extern __thread unsigned long my_put_cas_fail_count;
extern __thread unsigned long my_get_cas_fail_count;
extern __thread unsigned long my_null_count;
extern __thread unsigned long my_hop_count;
extern __thread unsigned long my_slide_count;

/* Interfaces */
int faaaq_enqueue(faaaq_t *queue, skey_t key, sval_t val);
sval_t faaaq_dequeue(faaaq_t *queue, uint64_t *double_collect_count);
void init_faaaq_queue(faaaq_t *queue, int thread_id);
faaaq_t *create_faaaq_queue(int thread_id);
faaaq_t* queue_register(faaaq_t* set, int thread_id);
size_t faaaq_queue_size(faaaq_t *queue);
uint64_t faaaq_enq_count(faaaq_t *queue);
uint64_t faaaq_deq_count(faaaq_t *queue);

#endif
