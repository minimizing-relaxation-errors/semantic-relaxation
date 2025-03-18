#ifndef D_BALANCED_QUEUE_H
#define D_BALANCED_QUEUE_H

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

#ifdef RELAXATION_LINEARIZATION_TIMESTAMP
#include "relaxation_linearization_timestamps.h"
#endif
// Include specific partial queue
#include "partial-faaaq.h"

/* ################################################################### *
 * Definition of macros: per data structure
 * ################################################################### */

#define DS_ADD(s, k, v) enqueue(s, k, v)
#define DS_REMOVE(s) dequeue(s)
#define DS_SIZE(s) queue_size(s)
#define DS_NEW(w, d, i) create_queue(w, d, i)
#define DS_REGISTER(q, i) d_balanced_register(q, i)

#define DS_HANDLE mqueue_t *
#define DS_TYPE mqueue_t
#define DS_NODE sval_t

typedef ALIGNED(CACHE_LINE_SIZE) struct mqueue_file
{
	PARTIAL_T *queues;
	uint32_t width;
	uint32_t d;
	uint8_t padding[CACHE_LINE_SIZE - (sizeof(PARTIAL_T *)) - 2 * sizeof(int32_t)];
} mqueue_t;

/*Global variables*/

/*Thread local variables*/
extern __thread ssmem_allocator_t *alloc;
extern __thread int thread_id;

extern __thread unsigned long my_put_cas_fail_count;
extern __thread unsigned long my_get_cas_fail_count;
extern __thread unsigned long my_null_count;
extern __thread unsigned long my_hop_count;
extern __thread unsigned long my_slide_count;

/* Interfaces */
int enqueue(mqueue_t *set, skey_t key, sval_t val);
sval_t dequeue(mqueue_t *set);
mqueue_t *create_queue(uint32_t n_partial, uint32_t d, int nbr_threads);
size_t queue_size(mqueue_t *set);
uint32_t random_index(mqueue_t *set);
sval_t double_collect(mqueue_t *set, uint32_t start_index);
mqueue_t *d_balanced_register(mqueue_t *set, int thread_id);

#endif