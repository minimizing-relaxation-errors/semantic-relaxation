#ifndef RELAXATION_ANALYSIS_TIMESTAMPS_H
#define RELAXATION_ANALYSIS_TIMESTAMPS_H

#include "common.h"
#include <stdint.h>
#include <time.h>
#include <sys/time.h>

// How many operations we can track per thread
// This should be set experimentally, but we probably can't handle too large values
#define MAX_RELAX_COUNTS 1e8

// The record for a single operation
typedef struct relax_stamp
{
    uint64_t start;
    uint64_t end;
    uint64_t lin;
    sval_t value;
} relax_stamp_t;

// Shared functions

// Get a timestamp from the realtime clock, shared accross processors
uint64_t get_timestamp();

// Add a put operation of a value with its timestamp
void add_relaxed_put(sval_t val, uint64_t start, uint64_t end, uint64_t lin);

// Add a get operation of a value with its timestamp
void add_relaxed_get(sval_t val, uint64_t start, uint64_t end, uint64_t lin);

// Init the relaxation analysis, global variables before the thread local one
void init_relaxation_analysis_shared(int nbr_threads);

// Init the relaxation analysis, thread local variables
void init_relaxation_analysis_local(int thread_id);

// de-init all memory for all threads
void destoy_relaxation_analysis_all(int nbr_threads);

// Print the stats from the relaxation measurement
void print_relaxation_measurements(int nbr_threads, char queue[]);

#endif
