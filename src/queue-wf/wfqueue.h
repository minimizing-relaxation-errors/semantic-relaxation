#ifndef WFQUEUE_H
#define WFQUEUE_H

#include "align.h"
#include "common.h"
#define EMPTY ((void *) 0)

#ifndef WFQUEUE_NODE_SIZE
#define WFQUEUE_NODE_SIZE ((1 << 10) - 2)
#endif

struct _enq_t {
  long volatile id;
  void * volatile val;
} CACHE_ALIGNED;

struct _deq_t {
  long volatile id;
  long volatile idx;
} CACHE_ALIGNED;

struct _cell_t {
  void * volatile val;
  struct _enq_t * volatile enq;
  struct _deq_t * volatile deq;
  // void * pad[5];
};

struct _node_t {
  struct _node_t * volatile next CACHE_ALIGNED;
  long id CACHE_ALIGNED;
  struct _cell_t cells[WFQUEUE_NODE_SIZE] CACHE_ALIGNED;
};

typedef struct CACHE_ALIGNED {
  /**
   * Index of the next position for enqueue.
   */
  volatile long Ei CACHE_ALIGNED;

  /**
   * Index of the next position for dequeue.
   */
  volatile long Di CACHE_ALIGNED;

  /**
   * Index of the head of the queue.
   */
  volatile long Hi CACHE_ALIGNED;

  /**
   * Pointer to the head node of the queue.
   */
  struct _node_t * volatile Hp;

  /**
   * Number of processors.
   */
  long nprocs;
#ifdef RECORD
  long slowenq;
  long slowdeq;
  long fastenq;
  long fastdeq;
  long empty;
#endif
} queue_t;

typedef struct _handle_t {
  /**
   * Pointer to the next handle.
   */
  struct _handle_t * next;

  /**
   * Hazard pointer.
   */
  //struct _node_t * volatile Hp;
  unsigned long volatile hzd_node_id;

  /**
   * Pointer to the node for enqueue.
   */
  struct _node_t * volatile Ep;
  unsigned long enq_node_id;

  /**
   * Pointer to the node for dequeue.
   */
  struct _node_t * volatile Dp;
  unsigned long deq_node_id;

  /**
   * Enqueue request.
   */
  struct _enq_t Er CACHE_ALIGNED;

  /**
   * Dequeue request.
   */
  struct _deq_t Dr CACHE_ALIGNED;

  /**
   * Handle of the next enqueuer to help.
   */
  struct _handle_t * Eh CACHE_ALIGNED;

  long Ei;

  /**
   * Handle of the next dequeuer to help.
   */
  struct _handle_t * Dh;

  /**
   * Pointer to a spare node to use, to speedup adding a new node.
   */
  struct _node_t * spare CACHE_ALIGNED;

  /**
   * Count the delay rounds of helping another dequeuer.
   */
  int delay;

  /**
   * Pointer to the associated queue
  */
  queue_t* queue;

#ifdef RECORD
  long slowenq;
  long slowdeq;
  long fastenq;
  long fastdeq;
  long empty;
#endif
} handle_t;


/* INTERFACE FOR 2D TESTING FRAMEWORK */

#define DS_ADD(s,k,v)       enqueue_wrap(s, (void*) k)
#define DS_REMOVE(s)        dequeue_wrap(s)
// #define DS_SIZE(s)          queue_size(s)
#define DS_REGISTER(s,i)    queue_register(s,i)
#define DS_NEW(n)           create_queue(n)

#define DS_TYPE             queue_t
#define DS_HANDLE           handle_t*

// Expose functions
int enqueue_wrap(handle_t *th, void *v);
sval_t dequeue_wrap(handle_t *th);
queue_t* create_queue(int nprocs);
handle_t* queue_register(queue_t *q, int id);

/* End of interface */


#endif /* end of include guard: WFQUEUE_H */

