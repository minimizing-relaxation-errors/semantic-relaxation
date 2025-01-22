BENCHS = src/stack-dra src/queue-dra src/queue-ms_lb src/queue-wf src/queue-wf-ssmem src/queue-k-segment src/stack-elimination src/stack-k-segment src/stack-treiber src/2Dc-counter src/2Dd-counter src/2Dc-stack src/2Dc-stack_optimized src/2Dc-stack_elastic-lpw src/2Dd-stack src/multi-stack_random-relaxed src/multi-counter-faa_random-relaxed src/multi-counter_random-relaxed  src/2Dd-queue src/2Dd-queue_optimized src/2Dd-queue_elastic-lpw src/2Dd-queue_elastic-law src/dcbo-ms src/simple-dcbo-ms src/dcbo-faaaq src/simple-dcbo-faaaq src/dcbo-lcrq src/simple-dcbo-lcrq src/dcbo-wfqueue src/simple-dcbo-wfqueue src/lcrq src/faaaq src/ms src/counter-cas src/single-faa
# src/2Dd-deque

.PHONY:	clean $(BENCHS)

all:
	$(MAKE)  2D multi_ran external_queues external_stacks external_counters dcbo dcbl

2Dd-queue:
	$(MAKE) src/2Dd-queue
2Dd-queue_optimized:
	$(MAKE) src/2Dd-queue_optimized
2Dd-queue_elastic-lpw:
	$(MAKE) src/2Dd-queue_elastic-lpw
2Dd-queue_elastic-law:
	$(MAKE) src/2Dd-queue_elastic-law
queue-1ra:
	$(MAKE) src/queue-dra
queue-2ra:
	$(MAKE) "CHOICES=two" src/queue-dra
queue-4ra:
	$(MAKE) "CHOICES=four" src/queue-dra
queue-8ra:
	$(MAKE) "CHOICES=eight" src/queue-dra
dcbo-ms:
	$(MAKE) src/dcbo-ms
dcbl-ms:
	$(MAKE) "HEURISTIC=LENGTH" src/dcbo-ms
simple-dcbo-ms:
	$(MAKE) src/simple-dcbo-ms
simple-dcbl-ms:
	$(MAKE) "HEURISTIC=LENGTH" src/simple-dcbo-ms
dcbo-faaaq:
	$(MAKE) src/dcbo-faaaq
dcbl-faaaq:
	$(MAKE) "HEURISTIC=LENGTH" src/dcbo-faaaq
simple-dcbo-faaaq:
	$(MAKE) src/simple-dcbo-faaaq
simple-dcbl-faaaq:
	$(MAKE) "HEURISTIC=LENGTH" src/simple-dcbo-faaaq
dcbo-lcrq:
	$(MAKE) src/dcbo-lcrq
dcbl-lcrq:
	$(MAKE) "HEURISTIC=LENGTH" src/dcbo-lcrq
simple-dcbo-lcrq:
	$(MAKE) src/simple-dcbo-lcrq
simple-dcbl-lcrq:
	$(MAKE) "HEURISTIC=LENGTH" src/simple-dcbo-lcrq
dcbo-wfqueue:
	$(MAKE) src/dcbo-wfqueue
dcbl-wfqueue:
	$(MAKE) "HEURISTIC=LENGTH" src/dcbo-wfqueue
simple-dcbo-wfqueue:
	$(MAKE) src/simple-dcbo-wfqueue
simple-dcbl-wfqueue:
	$(MAKE) "HEURISTIC=LENGTH" src/simple-dcbo-wfqueue

#2Dd-deque:
#	$(MAKE) src/2Dd-deque

stack-treiber:
	$(MAKE) src/stack-treiber
stack-elimination:
	$(MAKE) src/stack-elimination
stack-k-segment:
	$(MAKE) src/stack-k-segment
multi-st_ran:
	$(MAKE) src/multi-stack_random-relaxed
multi-st_ran2c:
	$(MAKE) "CHOICES=two" src/multi-stack_random-relaxed
multi-st_ran4c:
	$(MAKE) "CHOICES=four" src/multi-stack_random-relaxed
multi-st_ran8c:
	$(MAKE) "CHOICES=eight" src/multi-stack_random-relaxed
stack-1ra:
	$(MAKE) src/stack-dra
stack-2ra:
	$(MAKE) "CHOICES=two" src/stack-dra
stack-4ra:
	$(MAKE) "CHOICES=four" src/stack-dra
stack-8ra:
	$(MAKE) "CHOICES=eight" src/stack-dra
2Dc-stack:
	$(MAKE) -C src/2Dc-stack main
2Dc-stack_optimized:
	$(MAKE) -C src/2Dc-stack_optimized main
2Dc-stack_elastic-lpw:
	$(MAKE) src/2Dc-stack_elastic-lpw
2Dd-stack:
	$(MAKE) src/2Dd-stack

queue-ms_lb:
	$(MAKE) src/queue-ms_lb
queue-wf:
	$(MAKE) src/queue-wf
queue-wf-ssmem:
	$(MAKE) src/queue-wf-ssmem
queue-k-segment:
	$(MAKE) src/queue-k-segment
lcrq:
	$(MAKE) src/lcrq
faaaq:
	$(MAKE) src/faaaq
ms:
	$(MAKE) src/ms

multi-ct-faa_ran:
	$(MAKE) src/multi-counter-faa_random-relaxed
multi-ct_ran:
	$(MAKE) src/multi-counter_random-relaxed
multi-ct_ran2c:
	$(MAKE) "CHOICES=two" src/multi-counter_random-relaxed
multi-ct_ran4c:
	$(MAKE) "CHOICES=four" src/multi-counter_random-relaxed
multi-ct_ran8c:
	$(MAKE) "CHOICES=eight" src/multi-counter_random-relaxed
2Dc-counter:
	$(MAKE) src/2Dc-counter
2Dd-counter:
	$(MAKE) src/2Dd-counter
counter-cas:
	$(MAKE) src/counter-cas
single-faa:
	$(MAKE) src/single-faa


2D: 2Dc 2Dd
2Dc: 2Dc-counter 2Dc-stack 2Dc-stack_optimized 2Dc-stack_elastic-lpw
2Dd: 2Dd-counter 2Dd-stack 2Dd-queue_optimized 2Dd-queue 2Dd-queue_elastic-lpw 2Dd-queue_elastic-law #2Dd-deque
multi_ran: multi-ct-faa_ran multi-ct_ran multi-st_ran multi-ct_ran2c multi-st_ran2c multi-st_ran4c multi-ct_ran4c multi-st_ran8c multi-ct_ran8c
external_queues: queue-ms_lb queue-wf queue-wf-ssmem queue-k-segment lcrq faaaq ms
external_stacks: stack-treiber stack-elimination stack-k-segment
external_counters: counter-cas single-faa
dcbo: dcbo-ms simple-dcbo-ms dcbo-faaaq simple-dcbo-faaaq dcbo-lcrq simple-dcbo-lcrq dcbo-wfqueue simple-dcbo-wfqueue
dcbl: dcbl-ms simple-dcbl-ms dcbl-faaaq simple-dcbl-faaaq dcbl-lcrq simple-dcbl-lcrq dcbl-wfqueue simple-dcbl-wfqueue

clean:
	$(MAKE) -C src/queue-ms_lb clean
	$(MAKE) -C src/queue-wf clean
	$(MAKE) -C src/queue-wf-ssmem clean
	$(MAKE) -C src/queue-k-segment clean
	$(MAKE) -C src/2Dd-queue clean
	$(MAKE) -C src/2Dd-queue_optimized clean
	$(MAKE) -C src/2Dd-queue_elastic-lpw clean
	$(MAKE) -C src/2Dd-queue_elastic-law clean
	$(MAKE) -C src/dcbo-ms clean
	$(MAKE) -C src/dcbo-ms "HEURISTIC=LENGTH" clean
	$(MAKE) -C src/simple-dcbo-ms clean
	$(MAKE) -C src/simple-dcbo-ms "HEURISTIC=LENGTH" clean
	$(MAKE) -C src/dcbo-faaaq clean
	$(MAKE) -C src/dcbo-faaaq "HEURISTIC=LENGTH" clean
	$(MAKE) -C src/simple-dcbo-faaaq clean
	$(MAKE) -C src/simple-dcbo-faaaq "HEURISTIC=LENGTH" clean
	$(MAKE) -C src/dcbo-lcrq clean
	$(MAKE) -C src/dcbo-lcrq "HEURISTIC=LENGTH" clean
	$(MAKE) -C src/simple-dcbo-lcrq clean
	$(MAKE) -C src/simple-dcbo-lcrq "HEURISTIC=LENGTH" clean
	$(MAKE) -C src/dcbo-wfqueue clean
	$(MAKE) -C src/dcbo-wfqueue "HEURISTIC=LENGTH" clean
	$(MAKE) -C src/simple-dcbo-wfqueue clean
	$(MAKE) -C src/simple-dcbo-wfqueue "HEURISTIC=LENGTH" clean

	$(MAKE) -C src/faaaq clean
	$(MAKE) -C src/ms clean
	$(MAKE) -C src/lcrq clean

#	$(MAKE) -C src/2Dd-deque clean

	$(MAKE) -C src/stack-treiber clean
	$(MAKE) -C src/stack-elimination clean
	$(MAKE) -C src/stack-k-segment clean
	$(MAKE) -C src/multi-stack_random-relaxed clean
	$(MAKE) -C src/multi-stack_random-relaxed "CHOICES=two" clean
	$(MAKE) -C src/multi-stack_random-relaxed "CHOICES=four" clean
	$(MAKE) -C src/multi-stack_random-relaxed "CHOICES=eight" clean
	$(MAKE) -C src/2Dd-stack clean
	$(MAKE) -C src/2Dc-stack clean
	$(MAKE) -C src/2Dc-stack_optimized clean
	$(MAKE) -C src/2Dc-stack_elastic-lpw clean

	$(MAKE) -C src/multi-counter-faa_random-relaxed clean
	$(MAKE) -C src/multi-counter_random-relaxed clean
	$(MAKE) -C src/multi-counter_random-relaxed "CHOICES=two" clean
	$(MAKE) -C src/multi-counter_random-relaxed "CHOICES=four" clean
	$(MAKE) -C src/multi-counter_random-relaxed "CHOICES=eight" clean
	$(MAKE) -C src/2Dd-counter clean
	$(MAKE) -C src/2Dc-counter clean
	$(MAKE) -C src/counter-cas clean
	$(MAKE) -C src/single-faa clean


	rm -rf build

$(BENCHS):
	$(MAKE) -C $@ $(TARGET)

