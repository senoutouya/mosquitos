#include <kernel/kernel_common.h>
#include <kernel/threading/thread.h>

#ifndef _TIMER_H
#define _TIMER_H

#define TIMER_DIVIDER 1193
#define TIMER_FREQUENCY (14318180/12/TIMER_DIVIDER)

void timer_init();
uint64_t timer_ticks();

void timer_thread_stall(uint64_t microseconds);

void timer_thread_sleep(uint64_t milliseconds);
void timer_cancel_thread_sleep(KernelThread *thread);

// void timer_thread_sleep_internal(KernelThread *thread, uin64_t milliseconds);
#endif