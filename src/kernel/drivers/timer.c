#include <kernel/drivers/timer.h>
#include <kernel/drivers/text_output.h>
#include <kernel/drivers/apic.h>
#include <kernel/drivers/interrupt.h>
#include <kernel/util.h>
#include <kernel/datastructures/list.h>
#include <kernel/memory/kmalloc.h>
#include <kernel/threading/scheduler.h>

#define TIMER_IRQ 2

struct waiting_thread {
  ListEntry entry;

  KernelThread *thread;
  uint64_t wake_time;
};

static volatile struct {
  volatile uint64_t ticks; // Won't overflow for 5e8 ticks
  uint64_t cycles_per_tick;
  List waiting_threads;
} timer_data;

static inline void wake_waiting_thread(struct waiting_thread *wt) {
  list_remove(&timer_data.waiting_threads, &wt->entry);
  thread_wake(wt->thread);
  kfree(wt);
}

void timer_isr() {
  uint64_t current_ticks = __sync_add_and_fetch(&timer_data.ticks, 1);

  ListEntry *current = list_head(&timer_data.waiting_threads);
  while (current) {
    struct waiting_thread *current_waiting_thread = container_of(current, struct waiting_thread, entry);
    ListEntry *next = list_next(current);

    if (current_ticks >= current_waiting_thread->wake_time) {
      wake_waiting_thread(current_waiting_thread);
    }

    current = next;
  }
}

uint64_t timer_ticks() {
  return timer_data.ticks;
}

void timer_init() {
  REQUIRE_MODULE("interrupt");

  list_init(&timer_data.waiting_threads);

  interrupt_register_handler(PIC_TIMER_IV, timer_isr);

  // Use Legacy PIC Timer
  // TODO: Use HPET eventually
  io_write_8(0x43, 0b00110100);

  // Set up timer to have frequency TIMER_FREQUENCY
  io_write_8(0x40, TIMER_DIVIDER & 0xff);
  io_write_8(0x40, TIMER_DIVIDER >> 8);

  // Enable I/O APIC routing for PIC timer
  ioapic_map(TIMER_IRQ, PIC_TIMER_IV, false, false);

  // Calibrate cycles_per_tick
  timer_data.cycles_per_tick = 0;
  timer_data.ticks = 0;
  while (timer_data.ticks < 128) {
    timer_data.cycles_per_tick++;
    __sync_synchronize();
  }
  timer_data.cycles_per_tick >>= 5; // We execute about 4x more instructions in this loop than in the timer_thread_stall() loop

  REGISTER_MODULE("timer");
}

void timer_thread_stall(uint64_t microseconds) {
  // TODO: Who knows if this is even remotely precise. Also, it can get interrupted by things

  uint64_t cycles = microseconds * TIMER_FREQUENCY * timer_data.cycles_per_tick / 1e6;
  while (cycles > 0) cycles--;
}

void timer_thread_sleep(uint64_t milliseconds) {
  KernelThread *thread = scheduler_current_thread();

  struct waiting_thread *new_entry = kmalloc(sizeof(struct waiting_thread));

  new_entry->thread = thread;

  uint64_t ticks = milliseconds * 1000 / TIMER_FREQUENCY;
  new_entry->wake_time = timer_data.ticks + ticks;

  bool interrupts_enabled = interrupts_status();
  cli();

  list_push_front(&timer_data.waiting_threads, &new_entry->entry);

  // This won't return until the thread wakes up
  // NOTE: Interrupts will be disabled when it returns
  thread_sleep(thread);

  // Only re-enable interrupts if they were enabled before
  if (interrupts_enabled) sti();

}

void timer_cancel_thread_sleep(KernelThread *thread) {
  bool interrupts_enabled = interrupts_status();
  cli();

  ListEntry *current = list_head(&timer_data.waiting_threads);
  while (current) {
    struct waiting_thread *current_waiting_thread = container_of(current, struct waiting_thread, entry);
    ListEntry *next = list_next(current);

    if (current_waiting_thread->thread == thread) {
      wake_waiting_thread(current_waiting_thread);
    }

    current = next;
  }

  // Only re-enable interrupts if they were enabled before
  if (interrupts_enabled) sti();
}

