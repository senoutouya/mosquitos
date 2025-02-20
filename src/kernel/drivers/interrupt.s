.include "macros.s"

.extern interrupts_handlers
.extern isr_common

.macro isr_noerror num 
.globl isr\num
isr\num:
  save_context

  # TODO: Inline the isr_common call
  movq $0, %rsi
  movq $\num, %rdi
  call isr_common

  restore_context

  iretq
.endm

.macro isr_error num 
.globl isr\num
isr\num:
  save_context

  # TODO: Inline the isr_common call
  movq 128(%rsp), %rsi # Note, if you add 8 to the offset, you get the saved RIP (if there's an error)
  movq $\num, %rdi
  call isr_common

  restore_context

  iretq
.endm

# Actual isr definitions
isr_noerror 0
isr_noerror 1
isr_noerror 2
isr_noerror 3
isr_noerror 4
isr_noerror 5
isr_error 6
isr_noerror 7
isr_error 8
isr_noerror 9
isr_error 10
isr_error 11
isr_error 12
isr_error 13
isr_error 14
isr_noerror 16
isr_error 17
isr_noerror 18
isr_noerror 19
isr_noerror 20
isr_noerror 30
# ISRs 33&34 are handled by the scheduler 
isr_noerror 35
isr_noerror 36
isr_noerror 37
isr_noerror 39
isr_noerror 40
isr_noerror 0xe0
isr_noerror 0xe1
isr_noerror 0xe2
isr_noerror 0xe9
