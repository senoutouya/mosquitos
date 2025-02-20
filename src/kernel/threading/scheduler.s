.include "macros.s"

# NOTE: Should be called from inside an interrupt handler
# Assumes ss, rsp, rflags, cs, and rip are pushed on the
# stack. WILL remove these values (must be restored with
# scheduler_load_thread())
# NOTE: The caller MUST push rdi to the stack beforehand
.macro save_thread
  # rdi contains the address of ss in the KernelThread struct
  # The previous thread's rdi is pushed to the stack

  mov   %rax, 0x28(%rdi) # Save rax since we use it

  pop   %rax
  mov   %rax, 0x50(%rdi) # rdi

  pop   %rax
  mov   %rax, 0x20(%rdi) # rip
  pop   %rax
  mov   %rax, 0x18(%rdi) # cs
  pop   %rax
  mov   %rax, 0x10(%rdi) # rflags
  pop   %rax
  mov   %rax, 0x08(%rdi) # rsp
  pop   %rax
  mov   %rax, 0x00(%rdi) # ss

  # uint64_t rax, rbx, rcx, rdx, rsi, rdi, rbp;
  # uint64_t r8, r9, r10, r11, r12, r13, r14, r15;
  # uint64_t ds, es, fs, gs;

  mov   %rbx, 0x30(%rdi)
  mov   %rcx, 0x38(%rdi)
  mov   %rdx, 0x40(%rdi)
  mov   %rsi, 0x48(%rdi)
  # Skip rdi because we saved it earlier
  mov   %rbp, 0x58(%rdi)

  mov   %r8, 0x60(%rdi)
  mov   %r9, 0x68(%rdi)
  mov   %r10, 0x70(%rdi)
  mov   %r11, 0x78(%rdi)
  mov   %r12, 0x80(%rdi)
  mov   %r13, 0x88(%rdi)
  mov   %r14, 0x90(%rdi)
  mov   %r15, 0x98(%rdi)

  mov   %ds, 0xA0(%rdi)
  mov   %es, 0xA8(%rdi)
  mov   %fs, 0xB0(%rdi)
  mov   %gs, 0xB8(%rdi)

.endm

.extern scheduler_data

.globl scheduler_timer_isr
scheduler_timer_isr:
  push %rdi

  # Current thread is first element of scheduler_data struct
  mov   (scheduler_data), %rdi

  # Only save the current thread if the pointer is non-NULL
  test %rdi, %rdi
  jz no_save
  save_thread

  no_save:

  # NOTE: We can do whatever we want to registers now, they are all saved
  # NOTE 2: Theoretically this is true, but empirically it is false

  call  apic_send_eoi

  call  scheduler_set_next # Set current thread to next thread

  # Load new thread
  mov   (scheduler_data), %rdi

  # rdi contains the address of ss in the KernelThread struct
  push  0x00(%rdi) # ss
  push  0x08(%rdi) # rsp
  push  0x10(%rdi) # rflags
  push  0x18(%rdi) # cs
  push  0x20(%rdi) # rip

  # uint64_t rax, rbx, rcx, rdx, rsi, rdi, rbp;
  # uint64_t r8, r9, r10, r11, r12, r13, r14, r15;
  # uint64_t ds, es, fs, gs;

  mov   0x28(%rdi), %rax
  mov   0x30(%rdi), %rbx
  mov   0x38(%rdi), %rcx
  mov   0x40(%rdi), %rdx
  mov   0x48(%rdi), %rsi
  # Skip rdi for the moment, becuase we're using it
  mov   0x58(%rdi), %rbp

  mov   0x60(%rdi), %r8
  mov   0x68(%rdi), %r9
  mov   0x70(%rdi), %r10
  mov   0x78(%rdi), %r11
  mov   0x80(%rdi), %r12
  mov   0x88(%rdi), %r13
  mov   0x90(%rdi), %r14
  mov   0x98(%rdi), %r15

  mov   0xA0(%rdi), %ds
  mov   0xA8(%rdi), %es
  mov   0xB0(%rdi), %fs
  mov   0xB8(%rdi), %gs

  # Set rdi now that we're done with it
  mov   0x50(%rdi), %rdi

  iretq # Perform the switch
