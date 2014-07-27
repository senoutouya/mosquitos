#include <efi.h>
#include <efilib.h>

#include "keyboard_controller.h"
#include "text_output.h"
#include "util.h"
#include "pic.h"

struct GDTEntry {
  uint16_t limit_low;         // The lower 16 bits of the limit.
  uint16_t base_low;          // The lower 16 bits of the base.
  uint8_t  base_middle;       // The next 8 bits of the base.
  uint8_t  access;            //type: 4, s: 1, dpl: 2, p: 1;
  uint8_t  limit_high: 4, flags:4; //avl: 1, l: 1, d: 1, g: 1;
  uint8_t  base_high;         // The last 8 bits of the base.
} __attribute__((packed)) GDT[5];

struct GDTR {
  uint16_t size;
  uint64_t address;
} __attribute__((packed)) GDTR;

struct IDTEntry {
  uint16_t base_low;
  uint16_t selector;
  uint8_t  zero_1;
  uint8_t  attributes;
  uint16_t base_middle;
  uint32_t base_high;
  uint32_t zero_2;
} __attribute__((packed)) IDT[256];

struct IDTR {
  uint16_t size;
  uint64_t address;
} __attribute__((packed)) IDTR;

static void set_gdt_entry(int index, uint32_t base, uint32_t limit, uint8_t access, uint8_t flags) {
   GDT[index].base_low    = (base & 0xFFFF);
   GDT[index].base_middle = (base >> 16) & 0xFF;
   GDT[index].base_high   = (base >> 24) & 0xFF;

   GDT[index].limit_low   = (limit & 0xFFFF);
   GDT[index].limit_high  = (limit >> 16) & 0x0F;

   GDT[index].flags       = flags;
   GDT[index].access      = access;
}

static void set_idt_entry(int index, uint64_t base, uint16_t selector, uint8_t attributes) {
   IDT[index].base_low    = (base & 0xFFFF);
   IDT[index].base_middle = (base >> 16) & 0xFFFF;
   IDT[index].base_high   = (base >> 32) & 0xFFFFFFFF;

   IDT[index].selector    = selector;
   IDT[index].attributes  = attributes;

   IDT[index].zero_1      = 0;
   IDT[index].zero_2      = 0;
}

void double_fault_isr() {
  text_output_print("Double fault!\n");

  __asm__ ("iretq");
}

void default_isr() {
  outb(0x20, 0x20); // Acknowledge interrupts
  outb(0xA0, 0x20);
  __asm__ ("iretq");
}

// void gpe_isr() {

//   text_output_print("General Protection Exception!\n");

//   __asm__ ("hlt");
// }

void timer_isr() {
  text_output_print("Timer!\n");
  // outb(0x20, 0x20); // Acknowledge interrupts

  // __asm__ ("iretq");
}

void keyboard_isr() {
  uint8_t status = inb(0x64);
  uint8_t key = inb(0x60);

  char buf[] = { key + '0', status + '0', '\n', 0 };
  text_output_print(buf);
  text_output_print("Keyboard\n");

  outb(0x20, 0x20); // Acknowledge interrupts
  // __asm__ ("hlt");

  __asm__ ("iretq");
}

void isr() {
  text_output_print("ISR\n");
}

void print_something() {
  text_output_print("ISR!!\n");
}

extern void isr1();
extern void gpe_isr();

uint64_t get_esp() {
  uint64_t ret;
  __asm__ ("mov %%rsp, %0" : "=r" (ret));

  return ret;
}

extern void gdt_flush();

void keyboard_controller_init() {
  char buf[20];
  int2str((uint64_t)IDT, buf, sizeof(buf));
  text_output_print("IDT addres: ");
  text_output_print(buf);
  text_output_print("\n");

  text_output_print("Attempting to setup GDT and interrupts.\n");

  // Setup GDT

  set_gdt_entry(0, 0, 0, 0, 0);                // Null segment
  set_gdt_entry(1, 0, 0xFFFFFFFF, 0b10011010, 0b1010); // Code segment
  set_gdt_entry(2, 0, 0xFFFFFFFF, 0b10010010, 0b1010); // Data segment
  set_gdt_entry(3, 0, 0xFFFFFFFF, 0b11111010, 0b1010); // User mode code segment
  set_gdt_entry(4, 0, 0xFFFFFFFF, 0b11110010, 0b1010); // User mode data segment

  GDTR.size = sizeof(GDT) - 1;
  GDTR.address = (uint64_t)&GDT[0];

  // __asm__ ("lgdt %0" : : "m" (GDTR));
  gdt_flush();

  text_output_print("Loaded GDT.\n");

  pic_remap(0x20, 0x28);
  outb(0x21,0b11111101); // Enable keyboard IRQ
  outb(0xa1,0xff);

  // for (int i = 0; i < 256; ++i) {
  //   set_idt_entry(i, (uint64_t)isr, 0x08, 0b10001110);
  // }

  // GPE
  set_idt_entry(0xD, (uint64_t)gpe_isr, 0x08, 0b10001110); 

  set_idt_entry(0x03, (uint64_t)double_fault_isr, 0x08, 0b10001110);

  set_idt_entry(0x20, (uint64_t)isr1, 0x08, 0b10001110);
  set_idt_entry(0x21, (uint64_t)isr1, 0x08, 0b10001110);

  IDTR.size = sizeof(IDT) - 1;
  IDTR.address = (uint64_t)&IDT[0];

  __asm__ ("lidt %0" : : "m" (IDTR));

  text_output_print("Loaded IDT.\n");

  __asm__ ("sti");

  // __asm__ ("int $0x20");
  // __asm__ ("int $0x20");
  // __asm__ ("int $0x20");

  text_output_print("Sent interrupt.\n");
}
