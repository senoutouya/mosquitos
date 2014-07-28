#include "apic.h"
#include "util.h"
#include "text_output.h"

#define PIC1    0x20    /* IO base address for master PIC */
#define PIC2    0xA0    /* IO base address for slave PIC */
#define PIC1_COMMAND  PIC1
#define PIC1_DATA (PIC1+1)
#define PIC2_COMMAND  PIC2
#define PIC2_DATA (PIC2+1)
#define PIC_EOI   0x20    /* End-of-interrupt command code */

#define ICW1_ICW4 0x01    /* ICW4 (not) needed */
#define ICW1_SINGLE 0x02    /* Single (cascade) mode */
#define ICW1_INTERVAL4  0x04    /* Call address interval 4 (8) */
#define ICW1_LEVEL  0x08    /* Level triggered (edge) mode */
#define ICW1_INIT 0x10    /* Initialization - required! */
 
#define ICW4_8086 0x01    /* 8086/88 (MCS-80/85) mode */
#define ICW4_AUTO 0x02    /* Auto (normal) EOI */
#define ICW4_BUF_SLAVE  0x08    /* Buffered mode/slave */
#define ICW4_BUF_MASTER 0x0C    /* Buffered mode/master */
#define ICW4_SFNM 0x10    /* Special fully nested (not) */

static uint32_t *apic_base = (uint32_t *)0xfee00000;
static uint32_t *ioapic_index = (uint32_t *)0xfec00000;
static uint32_t *ioapic_data = (uint32_t *)0xfec00010;

void ioapic_write(int index, uint32_t value) {
  *ioapic_index = index;
  *ioapic_data = value;
}

uint32_t ioapic_read(int index) {
  *ioapic_index = index;
  return *ioapic_data;
}

void apic_write(int index, uint32_t value) {
  apic_base[index * 4] = value; // Registers are 16 bytes wide
}

uint32_t apic_read(int index) {
  return *(apic_base + index * 4); // Registers are 16 bytes wide
}

void ioapic_map(uint8_t irq_index, uint8_t idt_index) {
  const uint32_t low_index = 0x10 + irq_index * 2;
  const uint32_t high_index = 0x10 + irq_index * 2 + 1;

  uint32_t high = ioapic_read(high_index);
  // Set APIC ID
  high &= ~0xff000000;
  high |= apic_read(0x02) << 24; // Local APIC id
  ioapic_write(high_index, high);

  uint32_t low = ioapic_read(low_index);

  // Unmask the IRQ
  low &= ~(1<<16);

  // Set to physical delivery mode
  low &= ~(1<<11);

  // Set to fixed delivery mode
  low &= ~0x700;

  // Set delivery vector
  low &= ~0xff;
  low |= idt_index;

  ioapic_write(low_index, low);
}

void apic_init() {
  // Disable legacy PIC

  /* Set ICW1 */
  outb(PIC1_COMMAND, ICW1_INIT+ICW1_ICW4);
  outb(PIC2_COMMAND, ICW1_INIT+ICW1_ICW4);

  /* Set ICW2 (IRQ base offsets) */
  outb(PIC1_DATA, 0xe0);
  outb(PIC2_DATA, 0xe8);

  /* Set ICW3 */
  outb(PIC1_DATA, 4);
  outb(PIC2_DATA, 2);

  /* Set ICW4 */
  outb(PIC1_DATA, ICW4_8086);
  outb(PIC2_DATA, ICW4_8086);

  /* Disable all interrupts */
  outb(PIC1_DATA, 0xff);
  outb(PIC2_DATA, 0xff);

  // Enable APIC MSR
  uint64_t apic_msr = read_msr(0x1b);
  apic_msr |= (1 << 11);
  write_msr(0x1b, apic_msr);

  // Enable APIC flag in SIVR
  uint32_t val = apic_read(0x0f);
  val |= (1<<8);
  apic_write(0x0f, val);
}

void apic_send_eoi() {
  apic_write(0x0b, 1);
}
