#include <common/mem_util.h>
#include <kernel/drivers/apic.h>
#include <kernel/drivers/gdt.h>
#include <kernel/drivers/interrupt.h>
#include <kernel/drivers/text_output.h>
#include <kernel/util.h>

enum IDTDescriptorType { INTERRUPT_GATE = 0b01110, TRAP_GATE = 0b01111 };

// Private structs
static struct IDTDescriptor {
	uint16_t offset_low;
	uint16_t selector;
	uint8_t ist_index : 3;
	uint8_t zero_1 : 5;
	uint8_t type : 5;
	uint8_t privilege_level : 2;
	uint8_t present : 1;
	uint16_t offset_middle;
	uint32_t offset_high;
	uint32_t zero_2;
} __attribute__((packed)) IDT[256];

static struct IDTR {
	uint16_t size;
	uint64_t address;
} __attribute__((packed)) IDTR;

static void (*interrupts_handlers[256])(int);

// Helper functions
static void set_idt_entry(int index, uint64_t isr_address,
	enum IDTDescriptorType type) {
	memset(&IDT[index], 0, sizeof(IDT[index]));
	IDT[index].offset_low = (isr_address & 0xFFFF);
	IDT[index].offset_middle = (isr_address >> 16) & 0xFFFF;
	IDT[index].offset_high = (isr_address >> 32) & 0xFFFFFFFF;

	IDT[index].selector = GDT_KERNEL_CS;
	// IDT[index].ist_index   = 1;
	IDT[index].type = type;
	IDT[index].present = 1;
}

void isr_common(uint64_t num, uint64_t error_code) {
	interrupts_handlers[num](error_code);
	if (num < 0xe0) {
		apic_send_eoi_if_necessary(num);
	}
	else
	{
		// legacy PIC interrupt
		if (num >= 0xe8)
			io_write_8(0xA0, 0x20); // send to slave too
		io_write_8(0x20, 0x20);
	}
}

extern void isr0();
extern void isr1();
extern void isr2();
extern void isr3();
extern void isr4();
extern void isr5();
extern void isr6();
extern void isr7();
extern void isr8();
extern void isr9();
extern void isr10();
extern void isr11();
extern void isr12();
extern void isr13();
extern void isr14();
extern void isr16();
extern void isr17();
extern void isr18();
extern void isr19();
extern void isr20();
extern void isr30();
// TODO: Figure out how to not special case this here
extern void scheduler_timer_isr();  // We have to handle this separately
extern void isr35();
extern void isr36();
extern void isr37();
extern void isr39();
extern void isr40();
extern void isr0xe0();
extern void isr0xe1();
extern void isr0xe2();
extern void isr0xe9();

void isr_pic(int errcode)
{
	//kprintf("isr_pic %d ", apic_current_irq());
}

// Public functions
void interrupt_init() {
	REQUIRE_MODULE("gdt");
	REQUIRE_MODULE("apic");

	memset(IDT, 0, sizeof(IDT));

	// Exceptions (trap gates)
	set_idt_entry(0, (uint64_t)isr0, TRAP_GATE);
	set_idt_entry(1, (uint64_t)isr1, TRAP_GATE);
	set_idt_entry(2, (uint64_t)isr2, TRAP_GATE);
	set_idt_entry(3, (uint64_t)isr3, TRAP_GATE);
	set_idt_entry(4, (uint64_t)isr4, TRAP_GATE);
	set_idt_entry(5, (uint64_t)isr5, TRAP_GATE);
	set_idt_entry(6, (uint64_t)isr6, TRAP_GATE);
	set_idt_entry(7, (uint64_t)isr7, TRAP_GATE);
	set_idt_entry(8, (uint64_t)isr8, TRAP_GATE);
	set_idt_entry(9, (uint64_t)isr9, TRAP_GATE);
	set_idt_entry(10, (uint64_t)isr10, TRAP_GATE);
	set_idt_entry(11, (uint64_t)isr11, TRAP_GATE);
	set_idt_entry(12, (uint64_t)isr12, TRAP_GATE);
	set_idt_entry(13, (uint64_t)isr13, TRAP_GATE);
	set_idt_entry(14, (uint64_t)isr14, TRAP_GATE);
	set_idt_entry(16, (uint64_t)isr16, TRAP_GATE);
	set_idt_entry(17, (uint64_t)isr17, TRAP_GATE);
	set_idt_entry(18, (uint64_t)isr18, TRAP_GATE);
	set_idt_entry(19, (uint64_t)isr19, TRAP_GATE);
	set_idt_entry(20, (uint64_t)isr20, TRAP_GATE);
	set_idt_entry(30, (uint64_t)isr30, TRAP_GATE);

	// IRQs (interrupt gates)
	set_idt_entry(SCHEDULER_TIMER_IV, (uint64_t)scheduler_timer_isr,
		INTERRUPT_GATE);  // Local APIC timer (scheduler)

	set_idt_entry(KEYBOARD_IV, (uint64_t)isr35, INTERRUPT_GATE);   // Keyboard
	set_idt_entry(PIC_TIMER_IV, (uint64_t)isr36, INTERRUPT_GATE);  // PIC timer
	set_idt_entry(PCI_IV, (uint64_t)isr37, INTERRUPT_GATE);        // PCI ISR

	// Something is weird about IV 38...
	set_idt_entry(LOCAL_APIC_CALIBRATION_IV, (uint64_t)isr39,
		INTERRUPT_GATE);  // Local APIC timer (calibration)
//set_idt_entry(IDE_IV, (uint64_t)isr40, INTERRUPT_GATE);        // IDE ISR
	set_idt_entry(0xe0, (uint64_t)isr0xe0, INTERRUPT_GATE);
	set_idt_entry(0xe1, (uint64_t)isr0xe1, INTERRUPT_GATE);
	set_idt_entry(0xe2, (uint64_t)isr0xe2, INTERRUPT_GATE);
	set_idt_entry(0xe9, (uint64_t)isr0xe9, INTERRUPT_GATE);
	interrupt_register_handler(0xe0, isr_pic);
	interrupt_register_handler(0xe1, isr_pic);
	interrupt_register_handler(0xe2, isr_pic);
	interrupt_register_handler(0xe9, isr_pic);

	IDTR.size = sizeof(IDT) - 1;
	IDTR.address = (uint64_t)&IDT[0];

	__asm__("lidt %0" : : "m"(IDTR));

	REGISTER_MODULE("interrupt");
}

void interrupt_register_handler(int index, void (*handler)()) {
	if (index < 0 || index >= 256) return;

	interrupts_handlers[index] = handler;
}
