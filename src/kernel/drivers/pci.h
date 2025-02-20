#include <kernel/drivers/pci_drivers/pci_device_driver.h>
#include <kernel/kernel_common.h>
#include <kernel/util.h>

#ifndef _PCI_H
#define _PCI_H

typedef struct {
  uint16_t vendor_id, device_id;
  uint16_t command, status;
  uint8_t revision_id, program_if, subclass, class_code;
  uint8_t cache_line_size, latency_timer, header_type, bist;

  uint32_t base_address_0;
  uint32_t base_address_1;
  uint32_t base_address_2;
  uint32_t base_address_3;
  uint32_t base_address_4;
  uint32_t base_address_5;

  uint32_t cardbus_cis_pointer;
  uint16_t subsystem_vendor_id, subsystem_id;
  uint32_t expansion_rom_base_address;
  uint8_t capability_pointer;
  uint32_t reserved : 24;
  uint32_t reserved2;

  uint8_t interrupt_line, interrupt_pin, min_grant, max_latency;
} __attribute__((packed)) PCIGenericConfigHeader;

typedef struct _PCIDevice {
  uint8_t program_if, subclass, class_code;
  uint8_t bus, slot, function;
  uint8_t multifunction, header_type;
  uint8_t has_interrupts : 1;
  uint8_t other_flags : 7;
  uint32_t real_irq;
  PCIDeviceDriver driver;
  bool has_driver;
} PCIDevice;

// Functions

void pci_init();
void pci_enumerate_devices();
void pci_enumerate_devices_internal(int bus, int root, int depth);

uint32_t pci_config_read_word(uint8_t bus, uint8_t slot, uint8_t func,
                              uint8_t offset);
void pci_config_write_word(uint8_t bus, uint8_t slot, uint8_t func,
                           uint8_t offset, uint32_t value);
PCIDevice* pci_find_device(uint8_t class_code, uint8_t subclass,
                           uint8_t program_if);

void pci_register_device_driver(PCIDeviceDriver driver);

// Macros
#define PCI_OFFSET_FOR_HDR_FIELD(field) \
  (offsetof(PCIGenericConfigHeader, field) & ~0b11)
#define PCI_OFFSET_WITHIN_HDR_FIELD(field) \
  (offsetof(PCIGenericConfigHeader, field) - PCI_OFFSET_FOR_HDR_FIELD(field))

#define PCI_HEADER_FIELD_IN_WORD(word, field)              \
  (field_in_word(word, PCI_OFFSET_WITHIN_HDR_FIELD(field), \
                 member_size(PCIGenericConfigHeader, field)))
#define PCI_HEADER_READ_FIELD_WORD(bus, slot, func, field) \
  (pci_config_read_word(bus, slot, func, PCI_OFFSET_FOR_HDR_FIELD(field)))

#endif
