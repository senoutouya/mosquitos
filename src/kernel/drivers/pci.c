#include <kernel/drivers/apic.h>
#include <kernel/drivers/interrupt.h>
#include <kernel/drivers/pci.h>
#include <kernel/drivers/text_output.h>

#include <acpi.h>

#define PCI_MAX_BUS_NUM 256
#define PCI_MAX_SLOT_NUM 32
#define PCI_MAX_FUNCTION_NUM 8
#define PCI_NUM_INTERRUPT_PORTS 4

#define PCI_MAX_DEVICES 128
#define PCI_MAX_DRIVERS (2 * PCI_MAX_DEVICES)

typedef union {
  struct {
    uint8_t offset;
    uint8_t function_number : 3;
    uint8_t slot : 5;
    uint8_t bus_number;
    uint8_t reserved : 7;
    uint8_t enable : 1;
  } __attribute__((packed)) svalue;
  uint32_t ivalue;

} PCIConfigAddress;

static struct {
  PCIDevice devices[PCI_MAX_DEVICES];
  int num_devices;

  PCIDeviceDriver drivers[PCI_MAX_DRIVERS];
  int num_drivers;

  uint32_t irq_routing_table[PCI_MAX_SLOT_NUM][PCI_NUM_INTERRUPT_PORTS];
  bool use_gsi; // TODO: smx find better ways to determine IRQs
} pci_data;

// TODO: Try to set up MSI again
static void pci_isr() {
kprintf("pci isr%d.", apic_current_irq());
  for (int i = 0; i < pci_data.num_devices; ++i) {
    PCIDevice *device = &pci_data.devices[i];

    if (device->has_interrupts && device->has_driver) {
      device->driver.isr(&device->driver);
    }
  }
}

uint32_t pci_config_read_word(uint8_t bus, uint8_t slot, uint8_t func,
                              uint8_t offset) {
  PCIConfigAddress address;
  address.svalue.offset = offset;
  address.svalue.function_number = func;
  address.svalue.slot = slot;
  address.svalue.bus_number = bus;
  address.svalue.reserved = 0;
  address.svalue.enable = 1;

  io_write_32(0xCF8, address.ivalue);
  return io_read_32(0xCFC);
}

void pci_config_write_word(uint8_t bus, uint8_t slot, uint8_t func,
                           uint8_t offset, uint32_t value) {
  PCIConfigAddress address;
  address.svalue.offset = offset;
  address.svalue.function_number = func;
  address.svalue.slot = slot;
  address.svalue.bus_number = bus;
  address.svalue.reserved = 0;
  address.svalue.enable = 1;

  io_write_32(0xCF8, address.ivalue);
  return io_write_32(0xCFC, value);
}

ACPI_STATUS acpi_system_bus_walk_callback(ACPI_HANDLE Object,
                                          UINT32 NestingLevel UNUSED,
                                          void *Context UNUSED,
                                          void **ReturnValue UNUSED) {
  ACPI_STATUS status;
  ACPI_DEVICE_INFO *device_info;
  assert(AcpiGetObjectInfo(Object, &device_info) == AE_OK);

kprintf("ACPI: %s %x nest%d", &device_info->Name, device_info->Flags, NestingLevel);
	  if (device_info->Valid & ACPI_VALID_SUB)
		  kprintf(", SubsystemId %s", device_info->SubsystemId.String);
	  if (device_info->Valid & ACPI_VALID_UID)
		  kprintf(", UniqueId %s", device_info->UniqueId.String);
	  if (device_info->Valid & ACPI_VALID_HID)
		  kprintf(", HardwareId %s", device_info->HardwareId.String);
      if (device_info->Valid & ACPI_VALID_CID)
	  {
		  kprintf(", compatid:");
          for (int i = 0; i < device_info->CompatibleIdList.Count; ++i)
		  {
			  kprintf(" %s", device_info->CompatibleIdList.Ids[i].String);
          }
	  }
	  kprintf("\n");

  if (device_info->Flags == ACPI_PCI_ROOT_BRIDGE) {
    // TODO figure out how to deal with multiple buses

    static ACPI_PCI_ROUTING_TABLE routing_table[512]; // TODO: smx auto resize: stackoverflow, race condition
    ACPI_BUFFER buffer;
    buffer.Length = sizeof(routing_table);
    buffer.Pointer = &routing_table;

    ACPI_STATUS status = AcpiGetIrqRoutingTable(Object, &buffer);
    if (status != AE_OK)
    	kprintf("AcpiGetIrqRoutingTable status: %s\n", AcpiFormatException(status));
    assert(status == AE_OK);

ACPI_PCI_ROUTING_TABLE* prt_entry = routing_table;

    for (int i = 0; prt_entry->Length > 0; ++i) {
	assert((char*)prt_entry < (char*)routing_table + sizeof(routing_table));
      uint8_t  slot_number = (prt_entry->Address >> 16);

    kprintf("PRT[%d], slot %x pin %d idx %d, src %s\n", i, slot_number, prt_entry->Pin, prt_entry->SourceIndex,prt_entry->Source);
      // TODO: Make sure the SourceIndex isn't referencing a link device
      //pci_data.irq_routing_table[slot_number][(prt_entry->Pin & 0xFF)] =
      //    (prt_entry->SourceIndex & 0xFF);
	//prt_entry = (ACPI_PCI_ROUTING_TABLE*)((char*)prt_entry + prt_entry->Length);

// SourceIndex points to GSI
if (*prt_entry->Source) {
assert(!pci_data.use_gsi);

      ACPI_PCI_ROUTING_TABLE* table = prt_entry;
	  // get the PCI Link Object
	  ACPI_HANDLE linkObject;
      ACPI_STATUS status = AcpiGetHandle(Object, table->Source, &linkObject);
	  if (status != AE_OK)
	  {
		  panic("AcpiGetHandle failed for '%s'", table->Source);
	  };

	  // get the IRQ it is using
	  ACPI_BUFFER resbuf;
	  resbuf.Length = ACPI_ALLOCATE_BUFFER;
	  resbuf.Pointer = NULL;

	  status = AcpiGetCurrentResources(linkObject, &resbuf);
	  if (status != AE_OK)
	  {
		  panic("AcpiGetCurrentResources failed for '%s'", table->Source);
	  };

	  char* rscan = (char*)resbuf.Pointer;
	  int devIRQ = -1;

	  while (1)
	  {
		  ACPI_RESOURCE* res = (ACPI_RESOURCE*)rscan;
		  if (res->Type == ACPI_RESOURCE_TYPE_END_TAG)
		  {
			  break;
		  }

		  if (res->Type == ACPI_RESOURCE_TYPE_IRQ)
		  {
			  devIRQ = res->Data.Irq.Interrupts[table->SourceIndex];
			  break;
		  }
		  else if (res->Type == ACPI_RESOURCE_TYPE_EXTENDED_IRQ)
		  {
assert(table->SourceIndex==0);

			  devIRQ = res->Data.ExtendedIrq.Interrupts[table->SourceIndex];
const char* resSrc = res->Data.ExtendedIrq.ResourceSource.StringPtr;
//kprintf("ExtendedIrq Slot%d pin%d IRQ#%d '%s' ", (int)slot_number, prt_entry->Pin, devIRQ, table->Source);
pci_data.irq_routing_table[slot_number][prt_entry->Pin] = devIRQ;
			  //break;
		  }
else
	kprintf("***RES %d, ", res->Type);

		  rscan += res->Length;
	  }

      ACPI_FREE(resbuf.Pointer);
}
else
{
pci_data.use_gsi = true;
pci_data.irq_routing_table[slot_number][(prt_entry->Pin & 0xFF)] =
          (prt_entry->SourceIndex & 0xFF);
}
prt_entry = (ACPI_PCI_ROUTING_TABLE*)((char*)prt_entry + prt_entry->Length);    
}
  }

  ACPI_FREE(device_info);

  return AE_OK;
}

static void pci_load_irq_routing_table() {
  ACPI_HANDLE system_bus_handle;
  ACPI_STATUS status = AcpiGetHandle(NULL, "\\_SB", &system_bus_handle);
  assert(status == AE_OK);

  void *walk_return_value;
  status = AcpiWalkNamespace(ACPI_TYPE_DEVICE, system_bus_handle, 255,
                             acpi_system_bus_walk_callback, NULL, NULL,
                             &walk_return_value);
  assert(status == AE_OK);
}

UNUSED static void print_pci_device(PCIDevice *device) {
  text_output_printf(
      "[PCI %02X:%02X:%02X] Class Code: %02X:%02X:%02X, %sIRQ#%d, %s\n",
      device->bus, device->slot, device->function, device->class_code,
      device->subclass, device->program_if,
	device->has_interrupts ? "" : "No", device->real_irq,
      device->multifunction ? "multifunc" : "");
}

static PCIDeviceDriver *driver_for_device(PCIDevice *device) {
  for (int i = 0; i < pci_data.num_drivers; ++i) {
    PCIDeviceDriver *driver = &pci_data.drivers[i];
    if (driver->class_code == device->class_code &&
        driver->subclass == device->subclass &&
        driver->program_if == device->program_if) {
      return driver;
    }
  }

  return NULL;
}

static PCIDevice *add_pci_device(uint8_t bus, uint8_t slot, uint8_t function, int depth) {
  uint32_t vendor_word =
      PCI_HEADER_READ_FIELD_WORD(bus, slot, function, vendor_id);
  if (PCI_HEADER_FIELD_IN_WORD(vendor_word, vendor_id) != 0xffff) {
      assert(pci_data.num_devices < countof(pci_data.devices));
    PCIDevice *new_device = &pci_data.devices[pci_data.num_devices++];
    new_device->bus = bus;
    new_device->slot = slot;
    new_device->function = function;

    uint32_t class_field =
        PCI_HEADER_READ_FIELD_WORD(bus, slot, function, class_code);

    new_device->class_code = PCI_HEADER_FIELD_IN_WORD(class_field, class_code);
    new_device->subclass = PCI_HEADER_FIELD_IN_WORD(class_field, subclass);
    new_device->program_if = PCI_HEADER_FIELD_IN_WORD(class_field, program_if);

    uint32_t htype_field =
        PCI_HEADER_READ_FIELD_WORD(bus, slot, function, header_type);

    new_device->header_type =
        PCI_HEADER_FIELD_IN_WORD(htype_field, header_type);
    new_device->multifunction = (new_device->header_type & (1 << 7)) > 0;
    new_device->header_type = new_device->header_type & ~(1 << 7);

    uint32_t interrupt_field =
        PCI_HEADER_READ_FIELD_WORD(bus, slot, function, interrupt_pin);
    uint8_t interrupt_pin =
		PCI_HEADER_FIELD_IN_WORD(interrupt_field, interrupt_pin);
	uint8_t interrupt_line =
		PCI_HEADER_FIELD_IN_WORD(interrupt_field, interrupt_line);

    if (slot < 2 || bus > 0)
      text_output_printf("Loading incorrect IRQ: %i, %i\n", slot, bus);
    assert(interrupt_pin <= PCI_NUM_INTERRUPT_PORTS);
    if (interrupt_pin == 0) {
      new_device->has_interrupts = 0;
    } else {
      new_device->has_interrupts = 1;

	// calculate pin of the root bus, because PCI slots are daisy-chained
      int pin_offset = (depth + (interrupt_pin - 1)) % PCI_NUM_INTERRUPT_PORTS;  // INTA# is 0x01
      new_device->real_irq = pci_data.irq_routing_table[slot][pin_offset];
    }

    PCIDeviceDriver *driver = driver_for_device(new_device);
	text_output_printf("PCI %X:%X:%X Class %02X:%02X:%02X Vendor %X:%X %s %s#%d pin%d #%X\n", 
        bus, slot, function,
		new_device->class_code, new_device->subclass, new_device->program_if,
        PCI_HEADER_FIELD_IN_WORD(vendor_word, vendor_id),
		PCI_HEADER_FIELD_IN_WORD(vendor_word, device_id),
        new_device->multifunction ? "multif" : "",
	new_device->has_interrupts ? "IRQ" : "", new_device->real_irq,
	interrupt_pin, interrupt_line);

    if (new_device->header_type == 0)
	{
		uint32_t subsys_field =
			PCI_HEADER_READ_FIELD_WORD(bus, slot, function, subsystem_id);
	kprintf("Subsys %X:%X\n",
		PCI_HEADER_FIELD_IN_WORD(subsys_field, subsystem_vendor_id),
		PCI_HEADER_FIELD_IN_WORD(subsys_field, subsystem_id)
        );
    }
	else if (new_device->header_type == 1) {
		uint32_t busnumber_field =
			PCI_HEADER_READ_FIELD_WORD(bus, slot, function, base_address_2);
		text_output_printf("SubBusNumber Field %08X\n", busnumber_field);

		uint32_t primary_bus = (busnumber_field & 0xFF);
		uint32_t secondary_bus = ((busnumber_field >> 8) & 0xFF);
        assert(bus == primary_bus);

        pci_enumerate_devices_internal(secondary_bus, ++depth);
    }
    else
        kprintf("Unknown BUS Type %x\n", new_device->header_type);

    if (driver != NULL) {
      text_output_printf("Loading PCI driver \"%s\"...\n", driver->driver_name);
      new_device->driver = *driver;
      new_device->has_driver = true;
      new_device->driver.device = new_device;

      if (new_device->has_interrupts) {
        // TODO: We probably shouldn't remap if this IRQ has already been mapped
      kprintf("ioapic_map %d ", pci_data.use_gsi ? new_device->real_irq : interrupt_line);
        ioapic_map(pci_data.use_gsi ? new_device->real_irq : interrupt_line, PCI_IV, true, true);
      }

      // init() must be called when the device is able to issue commands
      new_device->driver.init(&new_device->driver);
    } else {
      new_device->has_driver = false;
    }

    return new_device;
  }

  return NULL;
}

void pci_enumerate_devices_internal(int bus, int depth) {
	if (depth >= PCI_MAX_BUS_NUM)
		return 0;

	for (int slot = 0; slot < PCI_MAX_SLOT_NUM; slot++) {
		PCIDevice* new_device = add_pci_device(bus, slot, 0, depth);

		if (new_device && new_device->multifunction) {
			for (int func = 1; func < PCI_MAX_FUNCTION_NUM; ++func) {
				add_pci_device(bus, slot, func, depth);
			}
		}
	}
}

void pci_enumerate_devices() {
  REQUIRE_MODULE("pci");
  pci_data.num_devices = 0;

  pci_enumerate_devices_internal(0, 0); // from root
	//kprintf("pci_data.num_devices : %d\n", pci_data.num_devices);
}

void pci_init() {
  REQUIRE_MODULE("interrupt");
  REQUIRE_MODULE("acpi_full");

  memset(&pci_data, 0, sizeof(pci_data));
  pci_data.num_drivers = 0;
  pci_data.use_gsi = false;

  pci_load_irq_routing_table();

  // TODO: Handle each interrupt number with a different ISR for better
  // performance
  interrupt_register_handler(PCI_IV, pci_isr);

  REGISTER_MODULE("pci");
}

PCIDevice *pci_find_device(uint8_t class_code, uint8_t subclass,
                           uint8_t program_if) {
  for (int i = 0; i < pci_data.num_devices; ++i) {
    PCIDevice *device = &pci_data.devices[i];

    if (device->class_code == class_code && device->subclass == subclass &&
        device->program_if == program_if) {
      return device;
    }
  }

  return NULL;
}

void pci_register_device_driver(PCIDeviceDriver driver) {
  REQUIRE_MODULE("pci");

  assert(pci_data.num_drivers < PCI_MAX_DRIVERS);
  pci_data.drivers[pci_data.num_drivers++] = driver;
}
