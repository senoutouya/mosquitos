#include <kernel/drivers/filesystem_tree.h>
#include <kernel/drivers/filesystems/mfs.h>
#include <kernel/drivers/text_output.h>

#define MAX_FILESYSTEMS 10

static struct FilesystemTreeData {
	Filesystem filesystems[MAX_FILESYSTEMS];
	size_t num_filesystems;
} fs_tree_data;

static FilesystemError add_filesystem(const char* const filesystem_id,
	const void* initialization_data) {
	assert(fs_tree_data.num_filesystems < MAX_FILESYSTEMS);
	Filesystem* new_filesystem =
		&fs_tree_data.filesystems[fs_tree_data.num_filesystems++];
	if (!filesystem_create(filesystem_id, new_filesystem)) {
		return FS_ERROR_NOT_FOUND;
	}

	text_output_printf("Adding filesystem to tree: %s at index %i\n",
		filesystem_id, fs_tree_data.num_filesystems - 1);

	return new_filesystem->init(new_filesystem, initialization_data);
}

void print_pci_device(PCIDevice* device);

void test_ata_ide_identity(int prog_if) {
	bool found = false;
	if (pci_find_device(0x01, 0x01, prog_if))
		found = true;
	else if (pci_find_device(0x01, 0x01, prog_if))
		found = true;
	else if (pci_find_device(0x01, 0x01, prog_if))
		found = true;
	if (found)
		kprintf("found IDE device CLS: 01:01:%X. TODO: driver\n", prog_if);
	else
		return;
	// TODO: crash io_write_8(0x1F7, 0xEC); if no disk
	/*
	  // master IDE IDENTITY with ATA command
	  io_write_8(0x1F6, 0xA0);
	  io_write_8(0x1F2, 0);
	  io_write_8(0x1F3, 0);
	  io_write_8(0x1F4, 0);
	  io_write_8(0x1F5, 0);
	  io_write_8(0x1F7, 0xEC);

	  uint8_t status = io_read_8(0x1F7);
	if (status == 0)
		kprintf("IDE master not exist\n");
	else
	{
	  while (status & 0x80)
		  status = io_read_8(0x1F7);
	  kprintf("status %x ", status);
	  for (int i = 0; i < 256; ++i)
	  {
		  uint16_t res = io_read_16(0x1F0);
		  kprintf("%c%c", res >> 8, res & 0xFF);
	  }
	}
	*/
}

void filesystem_tree_init() {
	REQUIRE_MODULE("ahci");
	REQUIRE_MODULE("pci");

	fs_tree_data.num_filesystems = 0;

	test_ata_ide_identity(0x80);
	test_ata_ide_identity(0x8A);
	test_ata_ide_identity(0x8F);

	PCIDevice* device = pci_find_device(0x01, 0x06, 0x01);
	if (device == NULL || !device->has_driver) {
		return;
	}

	PCIDeviceDriver* driver = &device->driver;
	uint64_t num_devices = -1;
	PCIDeviceDriverError ahci_error =
		driver->execute_command(driver, AHCI_COMMAND_NUM_DEVICES, NULL, 0,
			&num_devices, sizeof(num_devices));
	if (ahci_error != PCI_ERROR_NONE) return;

	for (uint64_t i = 0; i < num_devices; ++i) {
		const struct AHCIDeviceInfoCommand info_request = { .device_id = i };
		struct AHCIDeviceInfo info;
		ahci_error =
			driver->execute_command(driver, AHCI_COMMAND_DEVICE_INFO, &info_request,
				sizeof(info_request), &info, sizeof(info));
		assert(ahci_error == PCI_ERROR_NONE);

		kprintf("AHCI type%d %s,%s,%s,%s %x %x\n", info.device_type, info.serial_number, info.firmware_revision,
			info.media_serial_number, info.model_number, info.logical_sector_size, info.num_sectors);
		if (info.device_type == AHCI_DEVICE_SATA) {
			const MFSSATAInitData initialization_data = { .driver = driver,
														 .device_id = i };
			// TODO: Discover what type of FS is on the device and load that driver,
			// instead of always loading MFS_S.
			add_filesystem("MFS_S", &initialization_data);
		}
	}
}

FilesystemError filesystem_tree_add(const char* const identifier,
	const void* initialization_data,
	Filesystem** filesystem) {
	*filesystem = &fs_tree_data.filesystems[fs_tree_data.num_filesystems];
	return add_filesystem(identifier, initialization_data);
}

Filesystem* filesystem_tree_get(size_t index) {
	if (index >= fs_tree_data.num_filesystems) {
		return NULL;
	}

	return &fs_tree_data.filesystems[index];
}

size_t filesystem_tree_size() { return fs_tree_data.num_filesystems; }
