/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "batt_smbus.h"

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[]);

enum BATT_SMBUS_BUS {
	BATT_SMBUS_BUS_ALL = 0,
	BATT_SMBUS_BUS_I2C_INTERNAL,
	BATT_SMBUS_BUS_I2C_EXTERNAL,
};

/**
 * Local functions in support of the shell command.
 */

namespace batt_smbus
{

struct batt_smbus_bus_option {
	enum BATT_SMBUS_BUS busid;
	const char *devpath;
	BATT_SMBUS_constructor interface_constructor;
	uint8_t busnum;
	BATT_SMBUS	*dev;
} bus_options[] = {
	{ BATT_SMBUS_BUS_I2C_EXTERNAL, "/dev/batt_smbus_ext", &BATT_SMBUS_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_ONBOARD
	{ BATT_SMBUS_BUS_I2C_INTERNAL, "/dev/batt_smbus_int", &BATT_SMBUS_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
};

#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

int start(enum BATT_SMBUS_BUS busid);
bool start_bus(struct batt_smbus_bus_option &bus);
struct batt_smbus_bus_option &find_bus(enum BATT_SMBUS_BUS busid);
int reset(enum BATT_SMBUS_BUS busid);
int info();

void usage();
int manufacture_date();
int manufacturer_name();
int serial_number();

/**
 * start driver for a specific bus option
 */
bool start_bus(struct batt_smbus_bus_option &bus)
{
	PX4_INFO("starting %s", bus.devpath);

	if (bus.dev != nullptr) {
		PX4_WARN("bus option already started");
		return false;
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new BATT_SMBUS(interface, bus.devpath);

	if (bus.dev != nullptr && PX4_OK != bus.dev->init()) {
		PX4_WARN("init failed");
		delete bus.dev;
		bus.dev = nullptr;
		return false;
	}

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
int start(enum BATT_SMBUS_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == BATT_SMBUS_BUS_ALL && bus_options[i].dev != NULL) {
			// This device is already started.
			PX4_INFO("Smart battery %d already started", bus_options[i].dev);
			continue;
		}

		if (busid != BATT_SMBUS_BUS_ALL && bus_options[i].busid != busid) {
			// Not the one that is asked for.
			continue;
		}

		if (start_bus(bus_options[i])) {
			return PX4_OK;
		}

		PX4_INFO("Smart battery failed to start");
		return PX4_ERROR;
	}

	return PX4_ERROR;
}

/**
 * Find a bus structure for a busid.
 */
struct batt_smbus_bus_option &find_bus(enum BATT_SMBUS_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == BATT_SMBUS_BUS_ALL || busid == bus_options[i].busid) &&
		    bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "Could not find a smart battery: Did you start it?");
	// to satisfy other compilers
	return bus_options[0];
}



int manufacture_date()
{
	int result = -1;
	uint16_t man_date = 0;

	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct batt_smbus_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_INFO("%s", bus.devpath);
			result = bus.dev->manufacture_date(&man_date);
		}
	}

	if (PX4_OK == result) {
		// Convert the uint16_t into human-readable date format.
		uint16_t year = ((man_date >> 9) & 0xFF) + 1980;
		uint8_t month = (man_date >> 5) & 0xF;
		uint8_t day = man_date & 0x1F;
		PX4_INFO("The manufacturer date is: %4d-%02d-%02d", year, month, day);
		return PX4_OK;

	} else {
		PX4_WARN("Unable to read the manufacturer date.");
		return PX4_ERROR;
	}

}

int manufacturer_name()
{
	int result = -1;
	uint8_t man_name[22];

	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct batt_smbus_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_INFO("%s", bus.devpath);
			result = bus.dev->manufacturer_name(man_name, sizeof(man_name));
		}
	}

	if (PX4_OK == result) {
		PX4_INFO("The manufacturer name: %s", man_name);
		return PX4_OK;

	} else {
		PX4_WARN("Unable to read manufacturer name.");
		return PX4_ERROR;
	}
}

int serial_number()
{
	uint16_t serial_num = 0;

	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct batt_smbus_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_INFO("%s", bus.devpath);
			serial_num = bus.dev->get_serial_number();
		}
	}

	PX4_INFO("The serial number: 0x%04x (%d in decimal)", serial_num, serial_num);

	return PX4_OK;
}

void usage()
{
	PX4_INFO("\n\tstart            	 	- Starts the driver."
		 "\n\tstop 				- Stops the driver."
		 "\n\tman_name			- Prints the name of the manufacturer."
		 "\n\tman_date			- Prints the date of manufacture."
		 "\n\tserial_num			- Prints the serial number."
		 "\n\tsbs_info			- Prints the manufacturer name, date, and serial number."
		 "\n\tinfo				- Prints the last report"
		 "\n\tread_word			- Uses the SMbus read-word command. Argument is the command word. (see datasheet)"
		 "\n\twrite_word			- Uses the SMbus write-word command. Argument is the command word (see datasheet)"
		 "\n\tman_read			- Uses the SMbus block-read with ManufacturerAccess() command: cmd_code, length"
		 "\n\tread_flash			- Reads from flash: start address, number of bytes to read"
		 "\n\twrite_flash			- Writes to flash: start address, N number of bytes to write, byte1, byte2, ...byteN"
		 "\n\t"
		 "\n\tWhen writing to flash be sure to separate each byte by a space"
		 "\n\t");
}

} //namespace

int
batt_smbus_main(int argc, char *argv[])
{
	enum BATT_SMBUS_BUS busid = BATT_SMBUS_BUS_I2C_INTERNAL;
	int ch;


	// Jump over start/off/etc and look at options first.
	while ((ch = getopt(argc, argv, "XIA:")) != EOF) {
		switch (ch) {
		case 'X':
			busid = BATT_SMBUS_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = BATT_SMBUS_BUS_I2C_INTERNAL;
			break;

		case 'A':
			busid = BATT_SMBUS_BUS_ALL;
			break;

		default:
			batt_smbus::usage();
			return 0;
		}
	}

	const char *input = argv[optind];

	if (!strcmp(input, "start")) {
		return batt_smbus::start(busid);
	}

	struct batt_smbus::batt_smbus_bus_option &bus = batt_smbus::find_bus(busid);

	if (!strcmp(argv[2], "stop")) {
		delete bus.dev;
		bus.dev = nullptr;
		return 0;
	}

	if (!strcmp(argv[2], "suspend")) {
		bus.dev->stop();
		return 0;
	}

	if (!strcmp(argv[2], "resume")) {
		bus.dev->start();
		return 0;
	}

	if (!strcmp(argv[2], "man_name")) {
		batt_smbus::manufacturer_name();
		return 0;
	}

	if (!strcmp(argv[2], "man_date")) {
		batt_smbus::manufacture_date();
		return 0;
	}

	if (!strcmp(argv[2], "serial_num")) {
		batt_smbus::serial_number();
		return 0;
	}

	if (!strcmp(argv[2], "sbs_info")) {
		batt_smbus::manufacturer_name();
		batt_smbus::manufacture_date();
		batt_smbus::serial_number();
		return 0;
	}

	if (!strcmp(argv[2], "info")) {
		bus.dev->info();
		return 0;
	}

	if (!strcmp(argv[2], "unseal")) {
		bus.dev->unseal();
		return 0;
	}



	if (!strcmp(argv[2], "read_word")) {
		if (argv[3]) {
			uint8_t data[2];

			if (PX4_OK != bus.dev->read_word((uint8_t)atoi(argv[3]), data)) {
				PX4_INFO("Register read failed");
				return 1;

			} else {
				PX4_INFO("Register value: %d %d", data[1], data[0]);
				return 0;
			}
		}

		return 1;
	}

	// cmd code (u16), length (32bytes max)
	if (!strcmp(argv[2], "man_read")) {
		if (argv[3] && argv[4]) {
			uint8_t data[32] = {0};
			uint16_t cmd_code = atoi(argv[3]);
			unsigned length = atoi(argv[4]);

			if (length > 32) {
				PX4_WARN("Data length out of range: Max 32 bytes");
				return 1;
			}

			if (PX4_OK != bus.dev->manufacturer_read(cmd_code, data, length)) {
				PX4_INFO("Block write failed");
				return 1;

			} else {
				return 0;
			}
		}

		return 1;
	}

	// cmd code (u16), length (32bytes max)
	if (!strcmp(argv[2], "block_read")) {
		if (argv[3] && argv[4]) {
			uint8_t data[32] = {0};
			uint8_t cmd_code = atoi(argv[3]);
			unsigned length = atoi(argv[4]);

			if (length > 32) {
				PX4_WARN("Data length out of range: Max 32 bytes");
				return 1;
			}

			if (PX4_OK != bus.dev->block_read(cmd_code, data, length)) {
				PX4_INFO("Block read failed");
				return 1;

			} else {
				for (unsigned i = 0; i < length; i++) {
					PX4_INFO("%d", data[i]);
				}

				return 0;
			}
		}

		return 1;
	}

	// address, length, data1, data2, data3, etc
	if (!strcmp(argv[2], "block_write")) {
		if (argv[3] && argv[4]) {
			uint8_t cmd_code = atoi(argv[3]);
			unsigned length = atoi(argv[4]);
			uint8_t tx_buf[32] = {0};

			if (length > 32) {
				PX4_WARN("Data length out of range: Max 32 bytes");
				return 1;
			}

			// Data needs to be fed in 1 byte (0x01) at a time.
			for (unsigned i = 0; i < length; i++) {
				tx_buf[i] = atoi(argv[5 + i]);
				PX4_INFO("Read in: %d", tx_buf[i]);
			}

			if (PX4_OK != bus.dev->block_write(cmd_code, tx_buf, length)) {
				PX4_INFO("Dataflash read failed");
				return 1;

			} else {
				return 0;
			}
		}

		return 1;
	}

	if (!strcmp(argv[2], "read_flash")) {
		if (argv[3]) {
			uint16_t address = atoi(argv[3]);
			uint8_t rx_buf[32] = {0};

			if (PX4_OK != bus.dev->dataflash_read(address, rx_buf)) {
				PX4_INFO("Dataflash read failed");
				return 1;

			} else {
				return 0;
			}
		}

		return 1;
	}

	if (!strcmp(argv[2], "write_flash")) {
		if (argv[3] && argv[4]) {
			uint16_t address = atoi(argv[3]);
			unsigned length = atoi(argv[4]);
			uint8_t tx_buf[32] = {0};

			if (length > 32) {
				PX4_WARN("Data length out of range: Max 32 bytes");
				return 1;
			}

			// Data needs to be fed in 1 byte (0x01) at a time.
			for (unsigned i = 0; i < length; i++) {
				tx_buf[i] = atoi(argv[5 + i]);
			}

			if (PX4_OK != bus.dev->dataflash_write(address, tx_buf, length)) {
				PX4_INFO("Dataflash write failed: %d", address);
				usleep(100000);
				return 1;

			} else {
				usleep(100000);
				return 0;
			}
		}

		return 1;
	}

	batt_smbus::usage();
	return 0;
}