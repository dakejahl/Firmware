/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file pga460.cpp
 * @author Jacob Dahl <jacob.dahl@tealdrones.com>
 *
 * Driver for the TI PGA460 Ultrasonic Signal Processor and Transducer Driver
 */

#include <cstring>
#include <termios.h>
#include <math.h>

#include "pga460.h"


extern "C" __EXPORT int pga460_main(int argc, char *argv[]);

PGA460 *pga460_task;

PGA460::PGA460(const char *port) :
	CDev("PGA460", RANGE_FINDER0_DEVICE_PATH),
	_task_handle(-1),
	_task_is_running(0),
	_task_should_exit(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_fd(-1),
	_min_distance(MIN_DETECTABLE_DISTANCE),
	_max_distance(MAX_DETECTABLE_DISTANCE),
	_distance_sensor_topic(nullptr)
{
	// store port name
	strncpy(_port, port, sizeof(_port));
	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	// disable debug() calls
	_debug_enabled = false;
}

PGA460::~PGA460()
{
	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	if (_task_handle != -1) {
		_task_should_exit = true;
	}

	orb_unadvertise(_distance_sensor_topic);
}

int PGA460::stop()
{
	_task_should_exit = true;

	while (_task_is_running) {
		usleep(10000); // sleep for a tenth of a measurement cycle
	}

	px4_task_delete(_task_handle);

	return OK;
}

int PGA460::start()
{
	_task_should_exit = false;

	_paramHandle.resonant_frequency = param_find("PGA460_RES_FREQ");
	param_get(_paramHandle.resonant_frequency, &_params.resonant_frequency);

	if (init() != OK) {
		PX4_INFO("PGA460: Driver not started!");
		return PX4_ERROR;
	}

	_task_handle = px4_task_spawn_cmd("pga460",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  1100,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);


	return OK;
}

int PGA460::init()
{
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	if (init_pga460() != OK) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	struct distance_sensor_s ds_report = {};

	ds_report.timestamp = hrt_absolute_time();
	ds_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	ds_report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	ds_report.id = 0;
	ds_report.current_distance = -1.0f;	// make evident that this range sample is invalid
	ds_report.covariance = 0;

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_distance_sensor_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
		return PX4_ERROR;
	}

	return OK;
}

int PGA460::init_pga460()
{
	open_serial();

	if (!init_thresholds()) {
		PX4_WARN("Thresholds not initialized");
		return PX4_ERROR;
	}

	usleep(10000);

	// check to see if eeprom saved data matches desired data
	if (!check_eeprom()) {
		write_eeprom();
	}

	usleep(10000);

	/* Check if the device is even alive*/
	if (read_register(0x00) != USER_DATA1) {
		close_serial();
		return PX4_ERROR;
	}

	close_serial();


	return OK;
}

bool PGA460::init_thresholds()
{
	const uint8_t array_size = 35;
	uint8_t settings_buf[array_size] = {SYNCBYTE, BC_THRBW,
					    P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5,
					    P1_THR_6, P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10,
					    P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
					    P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5,
					    P2_THR_6, P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10,
					    P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15, 0xFF
					   };

	uint8_t checksum = calc_checksum(&settings_buf[1], sizeof(settings_buf) - 2);
	settings_buf[array_size - 1] = checksum;

	px4_write(_fd, &settings_buf[0], sizeof(settings_buf));

	// must wait >50us per datasheet
	usleep(100);

	if (read_threshold_registers()) {
		return 1;

	} else {
		print_device_status();
		return 0;
	}
}

void PGA460::task_main_trampoline(int argc, char *argv[])
{
	pga460_task->task_main();
}

void PGA460::task_main()
{
	while (1) {
		open_serial();

		while (!_task_should_exit) {
			_task_is_running = true;

			take_measurement();
			// Wait long enough for a pulse to travel 10meters (~30ms)
			usleep(30000);

			request_results();

			collect_results();
		}

		close_serial();

		_task_is_running = false;

		while (_task_should_exit) {
			usleep(100000);
		}
	}
}

void PGA460::take_measurement()
{
	// Issue a measurement command to detect one object using Preset 1 Burst/Listen
	uint8_t buf_tx[4] = {SYNCBYTE, P1BL, 0x01, 0xFF};
	uint8_t checksum = calc_checksum(&buf_tx[1], 2);
	buf_tx[3] = checksum;

	px4_write(_fd, &buf_tx[0], sizeof(buf_tx));
}

void PGA460::request_results()
{
	uint8_t buf_tx[2] = {SYNCBYTE, UMR};
	px4_write(_fd, &buf_tx[0], sizeof(buf_tx));
}

uint32_t PGA460::collect_results()
{
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int bytesread = 0;
	int timeout = 10;
	uint8_t buf_rx[6] = {0};

	int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);

	while (ret) {
		if (fds[0].revents & POLLIN) {
			bytesread += px4_read(_fd, buf_rx + bytesread, sizeof(buf_rx) - bytesread);

		} else { break; }

		ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);
	}

	uint16_t time_of_flight = (buf_rx[1] << 8) + buf_rx[2];
	uint8_t Width = buf_rx[3];
	uint8_t Amplitude = buf_rx[4];

	float object_distance = calculate_object_distance(time_of_flight);


	uORB_publish_results(object_distance);

	// B1,2: time_of_flight  B3: Width  B4: Amplitude
	uint32_t results = (time_of_flight << 16) | (Width << 8) | (Amplitude << 0);

	return results;
}

float PGA460::calculate_object_distance(uint16_t time_of_flight)
{
	float temperature = get_temperature();

	// if we have bad temp reading just use room temp
	if ((temperature > MAX_DETECTABLE_TEMPERATURE) || (temperature < MIN_DETECTABLE_TEMPERATURE)) {
		temperature = 20.0f;
	}

	// Formula for the speed of sound over temperature
	float speed_of_sound = 331.0f + 0.6f * temperature;

	// Calculate the distance in meters
	float millseconds_to_meters = 0.000001f;
	float object_distance = (float)time_of_flight * millseconds_to_meters * (speed_of_sound / 2.0f);

	return object_distance;
}

void PGA460::uORB_publish_results(const float &object_distance)
{
	struct distance_sensor_s report = {};
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	report.current_distance = object_distance;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.id = 0;
	report.covariance = 0;

	static bool data_is_valid = false;

	/* If we are within our MIN and MAX thresholds, continue */
	if (object_distance > get_minimum_distance() && object_distance < get_maximum_distance()) {

		/* Must have gotten an in-range sample within atleast 300ms of a previous in-range sample */
		data_is_valid = (report.timestamp - _previous_report.timestamp < 3e5);

		/* Height cannot change by more than 0.6m between measurements (6m/s / 10hz) */
		data_is_valid &= (report.current_distance < _previous_report.current_distance + 0.6f)
				 && (report.current_distance > _previous_report.current_distance - 0.6f);

		_previous_report = report;

	} else {
		data_is_valid = false;
	}

	if (data_is_valid) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}
}

float PGA460::get_temperature()
{
	uint8_t buf_tx[4] = {SYNCBYTE, TNLM, 0x00, 0xFF};
	uint8_t checksum = calc_checksum(&buf_tx[0], 3);
	buf_tx[3] = checksum;

	px4_write(_fd, &buf_tx[0], sizeof(buf_tx));
	// needs 2ms (datasheet)
	usleep(2000);

	buf_tx[1] = TNLR;
	px4_write(_fd, &buf_tx[0], sizeof(buf_tx) - 2);

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int bytesread = 0;
	int timeout = 10; //wait up to 10ms inbetween bytes
	uint8_t buf_rx[4] = {0};

	int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);

	while (ret) {
		if (fds[0].revents & POLLIN) {
			bytesread += px4_read(_fd, buf_rx + bytesread, sizeof(buf_rx) - bytesread);

		} else { break; }

		ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);
	}

	// These constants are from the pga460 data sheet on page 50
	float juntion_to_ambient_thermal_resistance = 96.1;
	float v_power = 16.5;
	float supply_current_listening = 0.012;
	// This equation is derived from the pga460 data sheet on page 50
	float temperature = ((buf_rx[1] - 64) / 1.5f) - (juntion_to_ambient_thermal_resistance * supply_current_listening *
			    v_power);

	return temperature;

}

int PGA460::open_serial()
{
	_fd = px4_open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_WARN("failed to open serial device");
		return _fd;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_fd, &uart_config);

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |  INLCR | IGNCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF);

	uart_config.c_iflag |= IGNPAR;

	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

	//uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);

	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	uart_config.c_cc[VMIN] = 1;

	uart_config.c_cc[VTIME] = 0;

	unsigned speed = 115200;

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_WARN("PGA460: ERR CFG: %d ISPD", termios_state);
		return 0;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_WARN("PGA460: ERR CFG: %d OSPD\n", termios_state);
		return 0;
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("PGA460: ERR baud %d ATTR", termios_state);
		return 0;
	}

	return _fd;
}

int PGA460::close_serial()
{
	int ret = px4_close(_fd);

	if (ret != 0) {
		PX4_WARN("PGA460: Could not close serial port");
	}

	return ret;
}

bool PGA460::write_eeprom()
{
	uint8_t settings_buf[46] = {SYNCBYTE, EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4,
				    USER_DATA5, USER_DATA6, USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10,
				    USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, USER_DATA15, USER_DATA16,
				    USER_DATA17, USER_DATA18, USER_DATA19, USER_DATA20,
				    TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, INIT_GAIN, FREQUENCY, DEADTIME,
				    PULSE_P1, PULSE_P2, CURR_LIM_P1, CURR_LIM_P2, REC_LENGTH, FREQ_DIAG, SAT_FDIAG_TH, FVOLT_DEC, DECPL_TEMP,
				    DSP_SCALE, TEMP_TRIM, P1_GAIN_CTRL, P2_GAIN_CTRL, 0xFF
				   };

	uint8_t checksum = calc_checksum(&settings_buf[1], sizeof(settings_buf) - 2);
	settings_buf[45] = checksum;

	px4_write(_fd, &settings_buf[0], sizeof(settings_buf));

	// Needs time, see datasheet timing requirements
	usleep(5000);
	unlock_eeprom();
	flash_eeprom();
	usleep(5000);

	uint8_t result = 0;

	// give up to 100ms for ee_cntrl register to reflect a successful eeprom write
	for (int i = 0; i < 100; i++) {
		result = read_register(EE_CNTRL_ADDR);
		usleep(1000);

		if (result & 1 << 2) {
			PX4_WARN("PGA460: EEPROM written to successfully");
			return 1;
		}
	}

	PX4_WARN("PGA460: Failed to write to EEPROM");
	print_diagnostics(result);
	return 0;
}

bool PGA460::check_eeprom()
{
	unlock_eeprom();

	const int array_size = 43;
	const uint8_t user_settings[array_size] = {USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4,
						   USER_DATA5, USER_DATA6, USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10,
						   USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, USER_DATA15, USER_DATA16,
						   USER_DATA17, USER_DATA18, USER_DATA19, USER_DATA20,
						   TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, INIT_GAIN, FREQUENCY, DEADTIME,
						   PULSE_P1, PULSE_P2, CURR_LIM_P1, CURR_LIM_P2, REC_LENGTH, FREQ_DIAG, SAT_FDIAG_TH, FVOLT_DEC, DECPL_TEMP,
						   DSP_SCALE, TEMP_TRIM, P1_GAIN_CTRL, P2_GAIN_CTRL
						  };

	uint8_t cmd_buf[2] = {SYNCBYTE, EEBR};

	px4_write(_fd, &cmd_buf[0], sizeof(cmd_buf));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int bytesread = 0;
	int timeout = 100;
	uint8_t buf_rx[array_size + 2] = {0};

	int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);

	while (ret) {
		if (fds[0].revents & POLLIN) {
			bytesread += px4_read(_fd, buf_rx + bytesread, sizeof(buf_rx) - bytesread);

		} else { break; }

		ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);
	}

	// Check the buffers to ensure they match
	int mismatch = memcmp(buf_rx + 1, user_settings, sizeof(buf_rx) - 2);

	if (mismatch == 0) {
		PX4_INFO("PGA460: EEPROM already has program settings");
		return 1;

	} else {
		PX4_WARN("PGA460: EEPROM does not have program settings. Bytes mismatch: %d", mismatch);
		print_diagnostics(buf_rx[0]);
		return 0;
	}
}

void PGA460::unlock_eeprom()
{
	// two step EEPROM unlock -- send unlock code w/ prog bit set to 0
	// this might actually be wrapped into command 11 (ee bulk write) but I am not sure
	uint8_t eeprom_write_buf[5] = {SYNCBYTE, SRW, EE_CNTRL_ADDR, EE_UNLOCK_ST1, 0xFF};
	uint8_t checksum = calc_checksum(&eeprom_write_buf[1], sizeof(eeprom_write_buf) - 2);
	eeprom_write_buf[4] = checksum;
	px4_write(_fd, &eeprom_write_buf[0], sizeof(eeprom_write_buf));
}

void PGA460::flash_eeprom()
{
	// send same unlock code with prog bit set to 1
	uint8_t eeprom_write_buf[5] = {SYNCBYTE, SRW, EE_CNTRL_ADDR, EE_UNLOCK_ST2, 0xFF};
	uint8_t checksum = calc_checksum(&eeprom_write_buf[1], sizeof(eeprom_write_buf) - 2);
	eeprom_write_buf[4] = checksum;
	px4_write(_fd, &eeprom_write_buf[0], sizeof(eeprom_write_buf));
}

bool PGA460::write_register(const uint8_t reg, const uint8_t val)
{
	// must unlock the eeprom registers before you can read or write to them
	if (reg < 0x40) {
		unlock_eeprom();
	}

	uint8_t buf_tx[5] = {SYNCBYTE, SRW, reg, val, 0xFF};
	uint8_t checksum = calc_checksum(&buf_tx[1], sizeof(buf_tx) - 2);
	buf_tx[4] = checksum;

	uint8_t ret = px4_write(_fd, &buf_tx[0], sizeof(buf_tx));

	if (ret != sizeof(buf_tx)) {
		return 0;
	}

	return 1;
}

uint8_t PGA460::read_register(const uint8_t reg)
{
	// must unlock the eeprom registers before you can read or write to them
	if (reg < 0x40) {
		unlock_eeprom();
	}

	uint8_t buf_tx[4] = {SYNCBYTE, SRR, reg, 0xFF};
	uint8_t checksum = calc_checksum(&buf_tx[1], 2);
	buf_tx[3] = checksum;

	px4_write(_fd, &buf_tx[0], sizeof(buf_tx));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int timeout = 100;
	int bytesread = 0;
	uint8_t buf_rx[3] = {0};

	int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

	while (ret) {
		if (fds[0].revents & POLLIN) {
			bytesread += px4_read(_fd, buf_rx + bytesread, sizeof(buf_rx) - bytesread);

		} else { break; }

		ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);
	}

	// prints errors if there are any
	print_diagnostics(buf_rx[0]);

	return buf_rx[1];
}

bool PGA460::read_threshold_registers()
{
	const int array_size = 32;
	uint8_t user_settings[array_size] = {P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4,
					     P1_THR_5, P1_THR_6, P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11,
					     P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
					     P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6,
					     P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13,
					     P2_THR_14, P2_THR_15
					    };

	uint8_t buf_tx[2] =  {SYNCBYTE, THRBR};

	px4_write(_fd, &buf_tx[0], sizeof(buf_tx));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int timeout = 100;
	int bytesread = 0;
	uint8_t buf_rx[array_size + 2] = {0};

	int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

	while (ret) {
		if (fds[0].revents & POLLIN) {
			bytesread += px4_read(_fd, buf_rx + bytesread, sizeof(buf_rx) - bytesread);

		} else { break; }

		ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);
	}

	// check to ensure the buffers match
	int mismatch = memcmp(buf_rx + 1, user_settings, sizeof(buf_rx) - 2);

	if (mismatch == 0) {
		PX4_INFO("Threshold registers have program settings");
		return 1;

	} else {
		PX4_WARN("Threshold registers do not have program settings");
		print_diagnostics(buf_rx[0]);
		return 0;
	}
}

void PGA460::print_device_status()
{
	uint8_t status_flags1 = read_register(0x4C);
	uint8_t status_flags2 = read_register(0x4D);

	if ((status_flags1 & 0x0F) || status_flags2) {
		if (status_flags1 & 0x0F & 1) {
			PX4_INFO("PGA460: Trim EEPROM space data CRC error");
		}

		if (status_flags1 & 0x0F & 1 << 1) {
			PX4_INFO("PGA460: User EEPROM space data CRC error");
		}

		if (status_flags1 & 0x0F & 1 << 2) {
			PX4_INFO("PGA460: Threshold map configuration register data CRC error");
		}

		if (status_flags1 & 0x0F & 1 << 3) {
			PX4_INFO("PGA460: Wakeup Error");
		}

		if (status_flags2 & 1) {
			PX4_INFO("PGA460: VPWR pin under voltage");
		}

		if (status_flags2 & 1 << 1) {
			PX4_INFO("PGA460: VPWR pin over voltage");
		}

		if (status_flags2 & 1 << 2) {
			PX4_INFO("PGA460: AVDD pin under voltage");
		}

		if (status_flags2 & 1 << 3) {
			PX4_INFO("PGA460: AVDD pin over voltage");
		}

		if (status_flags2 & 1 << 4) {
			PX4_INFO("PGA460: IOREG pin under voltage");
		}

		if (status_flags2 & 1 << 5) {
			PX4_INFO("PGA460: IOREG pin over voltage");
		}

		if (status_flags2 & 1 << 6) {
			PX4_INFO("PGA460: Thermal shutdown has occured");
		}
	}
}

void PGA460::print_diagnostics(const uint8_t diagnostic_byte)
{
	// Stores the most recent diagnostic byte as a global
	_diagnostic_byte = diagnostic_byte;

	// check the diagnostics bit field
	if (diagnostic_byte & 1 << 6) {
		if (diagnostic_byte & 1 << 0) {
			PX4_INFO("PGA460: Device busy");
		}

		if (diagnostic_byte & 1 << 1) {
			PX4_INFO("PGA460: Sync field bit rate too high/low");
		}

		if (diagnostic_byte & 1 << 2) {
			PX4_INFO("PGA460: Consecutive sync bit fields do not match");
		}

		if (diagnostic_byte & 1 << 3) {
			PX4_INFO("PGA460: Invalid checksum");
		}

		if (diagnostic_byte & 1 << 4) {
			PX4_INFO("PGA460: Invalid command");
		}

		if (diagnostic_byte & 1 << 5) {
			PX4_INFO("PGA460: General comm erorr");
		}

	} else if (diagnostic_byte & 1 << 7) {
		if (diagnostic_byte & 1 << 0) {
			PX4_INFO("PGA460: Device busy");
		}

		if (diagnostic_byte & 1 << 1) {
			PX4_INFO("PGA460: Threshold settings CRC error");
		}

		if (diagnostic_byte & 1 << 2) {
			PX4_INFO("PGA460: Frequency diagnostics error");
		}

		if (diagnostic_byte & 1 << 3) {
			PX4_INFO("PGA460: Voltage diagnostics error");
		}

		if (diagnostic_byte & 1 << 4) {
			PX4_INFO("PGA460: Always zero....");
		}

		if (diagnostic_byte & 1 << 5) {
			PX4_INFO("PGA460: EEPROM CRC or TRIM CRC error");
		}
	}
}

uint8_t PGA460::get_diagnostic_byte()
{
	return _diagnostic_byte;
}

uint8_t PGA460::find_resonant_frequency()
{
	// sweep from 25 to 75 == 35kHz to 45kHz
	uint8_t amplitudes[50] = {0};
	uint8_t maxval = 0;
	uint8_t max_index = 0;

	for (size_t frequency_index = 25; frequency_index < 74; frequency_index++) {
		write_register(0x1C, frequency_index);
		usleep(10000);

		take_measurement();

		usleep(100000);
		request_results();

		usleep(10000);

		uint32_t results = collect_results();
		PX4_INFO("amplitude is: %d", results & 0xFF);

		usleep(10000);

		amplitudes[frequency_index - 25] = (results & 0xFF);

		if ((amplitudes[frequency_index - 25] > maxval) && (amplitudes[frequency_index - 25] != 0xFF)) {
			maxval = amplitudes[frequency_index - 25];
			max_index = frequency_index;
		}
	}

	write_register(0x1C, max_index);
	usleep(10000);

	// formula from data sheet pg70
	float frequency = max_index * 0.2f + 30;
	param_set(param_find("PGA460_RES_FREQ"), &(frequency));

	return max_index;

}

uint16_t PGA460::get_system_diagnostics()
{
	uint8_t buf_tx[2] = {SYNCBYTE, SD};

	tcflush(_fd, TCIOFLUSH);

	px4_write(_fd, &buf_tx[0], sizeof(buf_tx));

	// @TODO: consider opportunity to refactor this
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int bytesread = 0;
	int timeout = 100;
	uint8_t buf_rx[4] = {0};

	int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

	while (ret) {
		if (fds[0].revents & POLLIN) {
			bytesread += px4_read(_fd, buf_rx + bytesread, sizeof(buf_rx) - bytesread);

		} else { break; }

		ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);
	}

	uint16_t sys_diag = (buf_rx[1] << 8) + buf_rx[2];

	// if a diagnostic bit is set we will be warned
	print_diagnostics(buf_rx[0]);
	PX4_INFO("\nPGA460: The transducer frequency has been measured as %2.2f \n"
		 "The decay period time has been measured as %d", 1000000 / ((double)buf_rx[1] * 500), buf_rx[2]);

	uint8_t saved_freq = read_register(0x1C);
	PX4_INFO("PGA460: \nThe transducer frequency is currently saved as %2.2f", (double)saved_freq * 0.2 + 30);

	return sys_diag;
}

void PGA460::suspend()
{
	_task_should_exit = true;

	while (_task_is_running) {
		usleep(200000); // sleep for two measurement cycles
	}
}

void PGA460::resume()
{
	_task_should_exit = false;
}

float PGA460::get_minimum_distance()
{
	return _min_distance;
}

void PGA460::set_minimum_distance(const float dist)
{
	_min_distance = dist;
}

float PGA460::get_maximum_distance()
{
	return _max_distance;
}

void PGA460::set_maximum_distance(const float dist)
{
	_max_distance = dist;
}

uint8_t PGA460::calc_checksum(uint8_t *data, const uint8_t size)
{
	uint8_t n = 0;
	uint8_t checksum_input[size] = {0};

	for (n = 0; n < size; n++) {
		checksum_input[n] = *data;
		data++;
	}

	int checksum_loops = n;
	uint16_t carry = 0;

	for (int j = 0; j < checksum_loops; j++) {
		if ((checksum_input[j] + carry) < carry) {
			carry = carry + checksum_input[j] + 1;

		} else {
			carry = carry + checksum_input[j];
		}

		if (carry > 0xFF) {
			carry = carry - 255;
		}
	}

	carry = (~carry & 0x00FF);
	return carry;
}


// Local functions in support of the shell command

bool start()
{
	if (pga460_task != nullptr) {
		PX4_INFO("driver already started");
		return 0;
	}

	pga460_task = new PGA460(PGA460_DEFAULT_PORT);

	if (pga460_task == nullptr) {
		PX4_ERR("failed to create instance of PGA460");
		return 1;
	}

	if (OK != pga460_task->start()) {
		PX4_ERR("Failed to start PGA460: could not reach device.");
		delete pga460_task;
		pga460_task = nullptr;
		return 1;
	}

	return 0;
}

bool stop()
{
	if (pga460_task != nullptr) {
		pga460_task->stop();
		delete pga460_task;
		pga460_task = nullptr;

	} else {
		PX4_INFO("driver not running");
	}

	return 0;
}

void info()
{
	PX4_INFO("\n\tstart            	 	- Starts the PGA460 driver."
		 "\n\tstop 				- Stops the PGA460 driver."
		 "\n\tcheck_eeprom		- Checks to see if the EEPROM has firmware default values"
		 "\n\twrite_eeprom		- Writes the firmware default values to the EEPROM"
		 "\n\tread_register(reg)		- Read a register."
		 "\n\twrite_register(reg, val)	- Writes a value to a register."
		 "\n\tcalibrate			- Sweeps from 35 - 45 kHz and sets the system frequency"
		 "\n\tdiagnostics		- Compares the measured frequency to the saved value"
		 "\n\t"
		 "\n\tAll arguments are in decimal"
		 "\n\t");
}


//	MAIN

int pga460_main(int argc, char *argv[])
{
	//Start/load the driver.
	if (!strcmp(argv[1], "start")) {
		start();
		return 0;
	}

	//Stop the driver
	if (!strcmp(argv[1], "stop")) {
		stop();
		return 0;
	}

	//Prints options
	if (!strcmp(argv[1], "help") || !strcmp(argv[1], "info")) {
		info();
		return 0;
	}

	//Checks the eeprom an reports whether or not the settings match the defaults
	if (!strcmp(argv[1], "check_eeprom")) {
		pga460_task->suspend();

		pga460_task->open_serial();

		bool ret = pga460_task->check_eeprom();

		if (ret) {
			PX4_INFO("EEPROM has firmware default settings");

		} else {
			PX4_INFO("EEPROM does not have firmware default settings");
		}

		pga460_task->close_serial();

		pga460_task->resume();
		return 0;
	}

	//Flashes default settings to the EEPROM
	if (!strcmp(argv[1], "write_eeprom")) {
		pga460_task->suspend();

		pga460_task->open_serial();

		bool ret = pga460_task->write_eeprom();

		if (ret) {
			PX4_INFO("EEPROM successfully written to");

		} else {
			PX4_INFO("EEPROM was not successfully written to");
		}

		pga460_task->close_serial();

		pga460_task->resume();
		return 0;
	}

	//Read the diagnostic registers: Measured transducer frequency and transducer decay time
	if (!strcmp(argv[1], "diagnostics")) {
		pga460_task->suspend();
		pga460_task->open_serial();
		pga460_task->get_system_diagnostics();
		pga460_task->close_serial();
		pga460_task->resume();
		return 0;
	}

	// Reads the (register)
	if (!strcmp(argv[1], "read_register")) {
		if (argv[2]) {
			pga460_task->suspend();
			usleep(100000);
			pga460_task->open_serial();
			uint8_t ret = pga460_task->read_register((uint8_t)atoi(argv[2]));
			pga460_task->close_serial();
			pga460_task->resume();
			PX4_INFO("Register has value %d", ret);

		} else {
			PX4_INFO("Unrecognized arguments");
		}

		return 0;
	}

	//Writes to the (register) the (value)
	if (!strcmp(argv[1], "write_register")) {
		if (argv[2] && argv[3]) {
			pga460_task->suspend();
			pga460_task->open_serial();
			uint8_t ret = pga460_task->write_register((uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3]));
			pga460_task->close_serial();
			pga460_task->resume();

			if (ret) { PX4_INFO("Register successfully written to"); }

		} else {
			PX4_INFO("Unrecognized arguments");
		}

		return 0;
	}

	//Sweeps across a frequency range and sets the operating frequency to the resonant
	if (!strcmp(argv[1], "calibrate")) {
		pga460_task->suspend();
		pga460_task->open_serial();
		uint8_t freq = pga460_task->find_resonant_frequency();
		pga460_task->close_serial();
		pga460_task->resume();

		PX4_INFO("Resonant frequency detected as: %d", freq);
		return 0;
	}


	PX4_INFO("Unrecognized arguments, try: start [device_name], stop, info ");
	return 1;
}
