/**
 * @file pga460.cpp
 * @author Khoi Tran <khoi@tealdrones.com>
 *
 * Driver for the TI PGA460 Ultrasonic Signal Processor and Transducer Driver
 */

#include <cstring>
#include <termios.h>

#include "pga460.h"

extern "C" __EXPORT int pga460_main(int argc, char *argv[]);

namespace pga460
{
PGA460	*g_dev;
}

PGA460::PGA460(const char *port) :
	CDev("PGA460", RANGE_FINDER0_DEVICE_PATH),
	_task_should_exit(false),
	_task_handle(-1),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr)
{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	// disable debug() calls
	_debug_enabled = true;

	memset(&_buf[0], 0, sizeof(_buf));
}

PGA460::~PGA460()
{

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	if (_task_handle != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_task_handle);
				break;
			}
		} while (_task_handle != -1);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	orb_unadvertise(_distance_sensor_topic);
}

int PGA460::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) {
			PX4_WARN("cdev init failed");
			break;
		}

		_fd = px4_open(RANGE_FINDER0_DEVICE_PATH, 0);

		if (_fd < 0) {
			PX4_WARN("failed to open range finder device");
			ret = 1;
			break;
		}

		px4_close(_fd);

		/* open fd */
		_fd = px4_open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_WARN("failed to open serial device");
			ret = 1;
			break;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag	 &= ~PARENB;

		//uart_config.c_cflag	 |= CSTOPB;

		unsigned speed = B115200;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d ISPD", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d OSPD\n", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_WARN("ERR baud %d ATTR", termios_state);
			ret = 1;
			break;
		}

		init_pga460();

		px4_close(_fd);

		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

		struct distance_sensor_s ds_report = {};
		ds_report.timestamp = hrt_absolute_time();
		ds_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		ds_report.orientation = 8;
		ds_report.id = 0;
		ds_report.current_distance = -1.0f;	// make evident that this range sample is invalid
		ds_report.covariance = 0;

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
			ret = 1;
			break;
		}

	} while (0);

	return ret;
}

void PGA460::task_main_trampoline(int argc, char *argv[])
{
	pga460::g_dev->task_main();
}

int PGA460::start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("pga460",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX - 30,
					  800,
					  (px4_main_t)&PGA460::task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

void PGA460::task_main()
{

	_fd = px4_open(_port, O_RDWR | O_NOCTTY);

	while (!_task_should_exit) {
		ultrasonicCmd();
		usleep(10000);
		pullUltrasonicMeasResult();
		usleep(100000);
	}

	px4_close(_fd);
}

void PGA460::init_pga460()
{
	init_thresholds();
	usleep(10000);

	init_eeprom();
	usleep(10000);
}

void PGA460::init_thresholds()
{
	uint8_t buf[35] = {SYNCBYTE, THRBW, P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4,
			   P1_THR_5, P1_THR_6, P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11,
			   P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
			   P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6,
			   P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13,
			   P2_THR_14, P2_THR_15, calcChecksum(THRBW)
			  };
	px4_write(_fd, &buf[0], sizeof(buf));
}

void PGA460::init_eeprom()
{
	uint8_t buf[46] = {SYNCBYTE, EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4,
			   USER_DATA5, USER_DATA6, USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10,
			   USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, USER_DATA15, USER_DATA16,
			   USER_DATA17, USER_DATA18, USER_DATA19, USER_DATA20,
			   TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, INIT_GAIN, FREQUENCY, DEADTIME,
			   PULSE_P1, PULSE_P2, CURR_LIM_P1, CURR_LIM_P2, REC_LENGTH, FREQ_DIAG, SAT_FDIAG_TH, FVOLT_DEC, DECPL_TEMP,
			   DSP_SCALE, TEMP_TRIM, P1_GAIN_CTRL, P2_GAIN_CTRL, calcChecksum(EEBW)
			  };
	px4_write(_fd, buf, sizeof(buf));
}

uint8_t PGA460::calcChecksum(uint8_t cmd)
{
	int checksumLoops = 0;
	cmd = cmd & 0x001F; // zero-mask command address of cmd to select correct switchcase statement

	switch (cmd) {
	case 0 : //P1BL
	case 1 : //P2BL
	case 2 : //P1LO
	case 3 : //P2LO
	case 17 : //BC_P1_valBL
	case 18 : //BC_P2_valBL
	case 19 : //BC_P1_valLO
	case 20 : //BC_P2_valLO
		_checksum_input[0] = cmd;
		_checksum_input[1] = 1;
		checksumLoops = 2;
		break;

	case 4 : //TNLM
	case 21 : //TNLM
		_checksum_input[0] = cmd;
		_checksum_input[1] = 0;
		checksumLoops = 2;
		break;

	case 5 : //UMR
	case 6 : //TNLR
	case 7 : //TEDD
	case 8 : //SD
	case 11 : //EEBR
	case 13 : //TVGBR
	case 15 : //THRBR
		_checksum_input[0] = cmd;
		checksumLoops = 1;
		break;

	case 9 : //RR
		_checksum_input[0] = cmd;
		_checksum_input[1] = _regAddr;
		checksumLoops = 2;
		break;

	case 10 : //RW
	case 22 : //BC_RW
		_checksum_input[0] = cmd;
		_checksum_input[1] = _regAddr;
		_checksum_input[2] = _regData;
		checksumLoops = 3;
		break;

	case 14 : //TVGBW
	case 24 : //BC_TVGBW
		_checksum_input[0] = cmd;
		_checksum_input[1] = TVGAIN0;
		_checksum_input[2] = TVGAIN1;
		_checksum_input[3] = TVGAIN2;
		_checksum_input[4] = TVGAIN3;
		_checksum_input[5] = TVGAIN4;
		_checksum_input[6] = TVGAIN5;
		_checksum_input[7] = TVGAIN6;
		checksumLoops = 8;
		break;

	case 16 : //THRBW
	case 25 : //BC_THRBW
		_checksum_input[0] = cmd;
		_checksum_input[1] = P1_THR_0;
		_checksum_input[2] = P1_THR_1;
		_checksum_input[3] = P1_THR_2;
		_checksum_input[4] = P1_THR_3;
		_checksum_input[5] = P1_THR_4;
		_checksum_input[6] = P1_THR_5;
		_checksum_input[7] = P1_THR_6;
		_checksum_input[8] = P1_THR_7;
		_checksum_input[9] = P1_THR_8;
		_checksum_input[10] = P1_THR_9;
		_checksum_input[11] = P1_THR_10;
		_checksum_input[12] = P1_THR_11;
		_checksum_input[13] = P1_THR_12;
		_checksum_input[14] = P1_THR_13;
		_checksum_input[15] = P1_THR_14;
		_checksum_input[16] = P1_THR_15;
		_checksum_input[17] = P2_THR_0;
		_checksum_input[18] = P2_THR_1;
		_checksum_input[19] = P2_THR_2;
		_checksum_input[20] = P2_THR_3;
		_checksum_input[21] = P2_THR_4;
		_checksum_input[22] = P2_THR_5;
		_checksum_input[23] = P2_THR_6;
		_checksum_input[24] = P2_THR_7;
		_checksum_input[25] = P2_THR_8;
		_checksum_input[26] = P2_THR_9;
		_checksum_input[27] = P2_THR_10;
		_checksum_input[28] = P2_THR_11;
		_checksum_input[29] = P2_THR_12;
		_checksum_input[30] = P2_THR_13;
		_checksum_input[31] = P2_THR_14;
		_checksum_input[32] = P2_THR_15;
		checksumLoops = 33;
		break;

	case 12 : //EEBW
	case 23 : //BC_EEBW
		_checksum_input[0] = cmd;
		_checksum_input[1] = USER_DATA1;
		_checksum_input[2] = USER_DATA2;
		_checksum_input[3] = USER_DATA3;
		_checksum_input[4] = USER_DATA4;
		_checksum_input[5] = USER_DATA5;
		_checksum_input[6] = USER_DATA6;
		_checksum_input[7] = USER_DATA7;
		_checksum_input[8] = USER_DATA8;
		_checksum_input[9] = USER_DATA9;
		_checksum_input[10] = USER_DATA10;
		_checksum_input[11] = USER_DATA11;
		_checksum_input[12] = USER_DATA12;
		_checksum_input[13] = USER_DATA13;
		_checksum_input[14] = USER_DATA14;
		_checksum_input[15] = USER_DATA15;
		_checksum_input[16] = USER_DATA16;
		_checksum_input[17] = USER_DATA17;
		_checksum_input[18] = USER_DATA18;
		_checksum_input[19] = USER_DATA19;
		_checksum_input[20] = USER_DATA20;
		_checksum_input[21] = TVGAIN0;
		_checksum_input[22] = TVGAIN1;
		_checksum_input[23] = TVGAIN2;
		_checksum_input[24] = TVGAIN3;
		_checksum_input[25] = TVGAIN4;
		_checksum_input[26] = TVGAIN5;
		_checksum_input[27] = TVGAIN6;
		_checksum_input[28] = INIT_GAIN;
		_checksum_input[29] = FREQUENCY;
		_checksum_input[30] = DEADTIME;
		_checksum_input[31] = PULSE_P1;
		_checksum_input[32] = PULSE_P2;
		_checksum_input[33] = CURR_LIM_P1;
		_checksum_input[34] = CURR_LIM_P2;
		_checksum_input[35] = REC_LENGTH;
		_checksum_input[36] = FREQ_DIAG;
		_checksum_input[37] = SAT_FDIAG_TH;
		_checksum_input[38] = FVOLT_DEC;
		_checksum_input[39] = DECPL_TEMP;
		_checksum_input[40] = DSP_SCALE;
		_checksum_input[41] = TEMP_TRIM;
		_checksum_input[42] = P1_GAIN_CTRL;
		_checksum_input[43] = P2_GAIN_CTRL;
		checksumLoops = 44;
		break;

	default: break;
	}

	uint16_t carry = 0;

	for (int i = 0; i < checksumLoops; i++) {
		if ((_checksum_input[i] + carry) < carry) {
			carry = carry + _checksum_input[i] + 1;

		} else {
			carry = carry + _checksum_input[i];
		}

		if (carry > 0xFF) {
			carry = carry - 255;
		}
	}

	carry = (~carry & 0x00FF);
	return carry;
}

void PGA460::ultrasonicCmd()
{
	uint8_t bufCmd[4] = {SYNCBYTE, 0xFF, 0x1, 0xFF}; // prepare bufCmd with 0xFF placeholders

	bufCmd[1] = P1BL;
	bufCmd[3] = calcChecksum(P1BL);

	px4_write(_fd, bufCmd, sizeof(bufCmd)); // serial transmit master data to initiate burst and/or listen command

	return;
}

bool PGA460::pullUltrasonicMeasResult()
{
	uint8_t buf[3] = {SYNCBYTE, UMR, calcChecksum(UMR)};
	px4_write(_fd, buf, sizeof(buf)); //serial transmit master data to read ultrasonic measurement results
	// we poll on data from the serial port
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	// wait for up to 100ms for data
	int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

	// timed out
	if (pret == 0) {
		return false;
	}

	uint8_t buf_rx[10];

	if (fds[0].revents & POLLIN) {
		memset(&buf_rx[0], 0, sizeof(buf_rx));
		int len = px4_read(_fd, &buf_rx[0], sizeof(buf_rx));

		if (len <= 0) {
			PX4_DEBUG("error reading PGA460");
		}

		uint16_t objDist = (buf_rx[1] << 8) + buf_rx[2];
		float range = (float)objDist / 2.0f * 0.000001f * SPEED_OF_SOUND;

		struct distance_sensor_s report = {};
		report.timestamp = hrt_absolute_time();
		report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		report.orientation = 8;
		report.current_distance = range;
		report.min_distance = 0;
		report.max_distance = 6.0f;
		report.covariance = 0;
		report.id = 0;

		// publish it
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	return true;
}

int pga460_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (pga460::g_dev != nullptr) {
			PX4_WARN("driver already started");
			return 0;
		}

		if (argc > 2) {
			pga460::g_dev = new PGA460(argv[2]);

		} else {
			pga460::g_dev = new PGA460(PGA460_DEFAULT_PORT);
		}

		if (pga460::g_dev == nullptr) {
			PX4_ERR("failed to create instance of PGA460");
			return 1;
		}

		if (PX4_OK != pga460::g_dev->init()) {
			delete pga460::g_dev;
			pga460::g_dev = nullptr;
			return 1;
		}

		if (OK != pga460::g_dev->start()) {
			delete pga460::g_dev;
			pga460::g_dev = nullptr;
			return 1;
		}

		return 0;
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		if (pga460::g_dev != nullptr) {
			delete pga460::g_dev;
			pga460::g_dev = nullptr;

		} else {
			PX4_WARN("driver not running");
		}

		return 0;
	}

	if (!strcmp(argv[1], "info")) {
		PX4_INFO("TI PGA460");
		PX4_INFO("update rate: 10 Hz");
		return 0;

	}

	PX4_WARN("unrecognized arguments, try: start [device_name], stop, info ");
	return 1;
}
