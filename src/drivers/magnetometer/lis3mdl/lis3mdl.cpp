/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file lis3mdl.cpp
 *
 * Driver for the LIS3MDL magnetometer connected via I2C or SPI.
 *
 * Based on the hmc5883 driver.
 */

#include "lis3mdl.h"

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int lis3mdl_main(int argc, char *argv[]);


LIS3MDL::LIS3MDL(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("LIS3MDL", path),
	_interface(interface),
	_work{},
	_reports(nullptr),
	_scale{},
	_last_report{},
	_mag_topic(nullptr),
	_comms_errors(perf_alloc(PC_COUNT, "lis3mdl_comms_errors")),
	_conf_errors(perf_alloc(PC_COUNT, "lis3mdl_conf_errors")),
	_range_errors(perf_alloc(PC_COUNT, "lis3mdl_range_errors")),
	_sample_perf(perf_alloc(PC_ELAPSED, "lis3mdl_read")),
	_calibrated(false),
	_continuous_mode_set(false),
	_mode(CONTINUOUS),
	_rotation(rotation),
	_measure_ticks(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_range_ga(4.0f),
	_range_scale(0), /* default range scale from counts to gauss */
	_check_state_cnt(0),
	_cntl_reg1(0xFC), // 1 11 111 0 0 | temp-en, ultra high performance (XY), fast_odr disabled, self test disabled
	_cntl_reg2(0x00), // 4 gauss FS range, reboot settings default
	_cntl_reg3(0x00), // operating mode CONTINUOUS!
	_cntl_reg4(0x0C), // Z-axis ultra high performance mode
	_cntl_reg5(0x00), // fast read disabled, continious update disabled (block data update)
	_range_bits(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LIS3MDL;

	// enable debug() calls
	_debug_enabled = false;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

LIS3MDL::~LIS3MDL()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
LIS3MDL::init()
{
	int ret = PX4_ERROR;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	return OK;
}

int
LIS3MDL::set_range(unsigned range)
{
	if (range <= 4) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 6842.0f;
		_range_ga = 4.0f;

	} else if (range <= 8) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 3421.0f;
		_range_ga = 8.0f;

	} else if (range <= 12) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 2281.0f;
		_range_ga = 12.0f;

	} else {
		_range_bits = 0x03;
		_range_scale = 1.0f / 1711.0f;
		_range_ga = 16.0f;
	}

	int ret = 0;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CTRL_REG2, (_range_bits << 5));

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CTRL_REG2, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	if (range_bits_in == (_range_bits << 5)) {
		return OK;

	} else {
		return PX4_ERROR;
	}
}

ssize_t
LIS3MDL::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(LIS3MDL_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	return ret;
}

int
LIS3MDL::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	unsigned dummy = 0;

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* zero would be bad */
			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool not_started = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(LIS3MDL_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (not_started) {
						start();
					}

					return OK;
				}

			/* Uses arg (hz) for a custom poll rate */
			default: {
					/* do we need to start internal polling? */
					bool not_started = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (not_started) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case MAGIOCGSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return 1000000 / TICK2USEC(_measure_ticks);

	case MAGIOCSRANGE:
		return set_range(arg);

	case MAGIOCGRANGE:
		return _range_ga;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		/* check calibration, but not actually return an error */
		(void)check_calibration();
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;


	case MAGIOCCALIBRATE:
		return calibrate(filp, arg);

	case MAGIOCEXSTRAP:
		return set_excitement(arg);

	case MAGIOCSELFTEST:
		return check_calibration();


	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return _interface->ioctl(cmd, dummy);

	case DEVIOCGDEVICEID:
		return _interface->ioctl(cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
LIS3MDL::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	set_register_default_values();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&LIS3MDL::cycle_trampoline, this, 1);
}

void
LIS3MDL::stop()
{
	if (_measure_ticks > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_ticks = 0;
		work_cancel(HPWORK, &_work);
	}
}

int
LIS3MDL::reset()
{
	int ret = 0;

	ret = set_register_default_values();

	if (ret != OK) {
		return PX4_ERROR;
	}

	ret = set_range(_range_ga);

	if (ret != OK) {
		return PX4_ERROR;
	}

	return OK;
}

void
LIS3MDL::cycle_trampoline(void *arg)
{
	LIS3MDL *dev = (LIS3MDL *)arg;

	dev->cycle();
}

void
LIS3MDL::cycle()
{
	/* _measure_ticks == 0  is used as _task_should_exit */
	if (_measure_ticks == 0) {
		return;
	}

	/* Collect last measurement at the start of every cycle */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* restart the measurement state machine */
		start();
		return;
	}


	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	if (_measure_ticks > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&LIS3MDL::cycle_trampoline,
			   this,
			   USEC2TICK(LIS3MDL_CONVERSION_INTERVAL));
	}
}

int
LIS3MDL::measure()
{
	int ret = 0;

	/* Send the command to begin a measurement. */
	if ((_mode == CONTINUOUS) && !_continuous_mode_set) {
		ret = write_reg(ADDR_CTRL_REG3, MODE_REG_CONTINOUS_MODE);
		_continuous_mode_set = true;

	} else if (_mode == SINGLE) {
		ret = write_reg(ADDR_CTRL_REG3, MODE_REG_SINGLE_MODE);
		_continuous_mode_set = false;
	}


	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
LIS3MDL::collect()
{
#pragma pack(push, 1)
	struct { /* X, Y, and Z data. */
		uint8_t         x[2];
		uint8_t         y[2];
		uint8_t         z[2];
	}       lis_report;

	struct {
		int16_t         x;
		int16_t         y;
		int16_t         z;
		int16_t         t;
	} report;
#pragma pack(pop)

	int     ret = 0;
	uint8_t buf_rx[2] = {0};

	perf_begin(_sample_perf);
	struct mag_report new_mag_report;
	bool sensor_is_onboard = false;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	new_mag_report.timestamp = hrt_absolute_time();
	new_mag_report.error_count = perf_event_count(_comms_errors);
	new_mag_report.range_ga = _range_ga;
	new_mag_report.scaling = _range_scale;
	new_mag_report.device_id = _device_id.devid;

	ret = _interface->read(ADDR_OUT_X_L, (uint8_t *)&lis_report, sizeof(lis_report));

	/* Weird behavior: the X axis will be read instead of the temperature registers if you use a pointer to a packed struct...not sure why.
	 * This works now, but further investigation to determine why this happens would be good (I am guessing a type error somewhere)
	 */
	ret = _interface->read(ADDR_OUT_T_L, (uint8_t *)&buf_rx, sizeof(buf_rx));

	if (ret != OK) {
		perf_count(_comms_errors);
		PX4_WARN("Register read error.");
		return ret;
	}

	report.x = (int16_t)((lis_report.x[1] << 8) | lis_report.x[0]);
	report.y = (int16_t)((lis_report.y[1] << 8) | lis_report.y[0]);
	report.z = (int16_t)((lis_report.z[1] << 8) | lis_report.z[0]);

	report.t = (int16_t)((buf_rx[1] << 8) | buf_rx[0]);

	float temperature = report.t;
	new_mag_report.temperature = 25.0f + (temperature / 8.0f);

	// XXX revisit for SPI part, might require a bus type IOCTL

	unsigned dummy = 0;
	sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);
	new_mag_report.is_external = !sensor_is_onboard;

	/*
	 * RAW outputs
	 *
	 */
	new_mag_report.x_raw = report.x;
	new_mag_report.y_raw = report.y;
	new_mag_report.z_raw = report.z;

	xraw_f = report.x;
	yraw_f = report.y;
	zraw_f = report.z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	new_mag_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	/* flip axes and negate value for y */
	new_mag_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	/* z remains z */
	new_mag_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	if (!(_pub_blocked)) {

		if (_mag_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_mag_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_mag_report,
							 &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

			if (_mag_topic == nullptr) {
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	}

	_last_report = new_mag_report;

	/* post a report to the ring */
	_reports->force(&new_mag_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

int
LIS3MDL::calibrate(struct file *filp, unsigned enable)
{
	struct mag_report report;
	ssize_t sz;
	int ret = 1;
	uint8_t num_samples = 10;

	// XXX do something smarter here
	int fd = (int)enable;

	float sum_excited[3] = {0.0f, 0.0f, 0.0f};
	float sum_non_excited[3] = {0.0f, 0.0f, 0.0f};

	/* start the sensor polling at 50 Hz */
	if (OK != ioctl(filp, SENSORIOCSPOLLRATE, 50)) {
		warn("FAILED: SENSORIOCSPOLLRATE 50Hz");
		ret = 1;
		goto out;
	}

	/* Set to 12 Gauss */
	if (OK != ioctl(filp, MAGIOCSRANGE, 12)) {
		PX4_WARN("FAILED: MAGIOCSRANGE 12 Ga");
		ret = 1;
		goto out;
	}

	usleep(20000);

	/* discard 10 samples to let the sensor settle */
	for (uint8_t i = 0; i < num_samples; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 1");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 10x */
	for (uint8_t i = 0; i < num_samples; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		sum_non_excited[0] += report.x;
		sum_non_excited[1] += report.y;
		sum_non_excited[2] += report.z;
	}

	sum_non_excited[0] /= num_samples;
	sum_non_excited[1] /= num_samples;
	sum_non_excited[2] /= num_samples;

	/* excite strap and take measurements */
	if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
		PX4_WARN("FAILED: MAGIOCEXSTRAP 1");
		ret = 1;
		goto out;
	}

	usleep(60000);

	/* discard 10 samples to let the sensor settle */
	for (uint8_t i = 0; i < num_samples; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 1");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 10x */
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		sum_excited[0] += report.x;
		sum_excited[1] += report.y;
		sum_excited[2] += report.z;
	}

	sum_excited[0] /= num_samples;
	sum_excited[1] /= num_samples;
	sum_excited[2] /= num_samples;

	if (1.0f < fabsf(sum_excited[0] - sum_non_excited[0]) && fabsf(sum_excited[0] - sum_non_excited[0]) < 3.0f &&
	    1.0f < fabsf(sum_excited[1] - sum_non_excited[1]) && fabsf(sum_excited[1] - sum_non_excited[1]) < 3.0f &&
	    0.1f < fabsf(sum_excited[2] - sum_non_excited[2]) && fabsf(sum_excited[2] - sum_non_excited[2]) < 1.0f) {
		ret = OK;

	} else {
		ret = -EIO;
		goto out;
	}

out:

	/* set back to normal mode */
	if (OK != ::ioctl(fd, MAGIOCSRANGE, 4)) {
		PX4_WARN("FAILED: MAGIOCSRANGE 4 Ga");
	}

	if (OK != ::ioctl(fd, MAGIOCEXSTRAP, 0)) {
		PX4_WARN("FAILED: MAGIOCEXSTRAP 0");
	}

	usleep(20000);

	return ret;
}

// int
// LIS3MLD::set_default_register_values()
// {

// }


int
LIS3MDL::check_scale()
{
	bool scale_valid;

	if ((-FLT_EPSILON + 1.0f < _scale.x_scale && _scale.x_scale < FLT_EPSILON + 1.0f) &&
	    (-FLT_EPSILON + 1.0f < _scale.y_scale && _scale.y_scale < FLT_EPSILON + 1.0f) &&
	    (-FLT_EPSILON + 1.0f < _scale.z_scale && _scale.z_scale < FLT_EPSILON + 1.0f)) {
		/* scale is one */
		scale_valid = false;

	} else {
		scale_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !scale_valid;
}

int
LIS3MDL::check_offset()
{
	bool offset_valid;

	if ((-2.0f * FLT_EPSILON < _scale.x_offset && _scale.x_offset < 2.0f * FLT_EPSILON) &&
	    (-2.0f * FLT_EPSILON < _scale.y_offset && _scale.y_offset < 2.0f * FLT_EPSILON) &&
	    (-2.0f * FLT_EPSILON < _scale.z_offset && _scale.z_offset < 2.0f * FLT_EPSILON)) {
		/* offset is zero */
		offset_valid = false;

	} else {
		offset_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !offset_valid;
}

int
LIS3MDL::check_calibration()
{
	bool offset_valid = (check_offset() == OK);
	bool scale_valid  = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {
		PX4_WARN("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
			 (offset_valid) ? "" : "offset invalid");
		_calibrated = (offset_valid && scale_valid);
	}

	/* return 0 if calibrated, 1 else */
	return !_calibrated;
}

int
LIS3MDL::set_excitement(unsigned enable)
{
	int ret;
	/* arm the excitement strap */
	ret = read_reg(ADDR_CTRL_REG1, _cntl_reg1);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	_cntl_reg1 &= ~0x01; // reset previous excitement mode

	if (((int)enable) < 0) {
		PX4_WARN("WARN: set_excitement negative not supported\n");

	} else if (enable > 0) {
		_cntl_reg1 |= 0x01;
	}

	::printf("set_excitement enable=%d cntl1=0x%x\n", (int)enable, (unsigned)_cntl_reg1);

	ret = write_reg(ADDR_CTRL_REG1, _cntl_reg1);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t conf_reg_ret = 0;
	read_reg(ADDR_CTRL_REG1, conf_reg_ret);

	//print_info();

	return !(_cntl_reg1 == conf_reg_ret);
}

int
LIS3MDL::set_register_default_values()
{
	write_reg(ADDR_CTRL_REG1, _cntl_reg1);
	write_reg(ADDR_CTRL_REG2, _cntl_reg2);
	write_reg(ADDR_CTRL_REG3, _cntl_reg3);
	write_reg(ADDR_CTRL_REG4, _cntl_reg4);
	write_reg(ADDR_CTRL_REG5, _cntl_reg5);

	return OK;
}

int
LIS3MDL::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
LIS3MDL::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}


void
LIS3MDL::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u ticks", _measure_ticks);
	print_message(_last_report);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace lis3mdl
{

/**
 * list of supported bus configurations
 */
struct lis3mdl_bus_option {
	enum LIS3MDL_BUS busid;
	const char *devpath;
	LIS3MDL_constructor interface_constructor;
	uint8_t busnum;
	LIS3MDL *dev;
} bus_options[] = {
#ifdef PX4_I2C_BUS_EXPANSION
	{ LIS3MDL_BUS_I2C_EXTERNAL, "/dev/lis3mdl_ext", &LIS3MDL_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#endif /* PX4_I2C_BUS_EXPANSION */
#ifdef PX4_I2C_BUS_ONBOARD
	{ LIS3MDL_BUS_I2C_INTERNAL, "/dev/lis3mdl_int", &LIS3MDL_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif /* PX4_I2C_BUS_ONBOARD */
#ifdef PX4_SPIDEV_LIS
	{ LIS3MDL_BUS_SPI, "/dev/lis3mdl_spi", &LIS3MDL_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif /* PX4_SPIDEV_LIS */
};

void    start(enum LIS3MDL_BUS busid, enum Rotation rotation);

int     stop();

bool    start_bus(struct lis3mdl_bus_option &bus, enum Rotation rotation);

bool	init(enum LIS3MDL_BUS busid);

int     calibrate(enum LIS3MDL_BUS busid);

int     info(enum LIS3MDL_BUS busid);

void    test(enum LIS3MDL_BUS busid);

void    reset(enum LIS3MDL_BUS busid);

void    usage();

struct 	lis3mdl_bus_option &find_bus(enum LIS3MDL_BUS busid);

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct lis3mdl_bus_option &bus, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new LIS3MDL(interface, bus.devpath, rotation);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	return true;
}

/**
 * Initialize the driver -- sets defaults and starts a cycle
 */
bool
init(enum LIS3MDL_BUS busid)
{
	struct lis3mdl_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
		return false;

	} else {
		PX4_INFO("Poll rate set to max (80hz)");
	}

	/* Set to 4 Gauss */
	if (OK != ioctl(fd, MAGIOCSRANGE, 4)) {
		PX4_WARN("FAILED: MAGIOCSRANGE 4 Gauss");

	} else {
		PX4_INFO("Set mag range to 4 Gauss");
	}

	close(fd);

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum LIS3MDL_BUS busid, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == LIS3MDL_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != LIS3MDL_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation);
		init(busid);
	}

	if (!started) {
		exit(1);
	}
}

/**
 * Stop the driver.
 */
int
stop()
{
	bool stopped = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != nullptr) {
			bus_options[i].dev->stop();
			delete bus_options[i].dev;
			bus_options[i].dev = nullptr;
			stopped = true;
		}
	}

	return !stopped;
}

/**
 * find a bus structure for a busid
 */
struct
lis3mdl_bus_option &find_bus(enum LIS3MDL_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == LIS3MDL_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum LIS3MDL_BUS busid)
{
	struct lis3mdl_bus_option &bus = find_bus(busid);
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'lis3mdl start')", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
		errx(1, "failed to get if mag is onboard or external");
	}

	/* set the queue depth to 5 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		print_message(report);
	}

	errx(0, "PASS");
}


/**
 * Self test check.
 *
 * Unlike HMC5883, self test feature cannot be used to calculate
 * scale.
 *
 * SELF TEST OPERATION
 * To check the LIS3MDL for proper operation, a self test feature is incorporated :
 * sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias).
 * A few measurements are taken and stored with and without the additional magnetic
 * field. According to ST datasheet, those values must stay between thresholds in order
 * to pass the self test.
 */
int
calibrate(enum LIS3MDL_BUS busid)
{
	int ret;
	struct lis3mdl_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'lis3mdl start' if the driver is not running", path);
	}

	if (OK != (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		PX4_WARN("failed to enable sensor calibration mode");
	}

	close(fd);

	return ret;
}

/**
 * Reset the driver.
 */
void
reset(enum LIS3MDL_BUS busid)
{
	struct lis3mdl_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
int
info(enum LIS3MDL_BUS busid)
{
	struct lis3mdl_bus_option &bus = find_bus(busid);

	PX4_WARN("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
	bus.dev->print_info();
	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'reset', 'info', 'calibrate'");
	PX4_WARN("options:");
	PX4_WARN("    -R rotation");
	PX4_WARN("    -C calibrate on start");
	PX4_WARN("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_LIS)
	PX4_WARN("    -I only internal bus");
#endif
}

} // namespace

int
lis3mdl_main(int argc, char *argv[])
{
	int ch;
	enum LIS3MDL_BUS busid = LIS3MDL_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;

	while ((ch = getopt(argc, argv, "XISR:CT")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_LIS)

		case 'I':
			busid = LIS3MDL_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = LIS3MDL_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = LIS3MDL_BUS_SPI;
			break;

		case 'C':
			calibrate = true;
			break;

		default:
			lis3mdl::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		lis3mdl::start(busid, rotation);

		if (calibrate) {
			if (OK != lis3mdl::calibrate(busid)) {
				errx(1, "calibration failed");
			}

		}

		lis3mdl::init(busid);

		exit(0);
	}

	/*
	 * Stop the driver.
	 */
	if (!strcmp(verb, "stop")) {
		return lis3mdl::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		lis3mdl::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		lis3mdl::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		lis3mdl::info(busid);
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (lis3mdl::calibrate(busid) == 0) {
			errx(0, "calibration successful");

		} else {
			errx(1, "calibration failed");
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'calibrate' 'or 'info'");
}
