/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/*
 * @file heater.cpp
 *
 * @author Khoi Tran <khoi@tealdrones.com>
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include <float.h>

#include <mathlib/mathlib.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <uORB/topics/parameter_update.h>

#include "heater.h"

namespace heater
{

Heater::Heater() :
	_parameter_sub(0),
	_task_should_exit(false),
	_task_is_running(false),
	_current_temp(0.0f),
	_error_temp(0.0f),
	_target_temp(0.0f),
	_proportional_gain(0.25f),
	_integrator_gain(0.025f),
	_proportional_value(0.0f),
	_integrator_value(0.0f),
	_feed_forward(0.25f),
	_duty_cycle(0.25f),
	_controller_period_usec(100000),
	_controller_time_on_usec(0),
	_heater_on(false),
	_sensor_accel_sub(-1),
	_sensor_accel{},
	_work{}
{
	_p_target_temp = param_find("SENS_IMU_TEMP");
	_p_sensor_id = param_find("SENS_TEMP_ID");
	px4_arch_configgpio(GPIO_HEATER);
}

Heater::~Heater()
{
	_task_should_exit = true;
	work_cancel(LPWORK, &_work);
	px4_arch_gpiowrite(GPIO_HEATER, 0);

	// Check if GPIO is stuck on, and if so, configure it as an input pulldown then reconfigure as an output.
	if (px4_arch_gpioread(GPIO_HEATER)) {
		px4_arch_configgpio(GPIO_HEATER_INPUT);
		px4_arch_configgpio(GPIO_HEATER);
		px4_arch_gpiowrite(GPIO_HEATER, 0);
	}
}

void Heater::check_params(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameter_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameter_sub, &paramUpdate);
	}

	if (updated || force) {
		param_get(_p_target_temp, &_target_temp);
	}
}

void
Heater::heater_controller_trampoline(void *arg)
{
	Heater *dev = reinterpret_cast<Heater *>(arg);

	dev->heater_controller();
}

void Heater::heater_controller()
{
	if (!_task_is_running) {
		// Initialize uORB topics.
		initialize_topics();

		check_params(true);

		// Task is now running, keep doing so until we need to stop.
		_task_is_running = true;
	}

	if (_heater_on) {
		// Turn the heater off.
		px4_arch_gpiowrite(GPIO_HEATER, 0);
		_heater_on = false;

		// Check if GPIO is stuck on, and if so, configure it as an input pulldown then reconfigure as an output.
		if (px4_arch_gpioread(GPIO_HEATER)) {
			px4_arch_configgpio(GPIO_HEATER_INPUT);
			px4_arch_configgpio(GPIO_HEATER);
			px4_arch_gpiowrite(GPIO_HEATER, 0);
		}
	}

	else {
		check_params(false);
		update_topics();

		// Determine the current temperature error.
		_current_temp = _sensor_accel.temperature;

		_error_temp = _target_temp - _current_temp;

		// Modulate the heater time on with a feedforward/PI controller.
		_proportional_value = _error_temp * _proportional_gain;
		_integrator_value += _error_temp * _integrator_gain;
		_integrator_value = math::max(math::min(_integrator_value, 0.25f), -0.25f);

		_controller_time_on_usec = (int)((_feed_forward + _proportional_value +
						  _integrator_value) * (float)_controller_period_usec);

		// Ensure the heater time on is clamped within the time allowed.
		_controller_time_on_usec = math::max(math::min(_controller_period_usec, _controller_time_on_usec), 0);

		_duty_cycle = (0.05f * ((float)_controller_time_on_usec / (float)_controller_period_usec)) + (0.95f * _duty_cycle);

		// Turn the heater on.
		_heater_on = true;
		px4_arch_gpiowrite(GPIO_HEATER, 1);
	}


	if (!_task_should_exit) {

		// Schedule next cycle.
		if (_heater_on) {
			work_queue(LPWORK, &_work, (worker_t)&Heater::heater_controller_trampoline, this,
				   USEC2TICK(_controller_time_on_usec));

		} else {
			work_queue(LPWORK, &_work, (worker_t)&Heater::heater_controller_trampoline, this,
				   USEC2TICK(_controller_period_usec - _controller_time_on_usec));
		}

	} else {
		_task_is_running = false;
	}
}

void Heater::initialize_topics()
{
	int32_t sensor_id;
	param_get(_p_sensor_id, &sensor_id);

	// Get the total number of accelerometer instances
	size_t num_of_imu = orb_group_count(ORB_ID(sensor_accel));

	// Check each instance for the correct ID
	for (size_t x = 0; x < num_of_imu; x++) {
		_sensor_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), (int)x);

		while (orb_update(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel) != true) {
			usleep(200000);
		}

		// If the correct ID is found, exit the for loop leaving _sensor_accel_sub pointing to the correct instance
		if (_sensor_accel.device_id == (uint32_t)sensor_id) {
			PX4_INFO("Found Sensor to Temp Compensate");
			break;
		}
	}

	PX4_INFO("Device ID:  %d", _sensor_accel.device_id);

	// Exit the driver if the sensor ID does not match the desired sensor
	if (_sensor_accel.device_id != (uint32_t)sensor_id) {
		_task_should_exit = true;
		PX4_INFO("Could not find sensor to control temperature");
	}
}

bool Heater::is_running()
{
	return _task_is_running;
}

int Heater::start()
{
	_task_should_exit = false;

	check_params(true);

	// Schedule a cycle to start the driver.
	work_queue(LPWORK, &_work, (worker_t)&Heater::heater_controller_trampoline, this, 0);

	return 0;
}

void Heater::stop()
{
	_task_should_exit = true;
}

void Heater::update_topics()
{
	orb_update(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel);
}

bool Heater::orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// check if there is new data to grab
	if (orb_check(handle, &newData) != OK) {
		return false;
	}

	if (!newData) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}

float Heater::set_feed_forward(float feed_forward_gain)
{
	_feed_forward = feed_forward_gain;
	return _feed_forward;
}

float Heater::get_feed_forward()
{
	return _feed_forward;
}

float Heater::set_integrator(float integrator_gain)
{
	_integrator_gain = integrator_gain;
	return _integrator_gain;
}

float Heater::get_integrator()
{
	return _integrator_gain;
}

float Heater::set_proportional(float proportional_gain)
{
	_proportional_gain = proportional_gain;
	return _proportional_gain;
}

float Heater::get_proportional()
{
	return _proportional_gain;
}

bool Heater::get_state()
{
	return _heater_on;
}

float Heater::get_current_temperature()
{
	return _current_temp;
}

float Heater::get_target_temperature()
{
	return _target_temp;
}

uint32_t Heater::get_target_id()
{
	return _sensor_accel.device_id;
}

float Heater::set_target_temperature(float target_temperature)
{
	_target_temp = target_temperature;
	return _target_temp;
}

int Heater::set_controller_period(int controller_period_usec)
{
	_controller_period_usec = controller_period_usec;
	return _controller_period_usec;
}

int Heater::get_controller_period()
{
	return _controller_period_usec;
}

float Heater::get_duty_cycle()
{
	return _duty_cycle;
}

} // namespace heater
