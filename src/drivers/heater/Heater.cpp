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
 * @file Heater.cpp
 *
 * @author Khoi Tran <khoi@tealdrones.com>
 */

#include <drivers/drv_hrt.h>
#include <float.h>

#include <mathlib/mathlib.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <uORB/topics/parameter_update.h>

#include "Heater.h"

namespace heater
{

Heater::Heater() :
	_parameter_sub(0),
	_task_should_exit(false),
	_task_is_running(false),
	_current_temp(0.0f),
	_error_temp(0.0f),
	_target_temp(0.0f),
	_proportional_gain(0.25),
	_integrator_gain(0.025),
	_proportional_value(0.0f),
	_integrator_value(0.0f),
	_feed_forward(0.15),
	_duty_cycle(0.15f),
	_controller_period_usec(100000),
	_controller_time_on_usec(0),
	_heater_on(false),
	_sensor_accel_sub(-1),
	_sensor_accel{},
	_work{}
{
	_p_target_temp = param_find("SENS_IMU_TEMP");
	px4_arch_configgpio(GPIO_HEATER);
	px4_arch_gpiowrite(GPIO_HEATER, 0);
}

Heater::~Heater()
{
	work_cancel(LPWORK, &_work);
	_task_should_exit = true;
}

int Heater::start()
{
	_task_should_exit = false;

	_check_params(true);

	if (px4_arch_gpioread(GPIO_HEATER)) {
		_heater_on = false;
		px4_arch_gpiowrite(GPIO_HEATER, 0);
	}

	// Schedule a cycle to start the driver.
	work_queue(LPWORK, &_work, (worker_t)&Heater::_heater_controller_trampoline, this, 0);

	return 0;
}

void Heater::stop()
{
	_task_should_exit = true;
}

void
Heater::_heater_controller_trampoline(void *arg)
{
	Heater *dev = reinterpret_cast<Heater *>(arg);

	dev->_heater_controller();
}

void Heater::_heater_controller()
{
	if (!_task_is_running) {
		// Initialize uORB topics.
		_initialize_topics();

		_check_params(true);

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
			usleep(50000);
			px4_arch_configgpio(GPIO_HEATER);
			usleep(50000);
			px4_arch_gpiowrite(GPIO_HEATER, 0);
		}
	}

	else {
		_check_params(false);
		_update_topics();

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
			work_queue(LPWORK, &_work, (worker_t)&Heater::_heater_controller_trampoline, this,
				   USEC2TICK(_controller_time_on_usec));

		} else {
			work_queue(LPWORK, &_work, (worker_t)&Heater::_heater_controller_trampoline, this,
				   USEC2TICK(_controller_period_usec - _controller_time_on_usec));
		}

	} else {
		_task_is_running = false;
	}
}

void Heater::_initialize_topics()
{
	// @TODO Update to support loading in sensor ID's. Add PARAM to select which sensor is read from.
	_sensor_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 1);
}

void Heater::_update_topics()
{
	_orb_update(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel);
}

void Heater::_update_params()
{
}

void Heater::_check_params(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameter_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameter_sub, &paramUpdate);
	}

	if (updated || force) {
		_update_params();
		param_get(_p_target_temp, &_target_temp);
	}
}

bool Heater::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
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

float Heater::set_proportional(float proportional_gain)
{
	_proportional_gain = proportional_gain;
	return _proportional_gain;
}

float Heater::get_proportional()
{
	return _proportional_gain;
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

float Heater::set_feed_forward(float feed_forward_gain)
{
	_feed_forward = feed_forward_gain;
	return _feed_forward;
}

float Heater::get_feed_forward()
{
	return _feed_forward;
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
