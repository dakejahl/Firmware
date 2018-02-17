/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file Heater.h
 *
 * @author Khoi Tran <khoi@tealdrones.com>
 */

#pragma once

#include <px4_workqueue.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_gyro.h>

namespace heater
{

class Heater
{
public:

	Heater();
	virtual ~Heater();

	/**
	 * @return true if this task is currently running.
	 */
	inline bool is_running() const
	{
		return _task_is_running;
	}

	/**
	 * Tells the task that it should exit.
	 */
	void stop();

	/**
	 * Get the work queue going.
	 */
	int start();

	/**
	 * @return current temperature
	 */
	int get_current_temperature() const
	{
		return (int)_current_temp;
	}

	/**
	 * @return target temperature
	 */
	int get_target_temperature() const
	{
		return (int)_target_temp;
	}

	/**
	 * @return heater state
	 */
	bool get_state() const
	{
		return _heater_on;
	}

protected:
	/**
	 * Called once to initialize uORB topics.
	 */
	void _initialize_topics();

	/**
	 * Update uORB topics.
	 */
	void _update_topics();


	/**
	 * Update parameters.
	 */
	void _update_params();

	int _parameter_sub;

	static bool _orb_update(const struct orb_metadata *meta, int handle, void *buffer);

private:
	static void _cycle_trampoline(void *arg);

	void _cycle();

	void _check_params(const bool force);

	/** @param _task_should_exit */
	bool _task_should_exit;

	/** @param _task_is_running */
	bool _task_is_running;

	/** @param _p_target_temp */
	param_t _p_target_temp;

	/** @param _current_temp */
	float _current_temp;

	/** @param _error_temp The error between current and target temperatures. */
	float _error_temp;

	/** @param _target_temp */
	float _target_temp;

	/** @param _temp_proportional_gain */
	float _temp_proportional_gain;

	/** @param _temp_integrator_gain */
	float _temp_integrator_gain;

	/** @param _temp_proportional_value */
	float _temp_proportional_value;

	/** @param _temp_integrator_value */
	float _temp_integrator_value;

	/** @param _heater_on_time */
	float _heater_on_time_sec;

	/** @param _heater_on_time */
	int _heater_on_time_usec;

	/** @param _heater_on */
	bool _heater_on;

	/** @param _sensor_gyro_sub */
	int _sensor_gyro_sub;

	/** @param _sensor_gyro */
	struct sensor_gyro_s _sensor_gyro;

	/** @param _work */
	struct work_s	_work;
};

} // namespace heater
