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
#include <uORB/topics/sensor_accel.h>

namespace heater
{

class Heater
{
public:

	Heater();
	virtual ~Heater();

	/**
	 * @brief
	 * @return true if this task is currently running.
	 */
	inline bool is_running() const
	{
		return _task_is_running;
	}

	/**
	 * @brief Tells the task to exit.
	 */
	void stop();

	/**
	 * @brief Initiates the heater driver work queue.
	 * return Returns 1 iff start was successful.
	 */
	int start();

	/**
	 * @brief Gets the heater driver state.
	 * @return Returns the heater driver state.
	 */
	bool get_state();

	/**
	 * @brief Gets the accelerometer current temperature.
	 * @return Returns the accelerometer current temperature.
	 */
	float get_current_temperature();

	/**
	 * @brief Gets the heater target temperature.
	 * @return Returns the heater target temperature
	 */
	float get_target_temperature();

	/**
	 * @brief Sets the heater target temperature.
	 * @return Returns the heater target temperature.
	 */
	float set_target_temperature(float target_temperature);

	/**
	 * @brief Sets the heater proportional gain value.
	 * @return Returns the heater proportional gain value.
	 */
	float set_proportional(float proportional_gain);

	/**
	 * @brief Gets the heater proportional gain value.
	 * @return Returns the heater proportional gain value.
	 */
	float get_proportional();

	/**
	 * @brief Sets the heater integrator gain value.
	 * @return Returns the heater integrator gain value.
	 */
	float set_integrator(float integrator_gain);

	/**
	 * @brief Gets the heater integrator gain value.
	 * @return Returns the heater integrator gain value.
	 */
	float get_integrator();

	/**
	 * @brief Sets the heater feed forward value.
	 * @return Returns the heater feed forward value.
	 */
	float set_feed_forward(float feed_forward_gain);

	/**
	 * @brief Gets the heater feed forward value.
	 * @return Returns the heater feed forward value.
	 */
	float get_feed_forward();

	/**
	 * @brief Sets the heater cycle period value in microseconds.
	 * @return Returns the heater cycle period value in microseconds.
	 */
	int set_controller_period(int controller_period_usec);

	/**
	 * @brief Gets the heater cycle period value in microseconds.
	 * @return Returns the heater cycle period value in microseconds.
	 */
	int get_controller_period();

	/**
	 * @brief Gets the average heater on duty cycle as a percent.
	 * @return Returns the average heater on cycle duty cycle as a percent.
	 */
	float get_duty_cycle();

protected:

	/**
	 * @brief Updates the uORB topics for local subscribers.
	 * @return Returns true iff update was successful.
	 */
	static bool _orb_update(const struct orb_metadata *meta, int handle, void *buffer);

	/**
	 * @brief Called once to initialize uORB topics.
	 */
	void _initialize_topics();

	/**
	 * @brief Updates uORB subscription topics.
	 */
	void _update_topics();

	/**
	 * @brief Updates subscription parameters.
	 */
	void _update_params();

	/** @param Local member variable to store the parameter subscriptions. */
	int _parameter_sub;

private:
	static void _heater_controller_trampoline(void *arg);

	/**
	 * @brief
	 */
	void _heater_controller();

	/**
	 * @brief Checks
	 */
	void _check_params(const bool force);

	/** @param _task_should_exit Indicator flag to stop the heater driver process. */
	bool _task_should_exit;

	/** @param _task_is_running Indicator flag for when the driver is running. */
	bool _task_is_running;

	/** @param _p_target_temp The heater controller temperature setpoint target parameter. */
	param_t _p_target_temp;

	/** @param _current_temp The accelerometer measured current temperature. */
	float _current_temp;

	/** @param _error_temp The error between current and target temperatures. */
	float _error_temp;

	/** @param _target_temp The heater controller temperature setpoint target. */
	float _target_temp;

	/** @param _proportional_gain The heater controller proportional gain value. */
	float _proportional_gain;

	/** @param _integrator_gain The heater controller integrator gain value. */
	float _integrator_gain;

	/** @param _proportional_value The heater controller proportional value. */
	float _proportional_value;

	/** @param _integrator_value The heater controller integrator value. */
	float _integrator_value;

	/** @param _feed_forward The heater controller feedforward value. */
	float _feed_forward;

	/** @param _duty_cycle The heater on duty cycle value. */
	float _duty_cycle;

	/** @param _controller_period_usec The heater controller time period in microseconds.*/
	int _controller_period_usec;

	/** @param _controller_time_on_usec The heater time on in microseconds.*/
	int _controller_time_on_usec;

	/** @param _heater_on Indicator for the heater on/off status. */
	bool _heater_on;

	/** @param _sensor_accel_sub The accelerometer subtopic subscribed to.*/
	int _sensor_accel_sub;

	/** @struct _sensor_accel Accelerometer struct to receive uORB accelerometer data. */
	struct sensor_accel_s _sensor_accel;

	/** @struct _work Work Queue struct for the RTOS scheduler. */
	struct work_s _work;
};

} // namespace heater
