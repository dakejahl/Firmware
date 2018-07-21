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
 * @file heater_main.cpp
 * Heater main
 *
 * @author Khoi Tran <khoi@tealdrones.com>
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 */

#include <string.h>
// #include <stdlib.h>
#include <errno.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

#include "heater_main.h"

/**
 * @brief IMU Heater Controller driver used to maintain consistent
 *        temparature at the IMU.
 */
extern "C" __EXPORT int heater_main(int argc, char *argv[]);

static int
heater::controller_period(char *argv[])
{
	if (heater_task->is_running()) {
		float controller_period_usec = 0.f;

		if (argv[2]) {
			controller_period_usec = atof(argv[2]);
			controller_period_usec = heater_task->set_controller_period(controller_period_usec);

		} else {
			controller_period_usec = heater_task->get_controller_period();
		}

		PX4_INFO("Controller period (usec):  %2.5f", (double)controller_period_usec);
		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

static int
heater::duty_cycle()
{
	if (heater_task->is_running()) {
		float duty_cycle = heater_task->get_duty_cycle();
		duty_cycle *= 100.f;
		PX4_INFO("Average duty cycle:  %3.1f%%", (double)duty_cycle);
		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

static int
heater::info()
{
	PX4_INFO("\n\tstart             - Starts the Heater driver."
		 "\n\tstop              - Stops the Heater driver."
		 "\n\tstatus            - Displays the current IMU temperature, target temperature, and heater on/off status."
		 "\n\ttemp              - Displays the current IMU temperature."
		 "\n\ttarget_temp       - Displays the current IMU temperature."
		 "\n\tsensor_id         - Displays the current IMU the heater is temperature controlling."
		 "\n\tfeed_forward      - Without argument displays the feed_forward gain value."
		 "\n\t                  - With float value argument sets and displays the feed_forward gain value."
		 "\n\tproportional      - Without argument displays the proportional gain value."
		 "\n\t                  - With float value argument sets and displays the proportional gain value."
		 "\n\tintegrator        - Without argument displays the integrator gain value."
		 "\n\t                  - With float value argument sets and displays the integrator gain value."
		 "\n\tcontroller_period - Without argument displays the heater driver cycle period value (microseconds)."
		 "\n\t                  - With int value argument sets and displays the heater driver cycle period value (microseconds)."
		 "\n\tduty_cycle        - Displays the heater duty cycle (%%).");
	return PX4_OK;
}

static int
heater::get_sensor_id()
{
	if (heater_task->is_running()) {
		uint32_t id = heater_task->get_target_id();
		PX4_INFO("Sensor ID:  %d", id);
		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

static int
heater::get_temperature()
{
	if (heater_task->is_running()) {
		float current_temp = heater_task->get_current_temperature();
		PX4_INFO("Current Temp:  %3.3f", (double)current_temp);
		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

static int
heater::feed_forward(char *argv[])
{
	if (heater_task->is_running()) {
		float feed_forward = 0.f;

		if (argv[2]) {
			feed_forward = atof(argv[2]);
			feed_forward = heater_task->set_feed_forward(feed_forward);

		} else {
			feed_forward = heater_task->get_feed_forward();
		}

		PX4_INFO("Feed Forward Value:  %2.5f", (double)feed_forward);
		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

static int
heater::integrator(char *argv[])
{
	if (heater_task->is_running()) {
		float integrator_gain = 0.f;

		if (argv[2]) {
			integrator_gain = atof(argv[2]);
			integrator_gain = heater_task->set_integrator(integrator_gain);

		} else {
			integrator_gain = heater_task->get_integrator();
		}

		PX4_INFO("Integrator Gain:  %2.5f", (double)integrator_gain);
		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

static int
heater::proportional(char *argv[])
{
	if (heater_task->is_running()) {
		float proportional_gain = 0.f;

		if (argv[2]) {
			proportional_gain = atof(argv[2]);
			proportional_gain = heater_task->set_proportional(proportional_gain);

		} else {
			proportional_gain = heater_task->get_proportional();
		}

		PX4_INFO("Proportional Gain:  %2.5f", (double)proportional_gain);
		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

static int
heater::start()
{
	if (heater_task != nullptr) {
		PX4_INFO("Heater driver already running");
		return PX4_ERROR;
	}

	heater_task = new Heater();

	// Check if alloc worked
	if (heater_task == nullptr) {
		PX4_WARN("Heater driver alloc failed: %d", -errno);
		return PX4_ERROR;
	}

	// Start new thread task
	int ret = heater_task->start();

	if (ret) {
		PX4_WARN("Heater driver task thread start failed: %d", -errno);
		return PX4_ERROR;
	}

	// Avoid memory fragmentation by not exiting start handler until the task has fully started
	const uint64_t timeout = hrt_absolute_time() + 5000000; // 5 second timeout

	// Do one sleep before the first check
	usleep(10000);

	if (!heater_task->is_running()) {
		while (!heater_task->is_running()) {
			usleep(50000);

			if (hrt_absolute_time() > timeout) {
				PX4_WARN("Heater driver start failed - Timeout");
				stop();
				return PX4_ERROR;
			}
		}
	}

	PX4_INFO("Heater driver started successfully.");
	return PX4_OK;
}

static int
heater::status()
{
	if (heater_task->is_running()) {
		float current_temperature = heater_task->get_current_temperature();
		float target_temperature = heater_task->get_target_temperature();
		bool heater_state = heater_task->get_state();

		PX4_INFO("Temp: %3.3f - Target Temp: %3.2f - Heater State: %s",
			 (double)current_temperature,
			 (double)target_temperature,
			 heater_state ? "On" : "Off");

		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

static int
heater::stop()
{
	if (!heater_task->is_running()) {
		PX4_WARN("Heater driver not running");
		return PX4_OK;
	}

	heater_task->stop();

	// Wait for task to die
	size_t counter = 0;

	while (heater_task->is_running()) {

		counter++;

		if (counter > 50) {
			PX4_WARN("Heater driver was not successfully stopped");
			return PX4_ERROR;
		}

		// wait 20ms at a time
		usleep(20000);
	}


	delete heater_task;
	heater_task = nullptr;

	PX4_INFO("Heater driver successfully stopped");
	return PX4_OK;
}

static int
heater::target_temperature(char *argv[])
{
	if (heater_task->is_running()) {
		float target_temp = 0.f;

		if (argv[2]) {
			target_temp = atof(argv[2]);
			target_temp = heater_task->set_target_temperature(target_temp);

		} else {

			target_temp = heater_task->get_target_temperature();
		}

		PX4_INFO("Target Temp:  %3.3f", (double)target_temp);
		return PX4_OK;

	} else {
		PX4_WARN("Heater driver not running");
		return PX4_ERROR;
	}
}

/**
 * Main entry point for the heater driver module
 */
int
heater_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("Usage: heater {start|stop|status|temp|target_temp|feed_forward|proportional"\
			 "|integrator|controller_period|duty_cycle|sensor_id|info}");
		return PX4_ERROR;
	}

	const char *arg_vector = argv[1];

	// Start the heater driver task.
	if (!strcmp(arg_vector, "start")) {
		return heater::start();
	}

	// Display the heater driver status.
	if (!strcmp(arg_vector, "status")) {
		return heater::status();
	}

	// Stop the heater driver task.
	if (!strcmp(arg_vector, "stop")) {
		return heater::stop();
	}

	// Display the IMU temperature.
	if (!strcmp(arg_vector, "temp")) {
		return heater::get_temperature();
	}

	// Display the IMU target temperature.
	if (!strcmp(arg_vector, "target_temp")) {
		return heater::target_temperature(argv);
	}

	// Display the id of the sensor we are controlling temperature on.
	if (!strcmp(arg_vector, "sensor_id")) {
		return heater::get_sensor_id();
	}

	// Display/Set the heater driver feed forward value.
	if (!strcmp(arg_vector, "feed_forward")) {
		return heater::feed_forward(argv);
	}

	// Display/Set the heater driver integrator gain value.
	if (!strcmp(arg_vector, "integrator")) {
		return heater::integrator(argv);
	}

	// Display/Set the heater driver proportional gain value.
	if (!strcmp(arg_vector, "proportional")) {
		return heater::proportional(argv);
	}

	// Display/Set the heater controller period value (usec).
	if (!strcmp(arg_vector, "controller_period")) {
		return heater::controller_period(argv);
	}

	// Display the heater on duty cycle as a percent.
	if (!strcmp(arg_vector, "duty_cycle")) {
		return heater::duty_cycle();
	}

	// Display the heater driver information/argument list.
	if (!strcmp(arg_vector, "usage") ||
	    !strcmp(arg_vector, "info")) {
		return heater::info();
	}

	return PX4_OK;
}
