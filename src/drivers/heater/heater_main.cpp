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
 * @maintainer Mark Sauder <mark.sauder@tealdrones.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>                     //usleep
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>        //Scheduler
#include <systemlib/err.h>              //print to console

#include "Heater.h"

namespace heater
{

// Function prototypes
static int heater_start();
static void heater_stop();

/**
 * land detector app start / stop handling function
 * This makes the land detector module accessible from the nuttx shell
 * @ingroup apps
 */
extern "C" __EXPORT int heater_main(int argc, char *argv[]);

// Private variables
static Heater *heater_task = nullptr;

/**
 * Stop the task, force killing it if it doesn't stop by itself
 */
static void heater_stop()
{
	if (heater_task == nullptr) {
		PX4_WARN("not running");
		return;
	}

	heater_task->stop();

	// Wait for task to die
	int i = 0;

	do {
		// wait 20ms at a time
		usleep(20000);

	} while (heater_task->is_running() && ++i < 50);


	delete heater_task;
	heater_task = nullptr;
	PX4_WARN("heater has been stopped");
}

/**
 * Start new task, fails if it is already running. Returns OK if successful
 */
static int heater_start()
{
	if (heater_task != nullptr) {
		PX4_WARN("already running");
		return -1;
	}

	heater_task = new Heater();

	// Check if alloc worked
	if (heater_task == nullptr) {
		PX4_WARN("alloc failed");
		return -1;
	}

	// Start new thread task
	int ret = heater_task->start();

	if (ret) {
		PX4_WARN("task start failed: %d", -errno);
		return -1;
	}

	// Avoid memory fragmentation by not exiting start handler until the task has fully started
	const uint64_t timeout = hrt_absolute_time() + 5000000; // 5 second timeout

	// Do one sleep before the first check
	usleep(10000);

	if (!heater_task->is_running()) {
		while (!heater_task->is_running()) {
			usleep(50000);

			if (hrt_absolute_time() > timeout) {
				PX4_WARN("start failed - timeout");
				heater_stop();
				return 1;
			}
		}
	}

	return 0;
}

/**
 * Main entry point for this module
 */
int heater_main(int argc, char *argv[])
{
	if (argc < 2) {
		goto exiterr;
	}

	// Start the heater driver task.
	if (!strcmp(argv[1], "start")) {
		if (heater_start() != 0) {
			PX4_INFO("heater start failed");
			return 1;
		}

		return 0;
	}

	// Stop the heater driver task.
	if (!strcmp(argv[1], "stop")) {
		heater_stop();
		return 0;
	}

	// Display the heater driver status.
	if (!strcmp(argv[1], "status")) {
		if (heater_task) {
			if (heater_task->is_running()) {
				float current_temperature = heater_task->get_current_temperature();
				float target_temperature = heater_task->get_target_temperature();
				bool heater_state = heater_task->get_state();

				PX4_INFO("Temp: %3.3f - Target Temp: %3.2f - Heater State: %s",
					 (double)current_temperature,
					 (double)target_temperature,
					 heater_state ? "On" : "Off");
			}

			return 0;

		} else {
			PX4_WARN("Heater driver not running");
			return 1;
		}
	}

	// Display the IMU temperature.
	if (!strcmp(argv[1], "temp")) {
		float current_temp = heater_task->get_current_temperature();
		PX4_INFO("Current Temp:  %3.3f", (double)current_temp);
		return 0;
	}

	// Display the IMU target temperature.
	if (!strcmp(argv[1], "target_temp")) {
		float target_temp = 0.f;

		if (argv[2]) {
			target_temp = atof(argv[2]);
			target_temp = heater_task->set_target_temperature(target_temp);

		} else {

			target_temp = heater_task->get_target_temperature();
		}

		PX4_INFO("Target Temp:  %3.3f", (double)target_temp);
		return 0;
	}

	// Display/Set the heater driver feed forward value.
	if (!strcmp(argv[1], "feed_forward")) {
		float feed_forward = 0.f;

		if (argv[2]) {
			feed_forward = atof(argv[2]);
			feed_forward = heater_task->set_feed_forward(feed_forward);

		} else {
			feed_forward = heater_task->get_feed_forward();
		}

		PX4_INFO("Feed Forward Value:  %2.5f", (double)feed_forward);
		return 0;
	}

	// Display/Set the heater driver proportional gain value.
	if (!strcmp(argv[1], "proportional")) {
		float proportional_gain = 0.f;

		if (argv[2]) {
			proportional_gain = atof(argv[2]);
			proportional_gain = heater_task->set_proportional(proportional_gain);

		} else {
			proportional_gain = heater_task->get_proportional();
		}

		PX4_INFO("Proportional Gain:  %2.5f", (double)proportional_gain);
		return 0;
	}

	// Display/Set the heater driver integrator gain value.
	if (!strcmp(argv[1], "integrator")) {
		float integrator_gain = 0.f;

		if (argv[2]) {
			integrator_gain = atof(argv[2]);
			integrator_gain = heater_task->set_integrator(integrator_gain);

		} else {
			integrator_gain = heater_task->get_integrator();
		}

		PX4_INFO("Integrator Gain:  %2.5f", (double)integrator_gain);
		return 0;
	}

	// Display/Set the heater controller period value (usec).
	if (!strcmp(argv[1], "controller_period")) {
		float controller_period_usec = 0.f;

		if (argv[2]) {
			controller_period_usec = atof(argv[2]);
			controller_period_usec = heater_task->set_controller_period(controller_period_usec);

		} else {
			controller_period_usec = heater_task->get_controller_period();
		}

		PX4_INFO("Controller period (usec):  %2.5f", (double)controller_period_usec);
		return 0;
	}

	// Display the heater driver information/argument list.
	if (!strcmp(argv[1], "help") || !strcmp(argv[1], "info")) {
		PX4_INFO("\n\tstart             - Starts the Heater driver."
			 "\n\tstop              - Stops the Heater driver."
			 "\n\tstatus            - Displays the current IMU temperature, target temperature, and heater on/off status."
			 "\n\ttemp              - Displays the current IMU temperature."
			 "\n\ttarget_temp       - Displays the current IMU temperature."
			 "\n\tfeed_forward      - Without argument displays the feed_forward gain value."
			 "\n\t                  - With float value argument sets and displays the feed_forward gain value."
			 "\n\tproportional      - Without argument displays the proportional gain value."
			 "\n\t                  - With float value argument sets and displays the proportional gain value."
			 "\n\tintegrator        - Without argument displays the integrator gain value."
			 "\n\t                  - With float value argument sets and displays the integrator gain value."
			 "\n\tcontroller_period - Without argument displays the heater driver cycle period value (microseconds)."
			 "\n\t                  - With int value argument sets and displays the heater driver cycle period value (microseconds).");
		return 0;
	}

exiterr:
	PX4_INFO("Usage: heater {start|stop|status|temp|target_temp|feed_forward|proportional|integrator|controller_period|help}");
	return 1;
}

}
