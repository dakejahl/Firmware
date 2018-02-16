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
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>					//usleep
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>	//Scheduler
#include <systemlib/err.h>			//print to console

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

	if (!strcmp(argv[1], "start")) {
		if (heater_start() != 0) {
			PX4_WARN("heater start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		heater_stop();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (heater_task) {

			if (heater_task->is_running()) {
				int current_temperature;
				int target_temperature;
				bool heater_state;

				current_temperature = heater_task->get_current_temperature();
				target_temperature = heater_task->get_target_temperature();
				heater_state = heater_task->get_state();

				PX4_WARN("Temp: %d - Targ.Temp: %d - Heater State: %s",
					 current_temperature,
					 target_temperature,
					 heater_state ? "On" : "Off");
			}

			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

exiterr:
	PX4_WARN("usage: heater {start|stop|status}");
	return 1;
}

}
