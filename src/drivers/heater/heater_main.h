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
 * @file heater_main.h
 * Heater main
 *
 * @author Mark Sauder <mark.sauder@tealdrones.com>
 */

#include <getopt.h>

#include "heater.h"

#ifndef ERROR
#define ERROR -1
#endif

namespace heater
{
/**
 * @brief Static Heater type pointer to the heater_task.
 */
static Heater *heater_task = nullptr;

/**
 * @brief Sets and/or reports the heater controller time period value in microseconds.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int controller_period(char *argv[]);

/**
 * @brief Reports the average heater controller duty cycle period in microseconds.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int duty_cycle();

/**
 * @brief Prints relevant heater driver usage information to the terminal.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int info();

/**
 * @brief Reports the heater temperature sensor ID.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int get_sensor_id();

/**
 * @brief Reports the heater temperature.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int get_temperature();

/**
 * @brief Sets and/or reports the heater controller feed fordward value.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int feed_forward(char *argv[]);

/**
 * @brief Sets and/or reports the heater controller integrator gain value.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int integrator(char *argv[]);

/**
 * @brief Sets and/or reports the heater controller proportional gain value.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int proportional(char *argv[]);

/**
 * @brief Starts new task, fails if it is already running.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int start();

/**
 * @brief Reports the curent status and diagnostic information about the heater driver.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int status();

/**
 * @brief Stops the task. Forces killing the task if it doesn't stop by itself.
 * @return Returns 0 iff successful, -1 otherwise.
 */
static int stop();

/**
 * @brief Sets and/or reports the heater target temperature.
 * @brief Returns 0 iff successful, -1 otherwise.
 */
static int target_temperature(char *argv[]);
}