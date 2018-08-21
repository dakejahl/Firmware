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

#include "heater.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>

#ifndef GPIO_HEATER_INPUT
#error "To use the heater driver, the board_config.h must define and initialize GPIO_HEATER_INPUT and GPIO_HEATER_OUTPUT"
#endif

struct work_s Heater::_work = {};

Heater::Heater() :
	ModuleParams(nullptr),
	_controller_period_usec(CONTROLLER_PERIOD_DEFAULT),
	_controller_time_on_usec(0),
	_duty_cycle(0.0f),
	_heater_on(false),
	_integrator_value(0.0f),
	_parameter_subscription(0),
	_proportional_value(0.0f),
	_sensor_accel(sensor_accel_s{}),
	_sensor_accel_subscription(-1),
	_sensor_temperature(0.0f)
{
	px4_arch_configgpio(GPIO_HEATER_OUTPUT);
	px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
}

Heater::~Heater()
{
	// Tell the work queue to exit.
	request_stop();
	work_cancel(LPWORK, &_work);

	// Drive the heater GPIO pin low.
	px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);

	// Verify if GPIO is low, and if not, configure it as an input pulldown then reconfigure as an output.
	if (px4_arch_gpioread(GPIO_HEATER_OUTPUT)) {
		px4_arch_configgpio(GPIO_HEATER_INPUT);
		px4_arch_configgpio(GPIO_HEATER_OUTPUT);
		px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
	}
}

int Heater::controller_period()
{
	return _controller_period_usec;
}

int Heater::controller_period(char *argv[])
{
	int controller_period_usec = 0;

	if (argv[2]) {
		controller_period_usec = atoi(argv[2]);
		controller_period_usec = this->controller_period(controller_period_usec);

	} else {
		controller_period_usec = this->controller_period();
	}

	PX4_INFO("Controller period (usec):  %2.5f", (double)controller_period_usec);
	return controller_period_usec;
}

int Heater::controller_period(int controller_period_usec)
{
	_controller_period_usec = controller_period_usec;
	return _controller_period_usec;
}

int Heater::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running() && !_object) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	const char *arg_v = argv[0];

	// Display/Set the heater controller period value (usec).
	if (strcmp(arg_v, "controller_period") == 0) {
		return get_instance()->controller_period(argv);
	}

	// Display the heater on duty cycle as a percent.
	if (strcmp(arg_v, "duty_cycle") == 0) {
		return get_instance()->duty_cycle();
	}

	// Display/Set the heater driver feed forward value.
	if (strcmp(arg_v, "feed_forward") == 0) {
		return get_instance()->feed_forward(argv);
	}

	// Display/Set the heater driver integrator gain value.
	if (strcmp(arg_v, "integrator") == 0) {
		return get_instance()->integrator(argv);
	}

	// Display/Set the heater driver proportional gain value.
	if (strcmp(arg_v, "proportional") == 0) {
		return get_instance()->proportional(argv);
	}

	// Display the id of the sensor we are controlling temperature on.
	if (strcmp(arg_v, "sensor_id") == 0) {
		return get_instance()->sensor_id();
	}

	// Displays/Set the current IMU temperature setpoint.
	if (strcmp(arg_v, "setpoint") == 0) {
		return get_instance()->temperature_setpoint(argv);
	}

	// Displays the IMU reported temperature.
	if (strcmp(arg_v, "temperature") == 0) {
		return get_instance()->sensor_temperature();
	}

	get_instance()->print_usage("Unrecognized command.");
	return PX4_OK;
}

void Heater::cycle()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_heater_on) {
		// Turn the heater off.
		px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
		_heater_on = false;

		// Check if GPIO is stuck on, and if so, configure it as an input pulldown then reconfigure as an output.
		if (px4_arch_gpioread(GPIO_HEATER_OUTPUT)) {
			px4_arch_configgpio(GPIO_HEATER_INPUT);
			px4_arch_configgpio(GPIO_HEATER_OUTPUT);
			px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
		}

	} else {
		update_params(false);

		orb_update(ORB_ID(sensor_accel), _sensor_accel_subscription, &_sensor_accel);

		// Obtain the current IMU sensor temperature.
		_sensor_temperature = _sensor_accel.temperature;

		// Calculate the temperature delta between the setpoint and reported temperature.
		float temperature_delta = _p_temperature_setpoint.get() - _sensor_temperature;

		// Modulate the heater time on with a feedforward/PI controller.
		_proportional_value = temperature_delta * _p_proportional_gain.get();
		_integrator_value += temperature_delta * _p_integrator_gain.get();

		// Constrain the integrator value to no more than 25% of the duty cycle.
		_integrator_value = math::constrain(_integrator_value, -0.25f, 0.25f);

		_controller_time_on_usec = (int)((_p_feed_forward_value.get() + _proportional_value +
						  _integrator_value) * (float)_controller_period_usec);

		// Constrain the heater time within the allowable duty cycle.
		_controller_time_on_usec = math::constrain(_controller_period_usec, 0, _controller_time_on_usec);

		// Filter the duty cycle value over a ~2 second time constant.
		_duty_cycle = (0.05f * ((float)_controller_time_on_usec / (float)_controller_period_usec)) + (0.95f * _duty_cycle);

		// Turn the heater on.
		_heater_on = true;

		px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 1);
	}


	// Schedule the next cycle.
	if (_heater_on) {
		work_queue(LPWORK, &_work, (worker_t)&Heater::cycle_trampoline, this,
			   USEC2TICK(_controller_time_on_usec));

	} else {
		work_queue(LPWORK, &_work, (worker_t)&Heater::cycle_trampoline, this,
			   USEC2TICK(_controller_period_usec - _controller_time_on_usec));
	}
}

void Heater::cycle_trampoline(void *arg)
{
	Heater *obj = reinterpret_cast<Heater *>(arg);

	obj->cycle();
}

float Heater::duty_cycle()
{
	PX4_INFO("Average duty cycle:  %3.1f%%", (double)(_duty_cycle * 100.f));
	return _duty_cycle;
}

float Heater::feed_forward()
{
	return _p_feed_forward_value.get();
}

float Heater::feed_forward(char *argv[])
{
	float feed_forward_value = 0.f;

	if (argv[2]) {
		feed_forward_value = atof(argv[2]);
		feed_forward_value = this->feed_forward(feed_forward_value);

	} else {
		feed_forward_value = this->feed_forward();
	}

	PX4_INFO("Feed Forward Value:  %2.5f", (double)feed_forward_value);
	return feed_forward_value;
}

float Heater::feed_forward(const float feed_forward_value)
{
	_p_feed_forward_value.set(feed_forward_value);
	return _p_feed_forward_value.get();
}

bool Heater::get_state()
{
	return _heater_on;
}

void Heater::initialize_topics()
{
	// Get the total number of accelerometer instances.
	size_t number_of_imus = orb_group_count(ORB_ID(sensor_accel));

	// Check each instance for the correct ID.
	for (size_t x = 0; x < number_of_imus; x++) {
		_sensor_accel_subscription = orb_subscribe_multi(ORB_ID(sensor_accel), (int)x);

		while (orb_update(ORB_ID(sensor_accel), _sensor_accel_subscription, &_sensor_accel) != true) {
			usleep(200000);
		}

		// If the correct ID is found, exit the for-loop with _sensor_accel_subscription pointing to the correct instance.
		if (_sensor_accel.device_id == (uint32_t)_p_sensor_id.get()) {
			PX4_INFO("Found Sensor to Temp Compensate");
			break;
		}
	}

	PX4_INFO("Device ID:  %d", _sensor_accel.device_id);

	// Exit the driver if the sensor ID does not match the desired sensor.
	if (_sensor_accel.device_id != (uint32_t)_p_sensor_id.get()) {
		request_stop();
		PX4_INFO("Could not find sensor to control temperature");
	}
}

void Heater::initialize_trampoline(void *arg)
{
	Heater *heater = new Heater();

	if (!heater) {
		PX4_ERR("Heater driver alloc failed");
		return;
	}

	heater->start();
	_object = heater;
}

float Heater::integrator()
{
	return _p_integrator_gain.get();
}

float Heater::integrator(char *argv[])
{

	float integrator_gain = 0.f;

	if (argv[2]) {
		integrator_gain = atof(argv[2]);
		integrator_gain = this->integrator(integrator_gain);

	} else {
		integrator_gain = this->integrator();
	}

	PX4_INFO("Integrator Gain:  %2.5f", (double)integrator_gain);
	return integrator_gain;
}

float Heater::integrator(const float integrator_gain)
{
	_p_integrator_gain.set(integrator_gain);
	return _p_integrator_gain.get();
}

bool Heater::orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// Check if there is new data to obtain.
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

int Heater::print_status()
{
	float sensor_temperature = this->sensor_temperature();
	float temperature_setpoint = this->temperature_setpoint();
	bool heater_state = this->get_state();

	PX4_INFO("Temp: %3.3f - Target Temp: %3.2f - Heater State: %s",
		 (double)sensor_temperature,
		 (double)temperature_setpoint,
		 heater_state ? "On" : "Off");

	return PX4_OK;
}

int Heater::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to regulate IMU temperature at a setpoint.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("heater", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("controller_period", "Reports the heater driver cycle period value, (us), and sets it if supplied an argument.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("duty_cycle", "Reports the heater duty cycle (%%).");
	PRINT_MODULE_USAGE_COMMAND_DESCR("feed_forward", "Sets the feedforward value if supplied an argument and reports the current value.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("integrator", "Sets the integrator gain value if supplied an argument and reports the current value.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("proportional", "Sets the proportional gain value if supplied an argument and reports the current value.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("sensor_id", "Reports the current IMU the heater is temperature controlling.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("setpoint", "Reports the current IMU temperature.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Starts the IMU heater driver as a background task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Reports the current IMU temperature, temperature setpoint, and heater on/off status.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stops the IMU heater driver.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("temp", "Reports the current IMU temperature.");

	return 0;
}

float Heater::proportional()
{
	return _p_proportional_gain.get();
}

float Heater::proportional(char *argv[])
{
	float proportional_gain = 0.f;

	if (argv[2]) {
		proportional_gain = atof(argv[2]);
		proportional_gain = this->proportional(proportional_gain);

	} else {
		proportional_gain = this->proportional();
	}

	PX4_INFO("Proportional Gain:  %2.5f", (double)proportional_gain);
	return proportional_gain;
}

float Heater::proportional(const float proportional_gain)
{
	_p_proportional_gain.set(proportional_gain);
	return _p_proportional_gain.get();
}

uint32_t Heater::sensor_id()
{
	uint32_t id = this->sensor_id();
	PX4_INFO("Sensor ID:  %d", id);
	return _sensor_accel.device_id;
}

float Heater::sensor_temperature()
{
	PX4_INFO("Current Temp:  %3.3f", (double)_sensor_temperature);
	return _sensor_temperature;
}

int Heater::start()
{
	if (is_running()) {
		PX4_INFO("Heater driver already running");
		return PX4_ERROR;
	}

	update_params(true);
	initialize_topics();

	// Kick off the cycling. We can call it directly because we're already in the work queue context
	cycle();

	PX4_INFO("Heater driver started successfully.");

	return PX4_OK;
}

int Heater::task_spawn(int argc, char *argv[])
{
	int ret = work_queue(LPWORK, &_work, (worker_t)&Heater::initialize_trampoline, nullptr, 0);

	if (ret < 0) {
		return ret;
	}

	ret = wait_until_running();

	if (ret < 0) {
		return ret;
	}

	_task_id = task_id_is_work_queue;
	return 0;
}

float Heater::temperature_setpoint()
{
	return _p_temperature_setpoint.get();
}

float Heater::temperature_setpoint(char *argv[])
{
	float target_temp = 0.f;

	if (argv[2]) {
		target_temp = atof(argv[2]);
		target_temp = this->temperature_setpoint(target_temp);

	} else {

		target_temp = this->temperature_setpoint();
	}

	PX4_INFO("Target Temp:  %3.3f", (double)target_temp);
	return target_temp;
}

float Heater::temperature_setpoint(const float temperature_setpoint)
{
	_p_temperature_setpoint.set(temperature_setpoint);
	return _p_temperature_setpoint.get();
}

void Heater::update_params(const bool force)
{
	bool updated;
	parameter_update_s param_update;

	orb_check(_parameter_subscription, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameter_subscription, &param_update);
	}

	if (updated || force) {
		ModuleParams::updateParams();
		_p_sensor_id.get();
		_p_feed_forward_value.get();
		_p_integrator_gain.get();
		_p_proportional_gain.get();
		_p_temperature_setpoint.get();
	}
}


/**
 * Main entry point for the heater driver module
 */
int heater_main(int argc, char *argv[])
{
	return Heater::main(argc, argv);
}