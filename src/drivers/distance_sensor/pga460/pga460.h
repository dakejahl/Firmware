/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file pga460.h
 * @author Jacob Dahl <jacob.dahl@tealdrones.com>
 *
 * Driver for the TI PGA460 Ultrasonic Signal Processor and Transducer Driver
 */

#ifndef _PGA460_H
#define _PGA460_H

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>

#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/parameter_update.h>

#define PGA460_DEFAULT_PORT "/dev/ttyS6"
#define MAX_DETECTABLE_DISTANCE         5.0f
#define MIN_DETECTABLE_DISTANCE         0.0f
#define MAX_DETECTABLE_TEMPERATURE      100.0f
#define MIN_DETECTABLE_TEMPERATURE     -20.0f

#define SYNCBYTE                        0x55


//      Define UART commands by name

// Single Address
#define P1BL            0x00    // Burst and Listen (Preset 1)
#define P2BL            0x01    // Burst and Listen (Preset 2)
#define P1LO            0x02    // Listen Only (Preset 1)
#define P2LO            0x03    // Listen Only (Preset 2)
#define TNLM            0x04    // Temperature and Noise-level measurement
#define UMR             0x05    // Ultrasonic Measurement Result
#define TNLR            0x06    // Temperature and noise level result
#define TEDD            0x07    // Transducer echo data dump
#define SD              0x08    // System diagnostics
#define SRR             0x09    // Register read
#define SRW             0x0A    // Register write
#define EEBR            0x0B    // EEPROM bulk read
#define EEBW            0x0C    // EEPROM bulk write
#define TVGBR           0x0D    // Time-varying-gain bulk read
#define TVGBW           0x0E    // Time-varying-gain bulk write
#define THRBR           0x0F    // Threshold bulk read
#define THRBW           0x10    // Threshold bulk write

// Broadcast -- device will execute command irrespecive of UART address field
#define BC_P1BL         0x11    // Burst and listen (Preset 1)
#define BC_P2BL         0x12    // Burst and listen (Preset 2)
#define BC_P1LO         0x13    // Listen only (Preset 1)
#define BC_P2LO         0x14    // Listen only (Preset 2)
#define BC_TNLM         0x15    // Temperature and noise-level measurement
#define BC_SRW          0x16    // Register write
#define BC_EEBW         0x17    // EEPROM bulk write
#define BC_TVGBW        0x18    // Time varying-gain bulk write
#define BC_THRBW        0x19    // Threshold bulk write

// Addresses and Settings
#define EE_CNTRL_ADDR   0x40
#define EE_UNLOCK_ST1   0x68
#define EE_UNLOCK_ST2   0x69

//      REGISTER MAP

#define USER_DATA1      0x0     //reg addr      0x0
#define USER_DATA2      0x0     //reg addr      0x1
#define USER_DATA3      0x0     //reg addr      0x2
#define USER_DATA4      0x0     //reg addr      0x3
#define USER_DATA5      0x0     //reg addr      0x4
#define USER_DATA6      0x0     //reg addr      0x5
#define USER_DATA7      0x0     //reg addr      0x6
#define USER_DATA8      0x0     //reg addr      0x7
#define USER_DATA9      0x0     //reg addr      0x8
#define USER_DATA10     0x0     //reg addr      0x9
#define USER_DATA11     0x0     //reg addr      0x0A
#define USER_DATA12     0x0     //reg addr      0x0B
#define USER_DATA13     0x0     //reg addr      0x0C
#define USER_DATA14     0x0     //reg addr      0x0D
#define USER_DATA15     0x0     //reg addr      0x0E
#define USER_DATA16     0x0     //reg addr      0x0F
#define USER_DATA17     0x0     //reg addr      0x10
#define USER_DATA18     0x0     //reg addr      0x11
#define USER_DATA19     0x0     //reg addr      0x12
#define USER_DATA20     0x0     //reg addr      0x13
#define TVGAIN0         0x9F    //reg addr      0x14
#define TVGAIN1         0xEF    //reg addr      0x15
#define TVGAIN2         0xFF    //reg addr      0x16
#define TVGAIN3         0xA5    //reg addr      0x17
#define TVGAIN4         0x6D    //reg addr      0x18
#define TVGAIN5         0xBE    //reg addr      0x19
#define TVGAIN6         0xFC    //reg addr      0x1A
#define INIT_GAIN       0x40    //reg addr      0x1B
#define FREQUENCY       (uint8_t)(5*(_params.resonant_frequency - 30.0f))       //reg addr      0x1C
#define DEADTIME        0xF0    //reg addr      0x1D
#define PULSE_P1        0x0C    //reg addr      0x1E
#define PULSE_P2        0x0C    //reg addr      0x1F
#define CURR_LIM_P1     0x3F    //reg addr      0x20
#define CURR_LIM_P2     0x5E    //reg addr      0x21
#define REC_LENGTH      0xE8    //reg addr      0x22
#define FREQ_DIAG       0x1B    //reg addr      0x23
#define SAT_FDIAG_TH    0x2C    //reg addr      0x24
#define FVOLT_DEC       0x7C    //reg addr      0x25
#define DECPL_TEMP      0xDF    //reg addr      0x26
#define DSP_SCALE       0x0     //reg addr      0x27
#define TEMP_TRIM       0x0     //reg addr      0x28
#define P1_GAIN_CTRL    0x0     //reg addr      0x29
#define P2_GAIN_CTRL    0x0     //reg addr      0x2A
#define EE_CRC          0x9     //reg addr      0x2B
#define EE_CNTRL        0x0     //reg addr      0x40
#define BPF_A2_MSB      0x85    //reg addr      0x41
#define BPF_A2_LSB      0xEA    //reg addr      0x42
#define BPF_A3_MSB      0xF9    //reg addr      0x43
#define BPF_A3_LSB      0xA5    //reg addr      0x44
#define BPF_B1_MSB      0x3     //reg addr      0x45
#define BPF_B1_LSB      0x2D    //reg addr      0x46
#define LPF_A2_MSB      0x7E    //reg addr      0x47
#define LPF_A2_LSB      0x67    //reg addr      0x48
#define LPF_B1_MSB      0x0     //reg addr      0x49
#define LPF_B1_LSB      0xCD    //reg addr      0x4A
#define TEST_MUX        0x0     //reg addr      0x4B
#define DEV_STAT0       0x80    //reg addr      0x4C
#define DEV_STAT1       0x0     //reg addr      0x4D
#define P1_THR_0        0x64    //reg addr      0x5F
#define P1_THR_1        0x8C    //reg addr      0x60
#define P1_THR_2        0xEE    //reg addr      0x61
#define P1_THR_3        0xEC    //reg addr      0x62
#define P1_THR_4        0xDF    //reg addr      0x63
#define P1_THR_5        0xCF    //reg addr      0x64
#define P1_THR_6        0xF6    //reg addr      0x65
#define P1_THR_7        0x8E    //reg addr      0x66
#define P1_THR_8        0x74    //reg addr      0x67
#define P1_THR_9        0x25    //reg addr      0x68
#define P1_THR_10       0x6C    //reg addr      0x69
#define P1_THR_11       0x64    //reg addr      0x6A
#define P1_THR_12       0x64    //reg addr      0x6B
#define P1_THR_13       0x64    //reg addr      0x6C
#define P1_THR_14       0x64    //reg addr      0x6D
#define P1_THR_15       0x0     //reg addr      0x6E
#define P2_THR_0        0x30    //reg addr      0x6F
#define P2_THR_1        0xE8    //reg addr      0x70
#define P2_THR_2        0x32    //reg addr      0x71
#define P2_THR_3        0xDD    //reg addr      0x72
#define P2_THR_4        0x68    //reg addr      0x73
#define P2_THR_5        0x5F    //reg addr      0x74
#define P2_THR_6        0x1     //reg addr      0x75
#define P2_THR_7        0x7E    //reg addr      0x76
#define P2_THR_8        0x3A    //reg addr      0x77
#define P2_THR_9        0xF7    //reg addr      0x78
#define P2_THR_10       0x45    //reg addr      0x79
#define P2_THR_11       0x82    //reg addr      0x7A
#define P2_THR_12       0x48    //reg addr      0x7B
#define P2_THR_13       0xEE    //reg addr      0x7C
#define P2_THR_14       0x4     //reg addr      0x7D
#define P2_THR_15       0xAC    //reg addr      0x7E
#define THR_CRC         0x16    //reg addr      0x7F


bool    start();
bool    stop();
void    info();

class PGA460 : public device::CDev
{
public:

	PGA460(const char *port = PGA460_DEFAULT_PORT);

	virtual ~PGA460();

	virtual int init();

	/**
	 * @brief Starts the pga460 driver
	 * @return Returns true if the open was successful.
	 */
	int start();

	/**
	 * @brief Stops the pga460 driver
	 * @return Returns OK when it completes.
	 */
	int stop();

	/**
	 * @brief Opens the serial port.
	 * @return Returns true if the open was successful or ERRNO.
	 */
	int open_serial();

	/**
	 * @brief Closes the serial port.
	 * @return Returns 0 if success or ERRNO.
	 */
	int close_serial();

	/**
	 * @brief Reads the threshold registers
	 * @return Returns true if the threshold registers are set to default
	 */
	bool read_threshold_registers();

	/**
	 * @brief Writes the user defined paramaters to device EEPROM.
	 * @return Returns true if the EEPROM was successfully written to.
	 */
	bool write_eeprom();

	/**
	 * @brief Reads the EEPROM and checks to see if it matches the default parameters
	 * @return Returns true if the EEPROM has default values.
	 */
	bool check_eeprom();

	/**
	 * @brief Writes a value to a register.
	 * @return Returns true for success or false for fail.
	 */
	bool write_register(const uint8_t reg, const uint8_t val);

	/**
	 * @brief Reads a register.
	 * @return Returns the value of the register at the specified address.
	 */
	uint8_t read_register(const uint8_t reg);

	/*
	 * @brief Get the diagnostic byte from the most recent pga460 response.
	 * @return Returns the diagnostic byte.
	 */
	uint8_t get_diagnostic_byte();

	/**
	 * @brief Reports the diagnostic data the diagnostic byte (first byte from slave).
	 */
	void print_diagnostics(const uint8_t diagnostic_byte);

	/**
	 * @brief Reports the diagnostic data from device status registers 1 and 2 if there is anything to report.
	 */
	void print_device_status();

	/**
	 * @brief Fills a u16 with system diagnostics bytes. Byte1 = Transducer frequency    Byte2 = Decay period time
	 * @return Returns a u16 that holds the two diagnostic bytes.
	 */
	uint16_t get_system_diagnostics();

	/**
	 * @brief Sweeps from 30 - 50 kHz and returns the frequeny with the largest receieved amplitude (the resonant frequency).
	 * @return Returns the value of the register to set the frequency according to equation: value*0.2 + 30 = frequency kHz
	 */
	uint8_t find_resonant_frequency();

	/**
	 * @brief Gets the minimum distance.
	 * @return Returns the minimum distance.
	 */
	float get_minimum_distance();

	/**
	 * @brief Sets the minimum distance.
	 */
	void set_minimum_distance(const float dist);

	/**
	 * @brief Gets the maximum distance.
	 * @return Returns the maximum distance.
	 */
	float get_maximum_distance();

	/**
	 * @brief Sets the maximum distance.
	 */
	void set_maximum_distance(const float dist);

	/**
	 * @brief Suspends the measurement cycle but does not shut down the driver.
	 */
	void suspend();

	/**
	 * @brief Resumes the measurement cycle.
	 */
	void resume();



private:
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	* @brief Main loop of the driver. Loops until _task_should_exit flag is set.
	*/
	void task_main();

	/**
	 * @brief Writes program defined threshold defaults to the register map and checks/writes the EEPROM.
	 */
	void init_pga460();

	/**
	 * @brief Commands the device to perform an ultrasonic measurement.
	 */
	void take_measurement();

	/**
	 * @brief Measurement is read from UART RX buffer and published to the uORB distance sensor topic.
	 */
	void request_results();

	/**
	 * @brief Collects the data in the serial port rx buffer, does math, and publishes the value to uORB
	 * @return Returns the measurment results in format: (u16)time_of_flight, (u8)width, (u8)amplitude
	 */
	uint32_t collect_results();

	/**
	 * @brief Calculates the distance from the measurement time of flight (time_of_flight) and current temperature.
	 * @return Returns the distance measurement in meters.
	 */
	float calculate_object_distance(uint16_t time_of_flight);

	/*
	 * @brief Issues a temperature measurement.
	 * @return Returns the temperature in degrees C.
	 */
	float get_temperature();

	/**
	 * @brief Commands the device to publish the measurement results to uORB.
	 */
	void uORB_publish_results(const float &dist, const float &temp);

	/**
	 * @brief Send the unlock command to the EEPROM to enable reading and writing -- not needed w/ bulk write
	 */
	void unlock_eeprom();

	/**
	 * @brief Send the program command to the EEPROM to start the flash process.
	 */
	void flash_eeprom();

	/**
	 * @brief Writes the user defined paramaters to device register map.
	 * @return Returns true if the thresholds were successfully written.
	 */
	bool init_thresholds();

	/**
	 * @brief Calculates the checksum of the transmitted commmand + data
	 * @return Returns the single byte checksum.
	 */
	uint8_t calc_checksum(uint8_t *data, const uint8_t size);

	/** @param _task_handle Handle for the task.  */
	px4_task_t _task_handle;

	/** @param _task_is_running Indicator flag for when the driver is running. */
	volatile bool _task_is_running;

	/** @param _task_should_exit Indicator flag to stop the ultrasonic measurement process. */
	volatile bool _task_should_exit;

	/** @param _transducer_freq Value of the calibrated (from factory) transducer frequency. */
	uint8_t _transducer_freq;

	/** @param _diagnostic_byte Holds the diagnostic byte from the most recent pga460 response. */
	uint8_t _diagnostic_byte;

	/** @param _class_instance Instance value returned from registering the class of device type. */
	int _class_instance;

	/** @param _orb_class_instance This is an output parameter and will be set to the newly created instance. */
	int _orb_class_instance;

	/** @param _fd Returns the file descriptor from px4_open(). */
	int _fd;

	/** @param _min_distance Returns the set minimum distance. */
	float _min_distance;

	/** @param _max_distance Returns the set maximum distance. */
	float _max_distance;

	float _previous_measurement;

	/** @param _port Stores the port name. */
	char _port[20];

	/** @orb_advert_t orb_advert_t uORB advertisement topic. */
	orb_advert_t _distance_sensor_topic;

	/**
	* @brief Handles for interesting parameters
	**/
	struct {
		param_t resonant_frequency;

	} _paramHandle{};


	struct {
		float resonant_frequency;

	} _params{};
};

#endif
