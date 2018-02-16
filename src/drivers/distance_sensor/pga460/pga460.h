/**
 * @file pga460.cpp
 * @author Khoi Tran <khoi@tealdrones.com>
 *
 * Driver for the TI PGA460 Ultrasonic Signal Processor and Transducer Driver
 */

#ifndef _PGA460_H
#define _PGA460_H

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <uORB/topics/distance_sensor.h>

#define PGA460_DEFAULT_PORT "/dev/ttyS6"	// frsky_telemetry on fmuv4
#define BUF_LEN 9
#define SPEED_OF_SOUND 343.2f
#define SYNCBYTE 0x55

// Define UART commands by name

// Single Address
#define P1BL 0x00	// Burst and Listen (Preset 1)
#define P2BL 0x01	// Burst and Listen (Preset 2)
#define P1LO 0x02	// Listen Only (Preset 1)
#define P2LO 0x03	// Listen Only (Preset 2)
#define TNLM 0x04	// Temperature and Noise-level measurement
#define UMR 0x05	// Ultrasonic Measurement Result
#define TNLR 0x06	// Temperature and noise level result
#define TEDD 0x07	// Transducer echo data dump
#define SD 0x08		// System diagnostics
#define SRR 0x09	// Register read
#define SRW 0x0A	// Register write
#define EEBR 0x0B	// EEPROM bulk read
#define EEBW 0x0C	// EEPROM bulk write
#define TVGBR 0x0D	// Time-varying-gain bulk read
#define TVGBW 0x0E	// Time-varying-gain bulk write
#define THRBR 0x0F	// Threshold bulk read
#define THRBW 0x10	// Threshold bulk write
//Broadcast
#define BC_P1BL 0x11	// Burst and listen (Preset 1)
#define BC_P2BL 0x12	// Burst and listen (Preset 2)
#define BC_P1LO 0x13	// Listen only (Preset 1)
#define BC_P2LO 0x14	// Listen only (Preset 2)
#define BC_TNLM 0x15	// Temperature and noise-level measurement
#define BC_RW 0x16		// Register write
#define BC_EEBW 0x17	// EEPROM bulk write
#define BC_TVGBW 0x18	// Time varying-gain bulk write
#define BC_THRBW 0x19	// Threshold bulk write

//Register Values

/*
	Time-varying Gain register values (TVG_*)

	Table for TVG_T* registers
	0x0 = 100us
	0x1 = 200us
	0x2 = 300us
	0x3 = 400us
	0x4 = 600us
	0x5 = 800us
	0x6 = 1000us
	0x7 = 1200us
	0x8 = 1400us
	0x9 = 2000us
	0xA = 2400us
	0xB = 3200us
	0xC = 4000us
	0xD = 5200us
	0xE = 6400us
	0xF = 8000us
*/
#define TVG_T0 0xE
#define TVG_T1 0xE
#define TVG_T2 0xE
#define TVG_T3 0xE
#define TVG_T4 0xE
#define TVG_T5 0xE

// Equation for TVG_G* registers
// Gain = 0.5*(TVG_G+1)+value(AFE_GAIN_RNG) [dB]
#define TVG_G1 0x05
#define TVG_G2 0x0F
#define TVG_G3 0x1F
#define TVG_G4 0x2B
#define TVG_G5 0x3B

// Frequency range shift
#define FREQ_SHIFT 0b0	// 0b0 = disabled, 0b1 = enabled. frequency = 6*result from FREQ register

/*
	Initial Gain register values (INIT_GAIN)
*/
#define BPF_BW 0x0		// Bandwidth = 2*(BPF_BW+1) [kHz]
#define GAIN_INIT 0x00	// Init_gain = 0.5*(GAIN_INIT+1)+value(AFE_GAIN_RNG) [dB]

/*
	Frequency register (FREQUENCY)
*/
#define FREQ 0x32	// Frequency = 0.2*FREQ + 30 [kHz]

/*
	Deadtime register (DEADTIME)
*/
#define THR_CMP_DEGLTCH 0x0
#define PULSE_DT 0x0

/*
	Pulse register values (PULSE_*)
*/
#define IO_IF_SEL 0b0	// 0b0 = time-based interface, 0b1 = one-wire UART
#define UART_DIAG 0b0	// 0b0 = diagnostic bit related to UART interface, 0b1 = diagnostic bit related to system diagnostics
#define IO_DIS 0b0		// enabled/diabled IO pin
#define P1_PULSE 0x10	// number of pulses for preset 1
#define UART_ADDR 0x0
#define P2_PULSE 0x10	// number of pulses for preset 2

/*
	Current Limit Register (CUR_LIM_P*)
*/
#define DIS_CL 0b0		// enabled/disable current limit for P1 and P2
#define CURR_LIM1 0x1F	// current_limit = 7*CURR_LIM1+50 [mA]
#define LPF_CO 0x0		// cut off frequency = LPF_CO + 1
#define CURR_LIM2 0x1f	// current limit = 7*CURR_LIM2+50 [mA]

/*
	Data Record Length Register (REC_LENGTH)
*/
#define P1_REC 0x9	// Record time = 4.096*(P1_REC+1) [ms]
#define P2_REC 0x9	// Record time = 4.096*(P2_REC+1) [ms]

/*
	Frequency Diagnostics Register (FREQ_DIAG)
*/
#define FDIAG_LEN 0x0	// Window Length = 3*FDIAG_LEN
#define FDIAG_START 0x0	// Diagnostic start time. start time = 100*FDIAG_START [us]

/*
	Decay Saturtion, frequency diagnostic, preset 1 non-linear enable control register (SAT_FDIAG_TH)
*/
#define FDIAG_ERR_TH 0x0	// threshold = (FDIAG_ERR_TH+1) [us]
#define SAT_TH 0x0			// Saturation diagnostic threshold level
#define P1_NLS_EN 0x0		// Set high to enable preset 1 non-linear scaling

/*
	Voltage threshold and Preset 2 non-linear scaling enable (FVTOL_DEV)
*/
#define P2_NLS_EN 0b0		// Set high to enable Preset 2 non-linear scaling
#define VPWR_OV_TH 0x2		// VPR over voltage threshold select: 0b00 = 12.3, 0b01 = 17.7, 0b10 = 22.8, 0b11 = 28.3
#define LPM_TMR 0x2 		// Low power mode enter time. 0b00 = 250ms, 0b01 = 500ms, 0b10 = 1s, 0b11 = 4s
#define FVOLT_ERR_THR 0x0

/*
	Decouple temperature and AFE gain register (DECPL_TEMP)
*/
#define AFE_GAIN_RNG 0b11	// 0b00 = 58-90dB, 0b01 = 52-84dB, 0b10 = 46-78dB, 0b11 = 32-64dB
#define LPM_EN 0b0			// Low power mode enable. 0 = disable, 1 = enable 
#define DECPL_TEMP_SEL 0b0	// Select time or temperature decouple. 0 = time decouple, 1 = temperature decouple
#define DECPL_T 0x0			// If time decouple, time = 4096 * (DECPL_T+1) [us]. If temp decouple, temp = 10 * DECPL_T - 40

/*
	DPS non-linear scaling and noise Register (DSP_SCALE)
*/
#define NOISE_LVL 0x01	// Values range from 0 to 31 to LSB steps for digital gain values. If digital is less than 8, multiply the noise level by Px_DIG_GAIN_LR/8
#define SCALE_K 0b0		// Non-linear scaling exponent selection: 0 = 1.50, 1 = 2.00
#define SCALE_N 0b00	// Select the threshold level point. 0b00 = TH9, 0b01 = TH10, 0b10 = TH11, 0b11 = TH12

/*
	Temperature sensor compensation Register (TEMP_TRIM)
*/
#define TEMP_GAIN 0x0
#define TEMP_OFF 0x0

/*
	Gain control Registers (P*_GAIN_CTRL)
*/
#define P1_DIG_GAIN_LR_ST 0x0	// Select the starting Preset 1 from which the LR digital gain starts. 0b00 = TH9, 0b01 = TH10, 0b10 = TH11, 0b11 = TH12
#define P1_DIG_GAIN_LR 0x1
#define P1_DIG_GAIN_SR 0x0

#define P2_DIG_GAIN_LR_ST 0x0	// Select the starting Preset 2 from which the LR digital gain starts. 0b00 = TH9, 0b01 = TH10, 0b10 = TH11, 0b11 = TH12
#define P2_DIG_GAIN_LR 0x1
#define P2_DIG_GAIN_SR 0x0

/*
	Threshold Registers (P*_THR_*)
*/
#define TH_P1_T1 0b1000
#define TH_P1_T2 0b0100
#define TH_P1_T3 0b1100
#define TH_P1_T4 0b1100
#define TH_P1_T5 0b1100
#define TH_P1_T6 0b1011
#define TH_P1_T7 0b1011
#define TH_P1_T8 0b1011
#define TH_P1_T9 0b1011
#define TH_P1_T10 0b1011
#define TH_P1_T11 0b1011
#define TH_P1_T12 0b1011

#define TH_P1_L1 23
#define TH_P1_L2 10
#define TH_P1_L3 10
#define TH_P1_L4 10
#define TH_P1_L5 10
#define TH_P1_L6 10
#define TH_P1_L7 10
#define TH_P1_L8 10
#define TH_P1_L9 10
#define TH_P1_L10 80
#define TH_P1_L11 80
#define TH_P1_L12 80
#define TH_P1_OFF 0x0

#define TH_P2_T1 0x2
#define TH_P2_T2 0x2
#define TH_P2_T3 0x2
#define TH_P2_T4 0x2
#define TH_P2_T5 0x2
#define TH_P2_T6 0x2
#define TH_P2_T7 0x2
#define TH_P2_T8 0x2
#define TH_P2_T9 0x2
#define TH_P2_T10 0x2
#define TH_P2_T11 0x2
#define TH_P2_T12 0x2

#define TH_P2_L1 0x2
#define TH_P2_L2 0x2
#define TH_P2_L3 0x2
#define TH_P2_L4 0x2
#define TH_P2_L5 0x2
#define TH_P2_L6 0x2
#define TH_P2_L7 0x2
#define TH_P2_L8 0x2
#define TH_P2_L9 0x2
#define TH_P2_L10 0x2
#define TH_P2_L11 0x2
#define TH_P2_L12 0x2
#define TH_P2_OFF 0x0

/*
	Registers
*/
#define USER_DATA0 0x0
#define USER_DATA1 0x0
#define USER_DATA2 0x0
#define USER_DATA3 0x0
#define USER_DATA4 0x0
#define USER_DATA5 0x0
#define USER_DATA6 0x0
#define USER_DATA7 0x0
#define USER_DATA8 0x0
#define USER_DATA9 0x0
#define USER_DATA10 0x0
#define USER_DATA11 0x0
#define USER_DATA12 0x0
#define USER_DATA13 0x0
#define USER_DATA14 0x0
#define USER_DATA15 0x0
#define USER_DATA16 0x0
#define USER_DATA17 0x0
#define USER_DATA18 0x0
#define USER_DATA19 0x0
#define USER_DATA20 0x0
#define TVGAIN0 TVG_T0 << 4 | TVG_T1
#define TVGAIN1 TVG_T2 << 4 | TVG_T3
#define TVGAIN2 TVG_T4 << 4 | TVG_T5
#define TVGAIN3 TVG_G1 << 2 | TVG_G2 >> 4
#define TVGAIN4 (TVG_G2 & 0xF) << 4 | TVG_G3 >> 2
#define TVGAIN5 (TVG_G3 & 0x3) << 6 | TVG_G4
#define TVGAIN6 TVG_G5 << 2 | FREQ_SHIFT
#define INIT_GAIN BPF_BW << 6 | GAIN_INIT
#define FREQUENCY FREQ
#define DEADTIME THR_CMP_DEGLTCH << 4 | PULSE_DT
#define PULSE_P1 IO_IF_SEL << 7 | UART_DIAG << 6 | IO_DIS << 5 | P1_PULSE
#define PULSE_P2 UART_ADDR << 5 | P2_PULSE
#define CURR_LIM_P1 DIS_CL << 7 | CURR_LIM1
#define CURR_LIM_P2 LPF_CO << 6 | CURR_LIM2
#define REC_LENGTH P1_REC << 4 | P2_REC
#define FREQ_DIAG FDIAG_LEN << 4 | FDIAG_START
#define SAT_FDIAG_TH FDIAG_ERR_TH << 5 | SAT_TH << 1 | P1_NLS_EN
#define FVOLT_DEC P2_NLS_EN << 7 | VPWR_OV_TH << 5 | LPM_TMR << 3 | FVOLT_ERR_THR
#define DECPL_TEMP AFE_GAIN_RNG << 6 | LPM_EN << 5 | DECPL_TEMP_SEL << 4 | DECPL_T
#define DSP_SCALE NOISE_LVL << 3 | SCALE_K << 2 | SCALE_N
#define TEMP_TRIM TEMP_GAIN << 4 | TEMP_OFF
#define P1_GAIN_CTRL P1_DIG_GAIN_LR_ST << 6 | P1_DIG_GAIN_LR << 3 | P1_DIG_GAIN_SR
#define P2_GAIN_CTRL P2_DIG_GAIN_LR_ST << 6 | P2_DIG_GAIN_LR << 3 | P2_DIG_GAIN_SR
#define P1_THR_0 TH_P1_T1 << 4 | TH_P1_T2
#define P1_THR_1 TH_P1_T3 << 4 | TH_P1_T4
#define P1_THR_2 TH_P1_T5 << 4 | TH_P1_T6
#define P1_THR_3 TH_P1_T7 << 4 | TH_P1_T8
#define P1_THR_4 TH_P1_T9 << 4 | TH_P1_T10
#define P1_THR_5 TH_P1_T11 << 4 | TH_P1_T12
#define P1_THR_6 (TH_P1_L1 & 0x1F) << 3 | TH_P1_L2 >> 2
#define P1_THR_7 (TH_P1_L2 & 0x3) << 6 | TH_P1_L3 << 1 | TH_P1_L4 >> 4
#define P1_THR_8 (TH_P1_L4 & 0xF) << 4 | TH_P1_L5 >> 1
#define P1_THR_9 (TH_P1_L5 & 0x1) << 7 | TH_P1_L6 << 2 | TH_P1_L7 >> 3
#define P1_THR_10 (TH_P1_L7 & 0x7) << 5 | TH_P1_L8
#define P1_THR_11 TH_P1_L9
#define P1_THR_12 TH_P1_L10
#define P1_THR_13 TH_P1_L11
#define P1_THR_14 TH_P1_L12
#define P1_THR_15 TH_P1_OFF
#define P2_THR_0 TH_P2_T1 << 4 | TH_P2_T2
#define P2_THR_1 TH_P2_T3 << 4 | TH_P2_T4
#define P2_THR_2 TH_P2_T5 << 4 | TH_P2_T6
#define P2_THR_3 TH_P2_T7 << 4 | TH_P2_T8
#define P2_THR_4 TH_P2_T9 << 4 | TH_P2_T10
#define P2_THR_5 TH_P2_T11 << 4 | TH_P2_T12
#define P2_THR_6 (TH_P2_L1 & 0x1F) << 3 | TH_P2_L2 >> 2
#define P2_THR_7 (TH_P2_L2 & 0x3) << 6 | TH_P2_L3 << 1 | TH_P2_L4 >> 4
#define P2_THR_8 (TH_P2_L4 & 0xF) << 4 | TH_P2_L5 >> 1
#define P2_THR_9 (TH_P2_L5 & 0x1) << 7 | TH_P2_L6 << 2 | TH_P2_L7 >> 3
#define P2_THR_10 (TH_P2_L7 & 0x7) << 5 | TH_P2_L8
#define P2_THR_11 TH_P2_L9
#define P2_THR_12 TH_P2_L10
#define P2_THR_13 TH_P2_L11
#define P2_THR_14 TH_P2_L12
#define P2_THR_15 TH_P2_OFF

class PGA460 : public device::CDev
{
public:
	PGA460(const char *port = PGA460_DEFAULT_PORT);
	virtual ~PGA460();

	virtual int 	init();

	int				start();

private:
	bool			_task_should_exit;
	int 			_task_handle;
	int				_class_instance;
	int				_orb_class_instance;
	orb_advert_t	_distance_sensor_topic;
	int				_fd;
	unsigned		_baudrate;
	char 			_port[20];
	uint8_t 		_buf[BUF_LEN];

	uint8_t			_checksum = 0x00; // UART checksum value
	uint8_t			_regAddr = 0x00; // data uint8_t for Register Address
	uint8_t			_regData = 0x00; // data uint8_t for Register Data
	uint8_t			_checksum_input[44];

	static void task_main_trampoline(int argc, char *argv[]);
	void task_main();

	void init_pga460();
	void init_thresholds();
	void init_eeprom();
	void ultrasonicCmd();
	bool pullUltrasonicMeasResult();

	uint8_t calcChecksum(uint8_t cmd);

	bool read_and_parse(uint8_t *buf, int len, float *range);
};

#endif
