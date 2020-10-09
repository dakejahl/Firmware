## Getting Started
#### Tools
- Ubuntu 18.04 or later
- ST Link V2 (of any variety -- I am using the cheap Amazon one that ships with the STM32 Blue Pill)
- PX4 Development environment setup https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html
---
#### Building and flashing the bootloader
- Download the PX4 bootloader repository https://github.com/PX4/Bootloader
- Build the bootloader for the pixracer

```
git submodule sync --recursive
git submodule update --init --recursive
make px4fmuv4_bl
```

- Connect debug pins to ST-Link V2 (PWR/GND/SWDIO/SWDCLK)
- Use `st-flash` to flash the bootloader at the beginning of FLASH
```
st-flash erase
st-flash write build/px4fmuv4_bl/px4fmuv4_bl.bin 0x8000000
```
---
#### Building and flashing PX4
- Download the PX4 Firmware repository https://github.com/PX4/Firmware
If you ran the magic shell scripts from the PX4 dev environment setup, PX4 should already be downloaded in the directory in which you ran the scipt. If not, run:
```
mkdir px4 && cd px4/
git clone git@github.com:PX4/Firmware.git
cd Firmware/
git submodule update --init --recursive
```
Add my repository as a remote and checkout the ANTs development branch
```
git remote add jake git@github.com:dakejahl/Firmware.git
git fetch jake
git checkout dev/ants_imu
git submodule update --init --recursive
```
- Disconnect the ST Link and Connect the FRONT connector to the board and connect to PC with USB cable.
- To build and flash the firmware, run
```
make ants_fmu-v4_default upload
```
---
#### Debugging and interacting with PX4
- Connect RX/TX from UART on debug header to FTDI cable connected to host PC. Open up your favorite terminal emulator, eg:
```
picocom /dev/ttyUSB0 -b 57600
```
- Unplug/replug the USB to power cycle the board. You should see this:
```
[boot] Fault Log info File No 4 Length 3177 flags:0x01 state:1
[boot] Fault Log is Armed
sercon: Registering CDC/ACM serial driver
sercon: Successfully registered the CDC/ACM serial driver
HW arch: ANTS_FMU_V4
FW git-hash: 314da4f60e372d84c776c8415475d1857be9fe1e
FW version: 1.11.0 c0 (17498304)
FW git-branch: dev/ants_imu
OS: NuttX
OS version: Release 8.2.0 (134349055)
OS git-hash: a96d05c286fc1c619111f220e500b14e18c0509c
Build datetime: Aug 31 2020 12:11:20
Build uri: localhost
Toolchain: GNU GCC, 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
PX4GUID: 000100000000303433323438511500400017
MCU: STM32F42x, rev. 3
[hardfault_log] Fault Log is Armed
INFO  [param] selected parameter default file /fs/mtd_params
INFO  [tune_control] Publishing standard tune 1
Board defaults: /etc/init.d/rc.board_defaults
INFO  [dataman] Unknown restart, data manager file '/fs/microsd/dataman' size is 362560 bytes
Board sensors: /etc/init.d/rc.board_sensors
reset done, 50 ms
icm20602 #0 on SPI bus 1 (devid=0x38)
icm20948 #0 on SPI bus 1 (devid=0x28)
dps310 #0 on SPI bus 2 (devid=0x68)
WARN  1619632    Failed to flush lifetime data -- is lifetime data collection enabled? (file ../../src/drivers/bq4050/bq4050.cpp line 372)
Board extras: /etc/init.d/rc.board_mavlink
INFO  [mavlink] mode: Config, data rate: 800000 B/s on /dev/ttyACM0 @ 57600B
INFO  [mavlink] mode: Normal, data rate: 20000 B/s on /dev/ttyS0 @ 921600B
WARN  [mavlink] hardware flow control not supported
Starting Main GPS on /dev/ttyS3
Starting MAVLink on /dev/ttyS1
INFO  [mavlink] mode: Normal, data rate: 1200 B/s on /dev/ttyS1 @ 57600B
No autostart ID found
Board extras: /etc/init.d/rc.board_extras
INFO  [logger] logger started (mode=all)

NuttShell (NSH)
nsh>
```
---
#### Useful commands
The nsh console provides useful commands to view information about the system. In general, all software modules can be invoked by simply calling their name. To see the list of available modules use `help`.
```
nsh> help
```
PX4 uses NuttX tasks as well as a work queue for scheduling modules/drivers. To see the status of drivers/modules running on the work queue:
```
nsh> work_queue status

Work Queue: 7 threads                        RATE        INTERVAL
|__ 1) wq:rate_ctrl
|   \__ 1) vehicle_angular_velocity      813.1 Hz         1230 us
|__ 2) wq:SPI1
|   |__ 1) icm20602                      813.0 Hz         1230 us
|   \__ 2) icm20948                      749.9 Hz         1334 us (1333 us)
|__ 3) wq:SPI2
|   \__ 1) dps310                         64.0 Hz        15625 us (15625 us)
|__ 4) wq:nav_and_controllers
|   |__ 1) ekf2                          203.3 Hz         4919 us
|   |__ 2) sensors                       203.3 Hz         4920 us
|   |__ 3) vehicle_acceleration          205.0 Hz         4879 us
|   |__ 4) vehicle_air_data               21.3 Hz        46933 us
|   |__ 5) vehicle_imu                   187.5 Hz         5332 us
|   \__ 6) vehicle_imu                   203.3 Hz         4919 us
|__ 5) wq:hp_default
|   |__ 1) adc                           100.0 Hz        10000 us (10000 us)
|   |__ 2) rc_update                       0.0 Hz            0 us
|   |__ 3) safety_button                  30.3 Hz        32999 us (33000 us)
|   \__ 4) tone_alarm                     10.0 Hz        99813 us
|__ 6) wq:UART4
|   \__ 1) rc_input                      250.0 Hz         4000 us (4000 us)
\__ 7) wq:lp_default
    |__ 1) load_mon                        2.0 Hz       499449 us (500000 us)
    \__ 2) send_event                     30.0 Hz        33332 us (33333 us)
```

You can also view the process list with `top`
```
 PID COMMAND                   CPU(ms) CPU(%)  USED/STACK PRIO(BASE) STATE FD
   0 Idle Task                  284395 57.611   276/  512   0 (  0)  READY  3
   1 hpwork                          0  0.000   344/ 1260 249 (249)  w:sig  3
   2 lpwork                          1  0.000   944/ 1612  50 ( 50)  w:sig  3
   3 init                         1558  0.000  1824/ 2924 100 (100)  w:sem  3
   4 wq:manager                      1  0.000   384/ 1252 255 (255)  w:sem  4
 325 mavlink_rcv_if0              1948  0.398  2744/ 4068 175 (175)  w:sem  4
  16 wq:hp_default                3455  0.696   844/ 1900 240 (240)  w:sem  4
  18 dataman                        20  0.000   760/ 1204  90 ( 90)  w:sem  4
  20 wq:lp_default                 277  0.099   796/ 1700 205 (205)  w:sem  4
  40 wq:SPI1                     62854 12.736  1560/ 2332 253 (253)  w:sem  4
  43 wq:SPI2                      1203  0.199   828/ 2332 252 (252)  w:sem  4
  91 wq:nav_and_controllers      32596  6.666  2072/ 7196 241 (241)  w:sem  4
  92 wq:rate_ctrl                13253  2.686  1136/ 1660 255 (255)  w:sem  4
 103 commander                    6584  1.293  1320/ 3212 140 (140)  w:sig  6
 105 commander_low_prio             11  0.000   872/ 2996  50 ( 50)  w:sem  6
 113 mavlink_if0                 33364  6.766  1920/ 2572 100 (100)  w:sig  4
 118 mavlink_if1                 22842  4.676  1920/ 2540 100 (100)  w:sig  4
 119 mavlink_rcv_if1              1795  0.298  2792/ 4068 175 (175)  w:sem  4
 173 gps                           363  0.000  1008/ 1676 205 (205)  w:sem  4
 216 mavlink_if2                  7414  1.492  1840/ 2484 100 (100)  w:sig  4
 217 mavlink_rcv_if2              1830  0.398  2792/ 4068 175 (175)  w:sem  4
 236 wq:UART4                     2882  0.597   744/ 1396 234 (234)  w:sem  4
 245 navigator                      65  0.099   932/ 1764 105 (105)  w:sem  5
 334 top                             1  0.000  1256/ 2028 239 (239)  RUN    3
 303 logger                       2182  0.398  2152/ 3644 230 (230)  w:sem  3
 314 log_writer_file                 0  0.000   368/ 1164  60 ( 60)  w:sem  3

Processes: 26 total, 2 running, 24 sleeping, max FDs: 15
CPU usage: 39.50% tasks, 2.89% sched, 57.61% idle
DMA Memory: 5120 total, 1024 used 1024 peak
Uptime: 494.400s total, 284.396s idle
nsh>
```

PX4 uses publish/subscribe to pass data, the pub/sub module is called uORB. To see the status of uORB topics run:
```
nsh> uorb top

update: 1s, num topics: 46
TOPIC NAME                 INST #SUB RATE #Q SIZE
actuator_armed                0    7    2  1   24
adc_report                    0    1  100  1   96
cpuload                       0    5    2  1   16
safety                        0    1    1  1   16
sensor_accel                  0    3  813  8   48
sensor_accel                  1    3  751  8   48
sensor_baro                   0    1   21  1   32
sensor_combined               0    2  203  1   48
sensor_gyro                   0    3  813  8   40
sensor_gyro                   1    3  751  8   40
system_power                  0    1  100  1   24
telemetry_status              0    2    1  1  104
telemetry_status              1    2    1  1  104
telemetry_status              2    2    1  1  104
vehicle_acceleration          0    1  203  1   32
vehicle_air_data              0    8   11  1   40
vehicle_angular_velocity      0    4  813  1   32
vehicle_control_mode          0   10    2  1   32
vehicle_imu                   0    3  203  1   56
vehicle_imu                   1    3  188  1   56
vehicle_imu_status            0    4    9  1   56
vehicle_imu_status            1    5   10  1   56
vehicle_status                0   22    2  1   56
nsh>
```

To print the data from the most recent publication of a given topic, run:
```
nsh> listener <name_of_topic>
```
eg:

```
nsh> listener sensor_accel

TOPIC: sensor_accel 2 instances

Instance 0:
 sensor_accel_s
	timestamp: 772172998  (0.009620 seconds ago)
	timestamp_sample: 772172672  (326 us before timestamp)
	device_id: 3670026 (Type: 0x38, SPI:1 (0x00))
	x: 9.4734
	y: 0.8705
	z: 1.0180
	temperature: 42.6157
	error_count: 0
	clip_counter: [0, 0, 0]

Instance 1:
 sensor_accel_s
	timestamp: 772179434  (0.012095 seconds ago)
	timestamp_sample: 772179145  (289 us before timestamp)
	device_id: 2621450 (Type: 0x28, SPI:1 (0x00))
	x: 9.6778
	y: 0.8631
	z: 0.7749
	temperature: 41.7506
	error_count: 0
	clip_counter: [0, 0, 0]
```
---
#### Interacting with the bq4050 driver
The driver is named `bq4050`. If you type that into the console you will see the manpage. Before trying to start the driver, check to make sure it is found on the i2c bus:
```
nsh> i2cdetect
Scanning I2C bus: 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```
As you can see, since I have nothing connected to the bus, it does not find any i2c devices at any of the available addresses.

Once the driver is successfully connected to the bq4050 and running, you will see the `battery_status` topic being published:
```
uorb top battery_status
```
You can view the data with listener
```
listener battery_status
```

The bq4050 driver is started automatically in the `rcS` startup script https://github.com/dakejahl/Firmware/blob/dev/ants_imu/ROMFS/cannode/init.d/rcS

All other ANTs board specific drivers/modules are started in the startup scripts found here https://github.com/dakejahl/Firmware/tree/dev/ants_imu/boards/ants/fmu-v4/init

---
There is a plethora of additional information to be found in the PX4 documentation. Make sure to read it!

- **Developer guide:** https://dev.px4.io/master/en/index.html
- **User guide:** https://docs.px4.io/master/en/index.html