/*

Copyright (c) 2011, Ascending Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

 */

#ifndef MAIN_H_
#define MAIN_H_

extern void mainloop(void);
extern void timer0ISR(void);
extern void timer1ISR(void);


volatile unsigned int GPS_timeout;
extern unsigned int GPS_cnt_trigger[4];
unsigned char GPS_init_status;

#define BATTERY_WARNING_VOLTAGE 10000	//9800 mV
#define BAT_DIV 10//(BATTERY_WARNING_VOLTAGE-9000)/100

//#define GPS_BEEP	//Warning, if GPS has no lock
//#define CHANNEL7_TO_5VSERVO_OUT	//CAM_Commands.desired_angle_roll is used for 5V servo output
#define TRIGGER_SERVO_ACTIVE	1800
#define TRIGGER_SERVO_INACTIVE	3600
#define CAMER_OFFSET_HUMMINGBIRD	63000

#define ControllerCyclesPerSecond 	1000

//#define INDOOR_GPS

//defines for VP_CAMREMOTE (in �s)
#define VP_SHOOT				2000
#define VP_LENSCLOSE_SLEEP		1900
#define VP_REFOCUS_LOCKFOCUS	1800
#define VP_ZOOM_IN				1700
#define VP_ZOOM_OUT				1600
#define VP_NEXT_PROFILE			1500
#define VP_PREVIOUS_PROFILE		1400
#define VP_SHUTTER_SPEED_INC	1300
#define VP_SHUTTER_SPEED_DEC	1200
#define VP_APERTURE_INC			1100
#define VP_APERTURE_DEC			1000


#define OFF 0
#define ON  1

#define NORMAL 0

//reconfigure GPS if needed
#define GPS_STARTUP 			0
#define GPS_NEEDS_CONFIGURATION	1
#define GPS_IS_CONFIGURED		2
#define GPS_CONFIG_ERROR		3

//packet descriptors
#define PD_IMURAWDATA       0x01
#define PD_LLSTATUS        	0x02
#define PD_IMUCALCDATA      0x03
#define PD_HLSTATUS        	0x04

#define PD_CTRLOUT			0x11
#define PD_FLIGHTPARAMS     0x12
#define PD_CTRLCOMMANDS		0x13
#define PD_CTRLINTERNAL		0x14
#define PD_RCDATA       	0x15
#define PD_CTRLSTATUS		0x16

#define PD_WAYPOINT     	0x20
#define PD_CURRENTWAY   	0x21
#define PD_NMEADATA     	0x22
#define PD_GPSDATA			0x23

#define PD_CAMERACOMMANDS	0x30

#define CAM_TRIGGERED		0x04

struct IMU_CALCDATA {
//angles derived by integration of gyro_outputs, drift compensated by data fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
    int angle_nick;
    int angle_roll;
    int angle_yaw;

//angular velocities, raw values [16 bit], bias free, in 0.0154 �/s (=> 64.8 = 1 �/s)
    int angvel_nick;
    int angvel_roll;
    int angvel_yaw;

//acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
    short acc_x_calib;
    short acc_y_calib;
    short acc_z_calib;

//horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
    short acc_x;
    short acc_y;
    short acc_z;

//reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree
    int acc_angle_nick;
    int acc_angle_roll;

//total acceleration measured (10000 = 1g)
    int acc_absolute_value;

//magnetic field sensors output, offset free and scaled; units not determined, as only the direction of the field vector is taken into account
    int Hx;
    int Hy;
    int Hz;

//compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
    int mag_heading;

//pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown; used for short-term position stabilization
    int speed_x;
    int speed_y;
    int speed_z;

//height in mm (after data fusion)
    int height;

//diff. height in mm/s (after data fusion)
    int dheight;

//diff. height measured by the pressure sensor [mm/s]
    int dheight_reference;

//height measured by the pressure sensor [mm]
    int height_reference;
};
extern struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;

struct IMU_RAWDATA {
//pressure sensor 24-bit value, not scaled but bias free
	int pressure;

//16-bit gyro readings; 32768 = 2.5V
    short gyro_x;
	short gyro_y;
	short gyro_z;

//10-bit magnetic field sensor readings
    short mag_x;
	short mag_y;
	short mag_z;

//16-bit accelerometer readings
    short acc_x;
	short acc_y;
    short acc_z;

//16-bit temperature measurement using yaw-gyro internal sensor
	unsigned short temp_gyro;

//16-bit temperature measurement using ADC internal sensor
	unsigned int temp_ADC;
};

extern struct IMU_RAWDATA IMU_RawData;


struct SYSTEM_PERMANENT_DATA
{
	unsigned int total_uptime;
	unsigned int total_flighttime;
	unsigned int onoff_cycles;
	unsigned int number_of_flights;
	unsigned int chksum;
};
extern struct SYSTEM_PERMANENT_DATA SYSTEM_Permanent_Data;

#endif /*MAIN_H_*/

