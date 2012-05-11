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

#ifndef LL_HL_COMM_
#define LL_HL_COMM_

//system flags
#define SF_PAGE_BIT1 						0x01
#define SF_PAGE_BIT2 						0x02
#define SF_SSP_ACK							0x04
#define SF_GPS_NEW							0x08
#define SF_HL_CONTROL_ENABLED 				0x10
#define SF_DIRECT_MOTOR_CONTROL 			0x20
#define SF_WAYPOINT_MODE					0x40
#define SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL	0x80
#define SF_SDK_DISABLE_MOTORONOFF_BY_STICK	0x100
#define SF_NEW_SDK							0x8000


//ctrl_flags
//scientific control
#define HL_CTRL_PITCH			0x01
#define HL_CTRL_ROLL			0x02
#define HL_CTRL_YAW 			0x04
#define HL_CTRL_THRUST			0x08
#define HL_CTRL_HEIGHT_ENABLED	0x10
#define HL_CTRL_GPS_ENABLED		0x20

//direct motor control
#define HL_CTRL_MOTORS_ONOFF_BY_RC	0x01



extern unsigned char wpCtrlWpCmd;
extern unsigned char wpCtrlWpCmdUpdated;

extern unsigned char wpCtrlAckTrigger;

extern unsigned short wpCtrlNavStatus;
extern unsigned short wpCtrlDistToWp;

struct WAYPOINT {
//always set to 1
  unsigned char wp_number;

//was dummy_1 and dummy_2
  char cam_angle_roll; //-127..127 -127=-30° +127=+30° -> ~0.25° Resolution, or number of photos @ waypoint if time&0x4000
  unsigned short cam_angle_pitch; //   0=-11000° max=22000=+11000° in 1/100th° cam_angle_nick and roll are used when 0x8000 is set!

//set to 0x01 for now (WPPROP_ABSCOORDS=0x01) => absolute coordinates
  unsigned char properties;

//max. speed to travel to waypoint in % (default 100)
  unsigned char max_speed;

//time to stay at a waypoint (XYZ) in 1/100th s
  unsigned short time;

//position accuracy to consider a waypoint reached in mm (default: 2500 (= 2.5 m))
  unsigned short pos_acc;

//chksum = 0xAAAA + wp.yaw + wp.height + wp.time + wp.X + wp.Y + wp.max_speed + wp.pos_acc + wp.properties + wp.wp_number;
  short chksum;

 //waypoint coordinates in mm 	// longitude in abs coords
  int X;
 //waypoint coordinates in mm  	// latitude in abs coords
  int Y;

//Not yet implemented
  int yaw;

//height over 0 reference in mm, NOT YET IMPLEMENTED
  int height;
};

//Waypoint navigation status (unsigned short navigation_status in CURRENT_WAY struct)
#define WP_NAVSTAT_REACHED_POS 			0x01	//vehicle has entered a radius of WAYPOINT.pos_acc and time to stay is not neccessarily over
#define WP_NAVSTAT_REACHED_POS_TIME		0x02 	//vehicle is within a radius of WAYPOINT.pos_acc and time to stay is over
#define WP_NAVSTAT_20M					0x04 	//vehicle within a 20m radius of the waypoint
#define WP_NAVSTAT_PILOT_ABORT			0x08	//waypoint navigation aborted by safety pilot

#define WPPROP_ABSCOORDS 			0x01	//if set waypoint is interpreted as absolute coordinates, else relative coords
#define WPPROP_HEIGHTENABLED 		0x02  	//set new height at waypoint
#define WPPROP_YAWENABLED 			0x04	//set new yaw-angle at waypoint
#define WPPROP_WAITFORUSERCOMMAND 	0x08 	//if set, the vehicle will wait for a command before travelling to the next waypoint
#define WPPROP_AUTOMATICGOTO		0x10 	//if set, vehicle will not wait for a goto command, but goto this waypoint directly
#define WPPROP_CAM_TRIGGER          0x20    //if set, photo camera is triggered when waypoint is reached and time to stay is 80% up
#define WPPROP_LAND                 0x40    //land at WP coordinates
#define WPPROP_LAUNCH               0x80    //launch at current coordinates, WP coordinates set as soon as vehicle has reached height

extern struct WAYPOINT wpToLL;

#define WP_CMD_SINGLE_WP 	0x01 //transmit single waypoint command
#define WP_CMD_LAUNCH		0x02 //launch to 10m at current position
#define WP_CMD_LAND			0x03 //land at current position
#define WP_CMD_GOHOME		0x04 //come home at current height. Home position is set when motors are started or with WP_CMD_SETHOME
#define WP_CMD_SETHOME		0x05 //save current home position
#define WP_CMD_ABORT		0x06 //abort navigation (stops current waypoint flying)

#define WP_CMD_SINGLE_WP_PART1 	0x81 //internal use!
#define WP_CMD_SINGLE_WP_PART2 	0x82 //internal use!

//Slow Data Up Channel defines

#define SUDC_NONE				0x00
//flight time. flighttime in slowDataUpChannelShort
#define SUDC_FLIGHTTIME			0x01
//set camera type. Cameratype in slowDataUpChannelShort
#define SUDC_SETCAMERA			0x02
//set payload options. payloadoptions in slowDataUpChannelShort
#define SUDC_SETPAYLOADOPTIONS	0x03
//set camera mount roll angle calibration
#define SUDC_SETCAMERAROLLANGLECALIB 0x04
//set camera mount roll angle calibration and store to eeprom
#define SUDC_SETCAMERAROLLANGLECALIBANDSTORE 0x05
//navigation status
#define SUDC_NAVSTATUS 0x06
//distance to waypoint
#define SUDC_DISTTOWP  0x07
//waypoint ack trigger
#define SUDC_WPACKTRIGGER 0x08

void LL_write_ctrl_data(char);
int HL2LL_write_cycle(void);
inline void SSP_rx_handler_HL(unsigned char);
inline void SSP_data_distribution_HL(void);
struct LL_ATTITUDE_DATA
{
	unsigned short system_flags;	//GPS data acknowledge, etc.

	short angle_pitch;	//angles [deg*100]
	short angle_roll;
	unsigned short angle_yaw;

	short angvel_pitch;	//angular velocities; bias-free [0.015°/s]
	short angvel_roll;
	short angvel_yaw;

				//<-- 14 bytes @ 1kHz
				//--> 3x 26 bytes @ 333 Hz
				//=> total = 40 bytes @ 1 kHz
//-----------------------------PAGE0
	unsigned char RC_data[10];	//8 channels @ 10 bit

	int latitude_best_estimate;	//GPS data fused with all other sensors
	int longitude_best_estimate;
	short acc_x;		//accelerations [mg]
	short acc_y;
	short acc_z;

	unsigned short temp_gyro;
//-----------------------------PAGE1
	unsigned char motor_data[16];	//speed 0..7, PWM 0..7

	short speed_x_best_estimate;
	short speed_y_best_estimate;
	int height;		//height [mm]
	short dheight;		//differentiated height[mm/s]
//------------------------------PAGE2
	short mag_x;
	short mag_y;
	short mag_z;

	short cam_angle_pitch;
	short cam_angle_roll;
	short cam_status;

	short battery_voltage1;
	short battery_voltage2;
	short flightMode;
	short slowDataUpChannelDataShort; //former flight_time
	short cpu_load;
	short status;
	short status2; //Bits 7..1: slowDataUpChannelSelect (7bit) Bit0:flying Bit15..8:active Motors

};

extern struct LL_ATTITUDE_DATA LL_1khz_attitude_data;

struct LL_CONTROL_INPUT
{
	unsigned short system_flags;
							//bit 0: page_select
							//bit 1: reserved (page_select)
							//bit 2: SSP_ack
							//bit 3: GPS new
							//bit 4: HL controller enabled
							//bit 5: 0 -> "scientific" commands
							//       1 -> direct motor commands
							//bit 6: waypoint mode

	unsigned short ctrl_flags;
							//bit 0..3:
							// pitch, roll, yaw, height enable bits
							//bit 4: height control enabled
							//bit 5: yaw_control enabled
							//bit 8..15: waypoint command if waypoint mode is active or POI options if POI mode is active

	short pitch, roll, yaw, thrust; 		//"scientific interface"
	unsigned char direct_motor_control[8];		//direct motor commands: pitch, roll, yaw, throttle, 4xDNC
							//or motor 0..7 (Falcon)

							//<-- 20 bytes @ 1kHz
							//--> 2x18 bytes @ 500 Hz
							//=> total = 38 bytes @ 1kHz

	int latitude;					//data received from GPS-unit
	int longitude;
	int height;
	short speed_x;
	short speed_y;
	short status;
//-----------------------------

	unsigned short hor_accuracy;
	unsigned short vert_accuracy;
	unsigned short speed_accuracy;
	unsigned short numSV;
	unsigned short heading;
	short battery_voltage_1, battery_voltage_2; 	//battery voltage read by HL-ADC [mV]
	unsigned char slowDataChannelSelect; // these three vars define a slow data transfer channel. the select byte defines which data is in the data channel
	unsigned char slowDataChannelDataChar;
	short slowDataChannelDataShort;
};
extern struct LL_CONTROL_INPUT LL_1khz_control_input;



#endif /*LL_HL_COMM_*/
