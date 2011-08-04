#ifndef LL_HL_COMM_
#define LL_HL_COMM_

//system flags
#define SF_PAGE_BIT1 				0x01
#define SF_PAGE_BIT2 				0x02
#define SF_SSP_ACK					0x04
#define SF_GPS_NEW					0x08
#define SF_HL_CONTROL_ENABLED 		0x10
#define SF_DIRECT_MOTOR_CONTROL 	0x20
#define SF_WAYPOINT_MODE			0x40

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

	short dummy_333Hz_1;
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
	short flight_time;
	short cpu_load;
	short status;
	short status2;

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
	short dummy_500Hz_2;
	short dummy_500Hz_3;
};

extern struct LL_CONTROL_INPUT LL_1khz_control_input;



#endif /*LL_HL_COMM_*/
