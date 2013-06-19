/*

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __HL_INTERFACE__
#define __HL_INTERFACE__

#include "extPositionCtrl.h"
#include "HL_SSDK.h"
#include <inttypes.h>

// Packet descriptors -----------------------------------------------
// don't use more than 255 packet types!
#define HLI_PACKET_ID_IMU                       0x01    ///< imu packet
#define HLI_PACKET_ID_STATUS                    0x02    ///< status packet
#define HLI_PACKET_ID_GPS                       0x03    ///< gps
#define HLI_PACKET_ID_RC                        0x04    ///< rc data
#define HLI_PACKET_ID_SSDK_DEBUG                0x05    ///< debug data from datafusion/position controller
#define HLI_PACKET_ID_CONTROL_LL                0x06    ///< this is for commands that get forwarded to the LL processor i.e. acceleration/attitude and gps velocities
#define HLI_PACKET_ID_CONTROL_HL                0x07    ///< commands for control by the HL
#define HLI_PACKET_ID_POSITION_UPDATE           0x08    ///< (absolute) position update from external reference
#define HLI_PACKET_ID_MOTORS                    0x09    ///< motor on/off commands
#define HLI_PACKET_ID_BAUDRATE                  0x0a    ///< set baudrate
#define HLI_PACKET_ID_SSDK_PARAMETERS           0x0b    ///< set params for matlab onboard controller
#define HLI_PACKET_ID_TIMESYNC                  0x0c    ///< timesync packet indentifier
#define HLI_PACKET_ID_CONTROL_ENABLE            0x0d    ///< packet to enable single axis for control
#define HLI_PACKET_ID_EKF_STATE                 0x0e    ///< packet to exchange the state of the position/attitude ekf
#define HLI_PACKET_ID_SUBSCRIPTION              0x0f    ///< packet to define the periods the packets are sent with
#define HLI_PACKET_ID_SSDK_STATUS               0x10    ///< SSDK status packet
#define HLI_PACKET_ID_ACK                       0x11    ///< Acknowledge packet
#define HLI_PACKET_ID_CONFIG                    0x12    ///< Acknowledge packet
#define HLI_PACKET_ID_MAG                       0x13    ///< Magnetic compass data packet


// flight mode defines for communication with LL processor ----------------------------------------------
#define HLI_FLIGHTMODE_ACC                      0x01
#define HLI_FLIGHTMODE_HEIGHT                   0x03
#define HLI_FLIGHTMODE_GPS                      0x07
#define HLI_SERIALINTERFACE_ENABLED_BIT         0x20
#define HLI_SERIALINTERFACE_ACTIVE_BIT          0x40
#define HLI_PITCH_BIT                           0x01
#define HLI_ROLL_BIT                            0x02
#define HLI_YAW_BIT                             0x04
#define HLI_THRUST_BIT                          0x08
#define HLI_HEIGHT_BIT                          0x10
#define HLI_GPS_BIT                             0x20

// communication defaults
#define HLI_COMMUNICATION_TIMEOUT               3
#define HLI_DEFAULT_BAUDRATE                    57600
#define HLI_DEFAULT_PERIOD_IMU                  20
#define HLI_DEFAULT_PERIOD_GPS                  200
#define HLI_DEFAULT_PERIOD_SSDK_DEBUG           200
#define HLI_DEFAULT_PERIOD_RCDATA               100
#define HLI_DEFAULT_PERIOD_EKF_STATE            0

// position control mode and state estimation defines
#define HLI_MODE_STATE_ESTIMATION_OFF           0x01    ///< switches state estimation on the HL off
#define HLI_MODE_STATE_ESTIMATION_HL_SSDK       0x02    ///< state estimation is performed on the HL in the SSDK based on positions that are sent to it
#define HLI_MODE_STATE_ESTIMATION_HL_EKF        0x04    ///< state estimation is performed on the HL based on the state prediction of an EKF
#define HLI_MODE_STATE_ESTIMATION_EXT           0x08    ///< state estimation is performed externally, e.g. on the host computer
#define HLI_MODE_POSCTRL_OFF                    0x10    ///< position control switched off
#define HLI_MODE_POSCTRL_LL                     0x20    ///< position control in performed on the LL controller. This just works if GPS is available
#define HLI_MODE_POSCTRL_HL                     0x40    ///< use position controller on the HL based on state estimation on the HL or external state estimation

// communication flags
#define HLI_COMM_ACK                            0x01    ///< defines that a packet has to be acknowledged when received

/// imu packet
typedef struct
__attribute__((packed))
{

  /// timestamp in us; synchronized to host pc
  int64_t timestamp;

  /// accelerations [mg]
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;

  /// angular velocities; bias-free in units of 0.015deg/s
  int16_t ang_vel_roll;
  int16_t ang_vel_pitch;
  int16_t ang_vel_yaw;

  /// angles [deg*100]
  int16_t ang_roll;
  int16_t ang_pitch;

  /// yaw from 0...360 [deg*100]
  uint16_t ang_yaw;

  /// height from pressure sensor [mm]
  int32_t height;

  /// differentiated height [mm/s]
  int16_t differential_height;

  /// motor speeds
  uint8_t motors[6];
} HLI_IMU;

/// gps packet
typedef struct
__attribute__((packed))
{
  /// GPS data fused with all other sensors

  /// timestamp in us; synchronized to host pc
  int64_t timestamp;

  /// latitude/longitude in deg * 10^7
  int32_t latitude;
  int32_t longitude;

  /// height [mm]
  uint32_t height;

  /// pressure height [mm]
  int32_t pressure_height;

  /// speed in x (E/W) and y(N/S) in mm/s
  int32_t speedX;
  int32_t speedY;

  /// heading from 0...360 [deg*100]
  uint16_t heading;

  /// accuracy estimates [mm] / [mm/s]
  uint32_t horizontalAccuracy;
  uint32_t verticalAccuracy;
  uint32_t speedAccuracy;

  /// GPS status information;
  /**
   * Bit7...Bit3: 0
   * Bit 2: longitude direction
   * Bit1: latitude direction
   * Bit 0: GPS lock
   */
  int32_t status;

  /// number of satellite vehicles used in NAV solution
  uint32_t numSatellites;
}HLI_GPS;

/// status of the helicopter
typedef struct
__attribute__((packed))
{
  /// timestamp in us; synchronized to host pc
  int64_t timestamp;

  /// battery voltage [mv]
  int16_t battery_voltage;

  uint16_t state_estimation;
  uint16_t position_control;
  uint16_t flight_mode;

  /// Flight time [s]
  uint16_t flight_time;

  /// cpu load in % of the time of one sdk loop
  uint16_t cpu_load;

  int32_t debug1;
  int32_t debug2;

  /// Motor Status:
  /**
   * -1: motors off
   *  0: stop motors
   *  1: start motors
   *  2: motors on
   */
  int8_t motors;

  /// total received packets
  uint32_t rx_packets;

  /// total received good packets
  uint32_t rx_packets_good;

  /// indicates if parameters for onboard filters/controller are available
  int8_t have_SSDK_parameters;

  int64_t timesync_offset;

} HLI_STATUS;

#define HLI_NUMBER_RC_CHANNELS 8
/// contains rc data that was transmitted to the helicopter
typedef struct
__attribute__((packed))
{
  /// timestamp in us; synchronized to host pc
  int64_t timestamp;

  uint16_t channel[HLI_NUMBER_RC_CHANNELS];
  /**
   * channel[0]: Pitch
   * channel[1]: Roll
   * channel[2]: Thrust
   * channel[3]: Yaw
   * channel[4]: Serial interface enable/disable
   * channel[5]: manual / height control / GPS + height control
   *
   * range of each channel: 0..4095
   */
}HLI_RCDATA;

/// struct for LL control commands
typedef struct
__attribute__((packed))
{
  /// pitch angle / velocity x
  int16_t x;
  /// roll angle / velocity y
  int16_t y;
  /// yaw angular velocity
  int16_t yaw;
  /// thrust(=acceleration z) / climbrate
  int16_t z;
} HLI_CMD_LL;

typedef struct
__attribute__((packed))
{
  /// 0: stop motors, 1: start motors
  uint8_t motors;
} HLI_MOTORS;

typedef struct
__attribute__((packed))
{
  uint32_t state;
  uint32_t baudrate;
} HLI_BAUDRATE;

/// from included file provided by asctec, defines position updates from an external reference
typedef struct EXT_POSITION HLI_EXT_POSITION;

/// from included file provided by asctec, defines position/velocity commands
typedef struct EXT_POSITION_CMD HLI_CMD_HL;

/// packet for time synchronization between HL processor and PC
/**
 *   Client |        | Server
 *          |        |
 *      tc1 |--------| ts1
 *          |        |
 *      tc2 |--------| ts2
 *          |        |
 *
 *  ts1=ts2; tc2 is measured on the arrival of the packet
 */
typedef struct
__attribute__((packed))
{
  int64_t tc1;
  int64_t ts1;
} HLI_TIMESYNC;

#define HLI_EKF_STATE_SIZE 16
#define HLI_EKF_STATE_CURRENT_STATE 1
#define HLI_EKF_STATE_INITIALIZATION 2
#define HLI_EKF_STATE_STATE_CORRECTION 3

typedef struct
__attribute__((packed))
{
  int64_t timestamp;

  /// accelerations [mg]
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;

  /// angular velocities; bias-free in units of 0.015deg/s
  int16_t ang_vel_roll;
  int16_t ang_vel_pitch;
  int16_t ang_vel_yaw;

  /// state in SI units
  float state[HLI_EKF_STATE_SIZE];

  uint32_t flag;
}HLI_EKF_STATE;

/// define at which frequencies data should be sent
typedef struct
__attribute__((packed))
{
  uint16_t imu; ///< imu packet, 20 ms
  uint16_t rcdata; ///< rc channels, 50 ms
  uint16_t ssdk_debug; ///< debug channels from the ssdk, 200 ms
  uint16_t gps; ///< gps data, 200 ms
  uint16_t ekf_state; ///< pose ekf state, 0 --> set to zero if you don't want to have it
  uint16_t mag; ///< magnetic compass packet
}HLI_SUBSCRIPTION;

/// configuration packet for HL processor
typedef struct
__attribute__((packed))
{
  uint16_t mode_state_estimation;
  uint16_t mode_position_control;
  uint16_t position_control_axis_enable;
  uint16_t battery_warning_voltage;
}HLI_CONFIG;

/// packet for SSDK status
typedef struct
__attribute__((packed))
{
  uint8_t have_parameters;
}HLI_SSDK_STATUS;

/// acknowledge packet with message type that has been acknowledged
typedef struct
__attribute__((packed))
{
  uint8_t ack_packet;
}HLI_ACK;

/// packet with magnetic compass readings
typedef struct
__attribute__((packed))
{
  int64_t timestamp;

  int16_t x;
  int16_t y;
  int16_t z;
}HLI_MAG;


#endif
