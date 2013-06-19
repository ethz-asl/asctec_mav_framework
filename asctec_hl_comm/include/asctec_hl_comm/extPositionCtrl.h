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

#ifndef EXTPOSITIONCTRL_H_
#define EXTPOSITIONCTRL_H_

struct __attribute__((packed)) EXT_POSITION
{
  /** Struct to send the current postion to the HL controller. In normal operation, only x, y, z, yaw have to be provided. 
   * When bitfield is set to EXT_POSITION_BYPASS_FILTER, the state estimator is bypassed and velocity MUST be provided. 
   * This is useful, if you have your own state estimation and just want to use the position controller. 
   * qualXX is currently "binary":set the value to greater than 30 if your estimate is valid. For lower values, the last valid position will be hold. 
   * coordinate system: ENU
   * x: front
   * y: left
   * z: up
   * yaw: ccw when viewed from top 
   */

  int x; ///< X-Position in mm
  int y; ///< Y-Position in mm
  int z; ///< Z-Position in mm
  int heading; ///< heading in 1/1000 degree. 0..360000
  short vX; ///< speed in X direction in mm/s
  short vY; ///< speed in Y direction in mm/s
  short vZ; ///< speed in Z direction in mm/s
  unsigned short bitfield; //
  unsigned char qualX; ///< Quality of X measurement. 0: poor; 100: excellent
  unsigned char qualY; ///< Quality of Y measurement. 0: poor; 100: excellent
  unsigned char qualZ; ///< Quality of Z measurement. 0: poor; 100: excellent
  unsigned char qualVx; ///< Quality of X speed measurement. 0: poor; 100: excellent
  unsigned char qualVy; ///< Quality of y speed measurement. 0: poor; 100: excellent
  unsigned char qualVz; ///< Quality of Z speed measurement. 0: poor; 100: excellent
  unsigned char count;
};

struct __attribute__((packed)) EXT_POSITION_CMD
{
  /** Struct to send velocity or position commands
   * coordinate system: ENU
   * x: front
   * y: left
   * z: up
   * yaw: ccw when viewed from top 
   */

  unsigned int seq; /// sequence counter
  int x; ///< desired X-Position in mm
  int y; ///< desired Y-Position in mm
  int z; ///< desired Z-Position in mm
  int heading; ///< desired heading in 1/1000 degree. 0..360000
  short vX; ///< desired speed in X direction in mm/s
  short vY; ///< desired speed in Y direction in mm/s
  short vZ; ///< desired speed in Z direction in mm/s
  short vYaw; ///< desired turn rate of yaw in 1/1000 deg/s
  short vMaxXY; ///< max velocity xy to reach a waypoint
  short vMaxZ; ///< max velocity z to reach a waypoint
  unsigned short bitfield; //
};
#endif /* EXTPOSITIONCTRL_H_ */

/// resets the position filter
#define EXT_POSITION_RESET 0x8000

/// bypasses the position filter --> speed AND position must be provided!
#define EXT_POSITION_BYPASS_FILTER 0x0001

/// sets the controller's setpoint to the current position
#define EXT_POSITION_CMD_RESET 			0x8000

/// sets control mode to velocity-control
#define EXT_POSITION_CMD_VELOCITY		0x0001

/// sets control mode to position control
#define EXT_POSITION_CMD_POSITION       0x0000

/// sets control mode to position control
#define EXT_POSITION_CMD_WORLDFIXED       0x0000

/// sets control mode to position control
#define EXT_POSITION_CMD_BODYFIXED       0x0010

