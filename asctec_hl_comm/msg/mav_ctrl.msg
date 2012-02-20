Header header

int8        type        #message type

# control commands, all units in SI units !!!
# There are 3 operating modes of the helicopter:
# 1. Acceleration: x, y, z correspond to x_dotdot etc... with the exception that yaw is angular velocity
#    Inputs must be in body-coordinates
#    Currently x~pitch, y~roll, z~thrust, units in rad and rad/s for yaw
# 2. Velocity: x, y, z, yaw correspond to x_dot etc...
#    Vehicle must have it's own pose estimation e.g. vision/GPS
#    Inputs must be in body-coordinates
#    Units in m/s and rad/s respectively
# 3. Position
#    Vehicle must have it's own pose estimation e.g. vision/GPS
#    Inputs in fixed coordinate system
#    v_max_* is only valid for this mode and limits the velocity a vehicle approaches the waypoint

float32     x
float32     y
float32     z
float32     yaw
float32     v_max_xy
float32     v_max_z

int8 acceleration = 1
int8 velocity = 2
int8 position = 3
int8 velocity_body = 4
int8 position_body = 5
