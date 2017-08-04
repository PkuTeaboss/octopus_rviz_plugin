Header header

uint8 NONE=0

# ------ Left Area (x direction) -------

# MPH
float32 speed

# 0 - 1
float32 throttle_percent

# 0 - 1
float32 brake_percent

float32 x_accel_data
float32 x_accel_upper_bound
float32 x_accel_lower_bound
float32 x_accel_upper_comfort
float32 x_accel_lower_comfort

float32 x_jerk_data
float32 x_jerk_upper_bound
float32 x_jerk_lower_bound
float32 x_jerk_upper_comfort
float32 x_jerk_lower_comfort

# --------------------------------------


# ------ Right Area (y direction) -------

# In radians, range -8.2 to 8.2
float32 steering_wheel_angle

uint8 LEFT=1
uint8 RIGHT=2
uint8 turn_signal

float32 y_accel_data
float32 y_accel_upper_bound
float32 y_accel_lower_bound
float32 y_accel_upper_comfort
float32 y_accel_lower_comfort

float32 y_jerk_data
float32 y_jerk_upper_bound
float32 y_jerk_lower_bound
float32 y_jerk_upper_comfort
float32 y_jerk_lower_comfort

# --------------------------------------


# ------ Middle Area (controls) -------

uint8 DBW=1
uint8 JOY=2

uint8 SPEED=1
uint8 PEDAL=2

uint8 YAWRATE=1
uint8 STEERING=2

uint8 linear_cmd_source
uint8 linear_cmd_method
float32 linear_cmd_data

uint8 angular_cmd_source
uint8 angular_cmd_method
float32 angular_cmd_data

# --------------------------------------


# ------ Middle Area (dbw enabled) -------

bool dbw_enabled

# --------------------------------------


