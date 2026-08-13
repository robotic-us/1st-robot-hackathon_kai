# One motor's trajectory reference and current state. All motion units are SI.
std_msgs/Header header
uint8 motor_id
int8 axis_index
bool connected
bool valid
bool oper
bool fault
bool stale
uint32 sequence

# Requested endpoint and instantaneous quintic trajectory reference.
float64 target_position_rad
float64 desired_position_rad
float64 desired_velocity_rad_s
float64 desired_acceleration_rad_s2

# Raw/simulated measurement and filtered or estimated current state.
float64 measured_position_rad
float64 filtered_position_rad
float64 estimated_velocity_rad_s
float64 estimated_acceleration_rad_s2
float64 current_a
float64 disturbance_current_a
float64 bus_voltage_v
float64 temperature_c
bool absolute_encoder_valid

# Controller error and output.
float64 position_error_rad
float64 velocity_error_rad_s
float64 control_acceleration_rad_s2
