# Control command produced by the master-MCU node for one motor.
uint8 MODE_DISABLED=0
uint8 MODE_ACCELERATION=1

std_msgs/Header header
uint8 motor_id
uint32 observation_sequence
uint8 mode
float64 acceleration_command_rad_s2
