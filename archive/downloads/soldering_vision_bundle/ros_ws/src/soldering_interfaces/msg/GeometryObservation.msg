# Fail-closed planar geometry observation produced from one camera frame.
std_msgs/Header header
uint32 sequence
bool valid
bool calibrated
string detector_backend
float32 inference_ms
float32 image_age_ms

VisionObject[] objects

bool fixed_wire_valid
float32 fixed_wire_x_mm
float32 fixed_wire_y_mm
bool moving_wire_valid
float32 moving_wire_x_mm
float32 moving_wire_y_mm
bool iron_tip_valid
float32 iron_tip_x_mm
float32 iron_tip_y_mm
bool solder_wire_valid
float32 solder_wire_x_mm
float32 solder_wire_y_mm

float32 wire_error_x_mm
float32 wire_error_y_mm
float32 iron_to_joint_mm
float32 solder_to_joint_mm
float32 solder_to_insulation_mm

string process_class
float32 process_confidence
bool occluded
