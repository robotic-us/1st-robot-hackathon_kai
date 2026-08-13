# One tracked visual object in image and calibrated work-plane coordinates.
uint8 UNKNOWN=0
uint8 FIXED_WIRE=1
uint8 MOVING_WIRE=2
uint8 IRON_TIP=3
uint8 SOLDER_WIRE=4
uint8 SOLDER_JOINT=5
uint8 INSULATION=6
uint8 COPPER=7

uint8 class_id
string class_name
int32 track_id
float32 confidence

float32 bbox_x_px
float32 bbox_y_px
float32 bbox_width_px
float32 bbox_height_px
float32 anchor_u_px
float32 anchor_v_px
float32 mask_area_px

bool position_valid
float32 x_mm
float32 y_mm
float32 z_mm
