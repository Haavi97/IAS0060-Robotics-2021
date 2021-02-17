Header header
#mode for fast imprecise movement
string MODE_FAST=FAST
#mode for precise movement with flippers up; adaptive power consumption proportional to speed (as fast as boyance controls can keep up...)
string MODE_SLOW=SLOW

string mode
float32 mode_change_time	#max time to switch to the new mode; 0 = as fast as possible
float32 dir_change_time		#max time to rotate flipper from one direction to another; 0 = as fast as possible
float32 oa_distance 		#distances less than this number of meters stop robot from moving in that direction, 0 disables Obstacle Avoidance