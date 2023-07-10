# Object label
string label

# Object label ID
int16 label_id

# Object sub
string sublabel

# Object confidence level (1-99)
float32 confidence

# Object centroid position
float32[3] position

# Position covariance
float32[6] position_covariance

# Object velocity
float32[3] velocity

# Tracking available
bool tracking_available

# Tracking state
# 0 -> OFF (object not valid)
# 1 -> OK
# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)
int8 tracking_state

# Action state
# 0 -> IDLE
# 2 -> MOVING
int8 action_state

# 3D dimensions (width, height, lenght)
float32[3] dimensions_3d

# 2D bounding box that I had to manually yank in here because zed github is trash
Keypoint2Di[4] corners