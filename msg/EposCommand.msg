
#Header header

uint8 type #command type:
# 1: motor position control (as many as size of array)
# 2: ?
# 3: ?
# 4: homing all motors
# 5: reinitilize all motors

uint8[] motor_id 
float32[] position
