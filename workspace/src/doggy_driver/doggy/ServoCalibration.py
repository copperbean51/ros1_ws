# Mini Pupper stores calibration data in nvram
# this is where we read it from if it is available

import numpy as np

MICROS_PER_RAD = 11.111 * 180.0 / np.pi
NEUTRAL_ANGLE_DEGREES = np.array(
[[  0.,  0.,  0.,  0.],
 [  0.,  0.,  0.,  0.],
 [  0.,  0.,  0.,  0.]]
)

try:
    MICROS_PER_RAD = data['MICROS_PER_RAD']
    NEUTRAL_ANGLE_DEGREES = data['NEUTRAL_ANGLE_DEGREES']
except:
    pass
