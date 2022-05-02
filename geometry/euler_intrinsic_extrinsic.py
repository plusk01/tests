"""
I always forget Euler angle relationships w.r.t intrinsic, extrinsic.
I also always forget in which order the elementary rotations are applied.
Euler angles are the worst.
"""

import numpy as np
from scipy.spatial.transform import Rotation as Rot


# https://github.com/ethz-asl/nclt_tools/blob/master/nclt-tools/python/transformation_helpers.py
def getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad):
    R_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0.0],
                       [np.sin(yaw_rad), np.cos(yaw_rad), 0.0],
                       [0.0, 0.0, 1.0]])

    R_pitch = np.array([[np.cos(pitch_rad), 0.0, np.sin(pitch_rad)],
                       [0.0, 1.0, 0.0],
                       [-np.sin(pitch_rad), 0.0, np.cos(pitch_rad)]])

    R_roll = np.array([[1.0, 0.0, 0.0],
                      [0.0, np.cos(roll_rad), -np.sin(roll_rad)],
                      [0.0, np.sin(roll_rad), np.cos(roll_rad)]])

    R = np.dot(np.dot(R_yaw, R_pitch), R_roll)
    return R

if __name__ == '__main__':
    r,p,y = 0.2, 0.01, 0.5

    # these are the same SO(3) object
    R2 = Rot.from_euler('xyz', [r,p,y], degrees=False).as_matrix() # extrinsic
    R3 = Rot.from_euler('ZYX', [y,p,r], degrees=False).as_matrix() # intrinsic
    print(R2)
    print()
    print(R3)
    assert(np.allclose(R2,R3))

    # this is the order of multiplication; i would call this 1-2-3 extrinsic
    # since roll is the first rotation to be applied to the vector. However,
    # I think there is some ambiguity to the phrase "first rotation to be
    # applied". In my mind, it makes sense that "to be applied" is referring
    # to the right-most rotation, as we normally left multiply transforms.
    R1 = getRotationMatrixFromEulerAnglesRollPitchYaw(r,p,y)
    print()
    print(R1)
    assert(np.allclose(R1,R2))
