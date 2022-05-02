"""
I have an IMU with an AHRS. The IMU is connected to the imu_link,
which is axis-aligned with the frd body frame. I want to rotate
the AHRS data so that it is axis-aligned with an flu body frame.

So imulinkfrd --> ahrsfrd, but i want imulinkflu --> ahrsflu.

If R_imulinkfrd_ahrsfrd is the orientation from the AHRS, then
I can make R_imulinkflu_ahrsflu (the desired) with

    R_flu_frd @ R_imulinkfrd_ahrsfrd @ R_flu_frd.T

This is a change of basis for the orientation.

for quat, just flip sign of y and z component

See e.g., Fig 4 of http://robots.engin.umich.edu/nclt/nclt.pdf
"""

import numpy as np
from scipy.spatial.transform import Rotation as Rot

# frd w.r.t flu; transforoms frd data into flu
# NED global has frd local; ENU global has flu
R_flu_frd = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

if __name__ == '__main__':
    r,p,y = 0.2, 0.01, 0.5

    # let's say this is given by the IMU AHRS
    q = Rot.from_euler('xyz', [r,p,y], degrees=False).as_quat() # extrinsic
    print(q)
    print()

    # rotate into the desired frame via change of basis
    R_imu_ahrs = R_flu_frd @ Rot.from_quat(q).as_matrix() @ R_flu_frd.T
    q_imu_ahrs = Rot.from_matrix(R_imu_ahrs).as_quat()
    print(q_imu_ahrs)
    print()

    # note that this is equivalent to the following
    x,y,z,w = q[0],q[1],q[2],q[3]
    q2_imu_ahrs = np.array([x,-y,-z,w])
    print(q2_imu_ahrs)
