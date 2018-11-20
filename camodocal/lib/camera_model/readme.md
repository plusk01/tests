camera_model
============

Taken from [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) (`b638c081e109267c5b350f03cee526f8aa6d28af`).

Stripped down further and using [this](https://github.com/ttroy50/cmake-examples/tree/master/02-sub-projects/A-basic) CMake example.

---

part of [camodocal](https://github.com/hengli/camodocal)

[Google Ceres](http://ceres-solver.org) is needed.

# Calibration:

Use [intrinsic_calib.cc](https://github.com/dvorak0/camera_model/blob/master/src/intrinsic_calib.cc) to calibrate your camera.

# Undistortion:

See [Camera.h](https://github.com/dvorak0/camera_model/blob/master/include/camodocal/camera_models/Camera.h) for general interface: 

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)

