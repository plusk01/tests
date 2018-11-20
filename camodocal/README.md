CamOdoCal
=========

The CamOdoCal library (2011-2017) was created at ETH-Zurich for the purpose of calibrating
the intrinisic and extrinsic parameters of multi-camera rigs. Additionally,
odometry can be used to aid in the optimization-based calibration. The optimizer
used in CamOdoCal is Google's Ceres Solver.

A main reason to use this library even when not dealing with multi-camera rigs
is the out-of-the-box camera models it provides:
- Pinhole
- Catadioptric
  ([MEI](http://www.robots.ox.ac.uk/~cmei/articles/single_viewpoint_calib_mei_07.pdf), fisheye, spherical)
- Wide-angle and fish-eye cameras
  ([Kannala-Brandt](http://www.ee.oulu.fi/mvg/files/pdf/pdf_697.pdf))

For this reason, [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) leverages a stripped-down version of CamOdoCal for
better usage of a wide variety of well-calibrated cameras.
