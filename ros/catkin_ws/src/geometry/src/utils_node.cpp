/**
 * @file multitask_node.cpp
 * @brief Entry point for multitask runner
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 20
 */

#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void printQ(const tf2::Quaternion& q)
{
  // print components
  std::cout << "q: " << q.w() << " , " << q.x() << ", ";
  std::cout << q.y() << ", " << q.z() << std::endl;

  // print axis
  std::cout << "ax " << q.getAxis().x() << ", ";
  std::cout << q.getAxis().y() << ", " << q.getAxis().z() << std::endl;
  // print angle
  std::cout << "an " << q.getAngle() << std::endl;

  std::cout << std::string(10, '-') << std::endl;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Wrap angle so that it is in [-pi, pi]
 *
 * @param[in]  angle  The angle to wrap
 *
 * @return     The wrapped angle
 */
static double wrapToPi(double angle)
{
  if (angle >  M_PI) return angle - 2*M_PI;
  if (angle < -M_PI) return angle + 2*M_PI;
  return angle;
}

// ----------------------------------------------------------------------------

/**
 * @brief      Wrap angle so that it is in [0, 2*pi]
 *
 * @param[in]  angle  The angle to wrap
 *
 * @return     The wrapped angle
 */
static double wrapTo2Pi(double angle)
{
  if (angle > 2*M_PI) return angle - 2*M_PI;
  if (angle < 0)      return angle + 2*M_PI;
  return angle;
}

// ============================================================================
// ============================================================================

int main(int argc, char *argv[])
{
  // wrapped between [0, 2*pi]
  double Y = wrapTo2Pi(M_PI + M_PI/2);
  std::cout << "Y: " << Y << std::endl;

  tf2::Quaternion q;
  q.setRPY(0, 0, Y);
  printQ(q);

  // wrapped between [-pi, pi]
  Y = wrapToPi(Y);
  std::cout << "Y: " << Y << std::endl;

  q.setRPY(0, 0, Y);
  printQ(q);

  return 0;
}
