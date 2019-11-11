/**
 * @file debugging_node.cpp
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 11 Nov 2019
 */

#include <memory>

#include <ros/ros.h>

#include <std_msgs/Bool.h>

class Thing
{
public:
  Thing() = default;
  ~Thing() = default;

  void compute() { data_ = true; }
private:
  bool data_ = false;
};

class Debugging
{
public:
  Debugging(const ros::NodeHandle nh, const ros::NodeHandle nhp)
  : nh_(nh), nhp_(nhp)
  {
    // force a segfault
    // *(int*)0=0;

    sub_in_ = nh_.subscribe("in", 1, &Debugging::inCb, this);
  }
  ~Debugging() = default;

private:
  ros::NodeHandle nh_, nhp_;
  ros::Subscriber sub_in_;
  std::unique_ptr<Thing> thing_;

  void inCb(const std_msgs::BoolConstPtr& msg)
  {
    ROS_INFO("Got input");
    thing_->compute();
  }

};

// ============================================================================
// ============================================================================

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Debugging");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  Debugging node(nhtopics, nhparams);
  ros::spin();
  return 0;
}
