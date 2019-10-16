/**
 * @file multitask_node.cpp
 * @brief Entry point for multitask runner
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 20
 */

#include <ros/ros.h>

#include <std_msgs/Bool.h>

class Multitask
{
public:
  Multitask(const ros::NodeHandle nh, const ros::NodeHandle nhp)
  : nh_(nh), nhp_(nhp)
  {
    // timer-based tasks
    tim_taskA_ = nh_.createTimer(ros::Duration(0.1), &Multitask::taskACb, this);
    tim_taskB_ = nh_.createTimer(ros::Duration(0.01), &Multitask::taskBCb, this);
    // tim_taskC_ = nh_.createTimer(ros::Duration(0.01), &Multitask::taskCCb, this);

    sub_in_ = nh_.subscribe("in", 1, &Multitask::inCb, this);
  }
  ~Multitask() = default;
  

private:
  ros::NodeHandle nh_, nhp_;
  ros::Subscriber sub_in_;
  ros::Timer tim_taskA_, tim_taskB_, tim_taskC_;

  void inCb(const std_msgs::BoolConstPtr& msg)
  {
    ROS_INFO("Got input");
  }

  // --------------------------------------------------------------------------

  void taskACb(const ros::TimerEvent& event)
  {
    ROS_INFO("Task A");
  }

  // --------------------------------------------------------------------------

  void taskBCb(const ros::TimerEvent& event)
  {
    ROS_INFO("Task B");
  }

  // --------------------------------------------------------------------------

  void taskCCb(const ros::TimerEvent& event)
  {
    ROS_INFO("Task C");
  }

};

// ============================================================================
// ============================================================================

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "multitask");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  Multitask node(nhtopics, nhparams);
  ros::spin();
  return 0;
}
