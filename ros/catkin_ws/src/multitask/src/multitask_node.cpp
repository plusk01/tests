/**
 * @file multitask_node.cpp
 * @brief Entry point for multitask runner
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 15 Oct 20
 */

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <std_msgs/Bool.h>

class Multitask
{
public:
  Multitask(const ros::NodeHandle nh, const ros::NodeHandle nhp)
  : nh_(nh), nhp_(nhp)
  {

    // Timers will be responsible for tasks A and B, which depend
    // on the 'heavy lifting' done in the main thread. To decouple
    // these timers from this 'heavy lifting', we create a dedicated
    // callback queue via a node handle. Anything created using this
    // node handle will use the specified callback queue.
    ros::NodeHandle nhQ(nh_);
    nhQ.setCallbackQueue(&task_queue_);

    // timer-based tasks (timers have no message queue)
    tim_taskA_ = nhQ.createTimer(ros::Duration(0.1), &Multitask::taskACb, this);
    tim_taskB_ = nhQ.createTimer(ros::Duration(0.1), &Multitask::taskBCb, this);

    // Tasks A and B are decoupled from the main thread 'heavy lifting',
    // but they could still be blocked by each other. To prevent them
    // from blocking each other (only themselves---see subscriber concurrency:
    //          https://stackoverflow.com/a/48544551/2392520),
    // we will service the task callback queue by a pool of threads.
    // We use an AsyncSpinner instead of MultiThreadedSpinner so that
    // we can still have a main loop in the spin() method.
    spinner_ = std::make_unique<ros::AsyncSpinner>(2, &task_queue_);
    // spinner_->start(); // note: don't start until we have data to process

    // Note: for preemption: https://answers.ros.org/question/250331/how-to-thread-a-specific-computationally-expensive-callback

    sub_in_ = nh_.subscribe("in", 1, &Multitask::inCb, this);
  }
  ~Multitask() = default;
  
  void spin()
  {
    ros::Rate r(5);
    while (ros::ok()) {

      if (processNewData) {
        // stop tasks B and C
        spinner_->stop();

        // do some heavy lifting
        ros::Duration(3).sleep();

        // allow downstream tasks to continue
        spinner_->start();

        processNewData = false;
      }

      // use single-threaded spinner to service one message from
      // the internal, global callback queue (the default one).
      // This callback queue only contains messages from inCb.
      // https://answers.ros.org/question/214629/multiple-callback-queues
      ros::spinOnce();

      // maintain a loop rate of (at most) r Hz
      r.sleep();
    }
  }

private:
  ros::NodeHandle nh_, nhp_;
  ros::CallbackQueue task_queue_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber sub_in_;
  ros::Timer tim_taskA_, tim_taskB_;

  bool processNewData = false; ///< set by incoming ros message

  void inCb(const std_msgs::BoolConstPtr& msg)
  {
    ROS_INFO("Got input");
    processNewData = true;
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

};

// ============================================================================
// ============================================================================

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "multitask");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  Multitask node(nhtopics, nhparams);
  node.spin();
  return 0;
}
