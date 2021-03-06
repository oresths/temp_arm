#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ultra_complex_communication/trilizaAction.h>
#include <stdlib.h>

#define boardLength  9

class player
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<ultra_complex_communication::trilizaAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  ultra_complex_communication::trilizaFeedback feedback_;
  ultra_complex_communication::trilizaResult result_;

public:

  player(std::string name) :
    as_(nh_, name, boost::bind(&player::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~player(void)
  {
  }

  void executeCB(const ultra_complex_communication::trilizaGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
  
    int choice;
    
    if (goal->gameOver) {
      ros::shutdown();
      ROS_INFO("p2 terminating...");
    }

    choice=rand()%9;
    feedback_.position = choice;
    as_.publishFeedback(feedback_);
    while ( goal->tbl[choice] != ' ' ) {
      choice=rand()%9;
      feedback_.position = choice;
      as_.publishFeedback(feedback_);
    }
    
    ROS_INFO("p2 choice: %d", choice);
    
    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    }
    
    // this sleep is not necessary, computed at 1 Hz for demonstration purposes
    r.sleep();

    if(success)
    {
      result_.tbl = goal->tbl;
      result_.tbl[choice] = 'X';
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "p2");
//ros::Time::init();
//ros::Duration(3).sleep();
  player play(ros::this_node::getName());
  ROS_INFO("p2 main");
  ros::spin();

  return 0;
}
