#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "complex_communication/triliza.h"
#include "complex_communication/terminate.h"
#include <stdlib.h>

uint8_t board_length = 9;
char board[9];
bool msg_received = false;

void chatterCallback(const complex_communication::triliza::ConstPtr& msg)
{
  int k;
  ROS_INFO("p1 heard");
  
  for (int i=0; i<board_length; i++)
  {
    board[i] = msg->tbl[i];
  }
  
  k=rand()%9;
  while ( board[k] != ' ' ) {
    k=rand()%9;
  }
  
  ROS_INFO("P1 heard: [%d]", k);
  
  board[k] = 'O';
  msg_received = true;
}

void termCallback(const complex_communication::terminate::ConstPtr& msg)
{
  if (msg->game_ends)
    ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "p1");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<complex_communication::triliza>("/task2/board_check", 1000);

  ros::Subscriber sub = n.subscribe("/task2/p1_ctrl", 1000, chatterCallback);
  
  ros::Subscriber sub_term = n.subscribe("/task2/termination", 1000, termCallback);

  ros::Rate loop_rate(2);

  while (ros::ok())
  {
    complex_communication::triliza msg;
    
    loop_rate.sleep();

    ros::spinOnce();
    
    if (msg_received) {
      for (int i=0; i<board_length; i++)
        msg.tbl[i] = board[i];
      pub.publish(msg);
      msg_received = false;
    }

    ROS_INFO("p1");
  }

  return 0;
}

