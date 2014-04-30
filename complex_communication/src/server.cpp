#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "complex_communication/triliza.h"
#include "complex_communication/terminate.h"

//#include <sstream>
//#include <stdlib.h>
//#include <stdio.h>
//#include <string>

uint8_t board_length = 9;
char board[9] = {' ',' ',' ',' ',' ',' ',' ',' ',' '};
bool game_ended = false;
bool msg_received = false;

void chatterCallback(const complex_communication::triliza::ConstPtr& msg)
{
  for (int i=0; i<board_length; i++)
  {
    board[i] = msg->tbl[i];
  }

  std::string str(board);
  ROS_INFO("I heard: [%s]",str.c_str());
  
  //finds 3 symbols in a row, not normal triliza
  int sumO = 0;
  int sumX = 0;
  int sumFilled = 0;
  for (int i=0; i<board_length; i++)  {
    if (board[i] == 'O')    {
      sumO++;
      if (sumO>=3) break;
      sumX = 0;
      sumFilled++;
    }
    else if (board[i] == 'X')    {
      sumX++;
      if (sumX>=3) break;
      sumO = 0;
      sumFilled++;
    }
    else if (board[i] == ' ')   {
      sumO = 0;
      sumX = 0;
    }
  }
  
  if (sumO>=3){
    ROS_INFO("Player1 wins");
    game_ended = true;
  }
  else if (sumX>=3) {
    ROS_INFO("Player2 wins");
    game_ended = true;
  }
  else if (sumFilled>=9) {
    ROS_INFO("Both losers");
    game_ended = true;
  }
  
  msg_received = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");

  ros::NodeHandle n;

  ros::Publisher p1_pub = n.advertise<complex_communication::triliza>("/task2/p1_ctrl", 1000);
  ros::Publisher p2_pub = n.advertise<complex_communication::triliza>("/task2/p2_ctrl", 1000);
  
  ros::Publisher pub_term = n.advertise<complex_communication::terminate>("/task2/termination", 1000);

  ros::Subscriber sub = n.subscribe("/task2/board_check", 1000, chatterCallback);

  ros::Rate loop_rate(2);

  short int turn = 1;
  ros::Duration(0.5).sleep();  //gives time for subscribers to subscribe
  complex_communication::triliza msg;
  for (int i=0; i<board_length; i++)  {
    msg.tbl[i] = board[i];
  }
  p1_pub.publish(msg);
  turn = 2;
  while (ros::ok())
  {
    if (game_ended) { //nodes cannot shutdown if are executing a callback
      complex_communication::terminate msg;
      msg.game_ends = true;
      pub_term.publish(msg);  //every subcriber get this message
      ros::shutdown();
    }
    else  {
      complex_communication::triliza msg;
    
      if (msg_received) {
        for (int i=0; i<board_length; i++)  {
          msg.tbl[i] = board[i];
        }
      
        switch (turn) {
        case 1:
          p1_pub.publish(msg);
          turn = 2;
          break;
        case 2:
          p2_pub.publish(msg);
          turn = 1;
          break;
        }
        msg_received = false;
      }

      ROS_INFO("serv");
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    
  }

  return 0;
}

