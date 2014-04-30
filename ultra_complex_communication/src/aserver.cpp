#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//It isn't written in tutorial, but for messages headers to be always ->
// -> available add_dependencies(*) must be added
#include <ultra_complex_communication/trilizaAction.h>

#define boardLength  9

class server
{
protected:

  std::string nodeName;
  char board[boardLength];
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ultra_complex_communication::trilizaAction> ac1;
  actionlib::SimpleActionClient<ultra_complex_communication::trilizaAction> ac2;
  ultra_complex_communication::trilizaGoal goal;

public:

  server(std::string name) :
    nodeName(name),
    ac1("aplayer1_l", true),  //node names here must agree with *.launch. Maybe ->
    ac2("aplayer2_l", true)   //->passed as an argument in *.launch ?
  {
    
    ROS_INFO("Waiting for action servers to start.");
    // wait for the action server to start
    ac1.waitForServer(); //will wait for infinite time
    ac2.waitForServer(); //will wait for infinite time

    ROS_INFO("Action servers started.");

    for (int i=0; i<boardLength; i++)  {
      board[i] = ' ';
    }
    
    ultra_complex_communication::trilizaResult result;
    bool gameEnded = false;
    goal.gameOver = false;
    int turn = 1;
    while (!gameEnded) {
      
      goal.tbl.clear();
      for (int i=0; i<boardLength; i++)  {
        //goal.tbl[i]=board[i] would not work because goal declared as "uint8[] tbl"
        //->(we push back so goal can reallocate needed memory)
        goal.tbl.push_back(board[i]);
      }
      
      bool finished_before_timeout;
      if (turn==1) {  //a switch-case control structure would not work here because we
                      //-> must initialize actionlib::SimpleClientGoalState state
        ac1.sendGoal(goal);
        
        //wait for the action to return
        finished_before_timeout = ac1.waitForResult(ros::Duration(2.0));
        result = *(ac1.getResult());
      
        for (int i=0; i<boardLength; i++)  {
          board[i] = result.tbl[i];
          //board[i] = ac1.getResult()->tbl[i];
        }
        
        actionlib::SimpleClientGoalState state = ac1.getState();
        ROS_INFO("p1 Action finished: %s",state.toString().c_str());
        
        turn = 2;
      }
      else if (turn==2) {
        ac2.sendGoal(goal);
        
        finished_before_timeout = ac2.waitForResult(ros::Duration(2.0));
        result = *(ac2.getResult());
      
        for (int i=0; i<boardLength; i++)  {
          board[i] = result.tbl[i];
          //board[i] = ac2.getResult()->tbl[i];
        }
        
        actionlib::SimpleClientGoalState state = ac2.getState();
        ROS_INFO("p2 Action finished: %s",state.toString().c_str());
        
        turn = 1;
      }
      

      if (finished_before_timeout)
      {        
        std::string str(board,boardLength); //boardLength needed because board not null-terminat
        ROS_INFO("I heard: [%s]",str.c_str());
        
        //finds 3 symbols in a row, not normal triliza
        int sumO = 0;
        int sumX = 0;
        int sumFilled = 0;
        for (int i=0; i<boardLength; i++)  {
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
          gameEnded = true;
        }
        else if (sumX>=3) {
          ROS_INFO("Player2 wins");
          gameEnded = true;
        }
        else if (sumFilled>=9) {
          ROS_INFO("Both losers");
          gameEnded = true;
        }
      }
      else
        ROS_INFO("Action did not finish before the time out.");
    }
    
    if (gameEnded)
    {
      ROS_INFO("Game over");
      goal.gameOver = true;
      ac1.sendGoal(goal);
      ac2.sendGoal(goal);
    }
    
  }

};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "control");

  server serve(ros::this_node::getName());

  return 0; //exit
}

