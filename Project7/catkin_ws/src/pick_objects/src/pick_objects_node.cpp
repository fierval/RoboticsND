#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>
#include <vector>
#include <tuple>
using namespace std;

void SetGoal(move_base_msgs::MoveBaseGoal &goal, float x, float y, float w)
{
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach (pickup)
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;
}

void PrintLoc(string &name, float x, float y, float w)
{
  ROS_INFO("%s location: x: %f, y: %f, w: %f", name.c_str(), x, y, w);
}

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pickup_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";

  vector<string> goalNames{"Pickup", "Dropoff"};
  vector<tuple<float, float, float>> pos{make_tuple(3.0, 4.0, 1.0), make_tuple(-5.0, 1.0, 1.0)};

  for (int i = 0; i < 2; i++)
  {
    float x, y, w;
    tie(x, y, w) = pos[i];

    SetGoal(goal, x, y, w);

    // Send the goal position and orientation for the robot to reach
    string name(goalNames[i]);
    name[0] = tolower(name[0]);

    ROS_INFO("Sending %s", name.c_str());
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      PrintLoc(goalNames[i], x, y, w);
    }
    else
    {
      ROS_INFO("Failed to reach %s location", goalNames[i].c_str());
      return 1;
    }

    ROS_INFO("Waiting 5 sec...");
    ros::Duration(5.0).sleep();
  }

  return 0;
}