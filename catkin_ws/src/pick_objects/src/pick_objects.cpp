#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  ros::init(argc, argv, "zoneInfo");

  ros::NodeHandle n;

  ros::Publisher pick_pub = n.advertise<std_msgs::String>("/posInfo", 10);

  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -3.38;
  goal.target_pose.pose.position.y = -1.8;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending 1st goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Objects are picked up");
    std_msgs::String pick_msg;
    pick_msg.data = "pickup";
    pick_pub.publish(pick_msg);
    sleep(5);

    goal.target_pose.pose.position.x = -0.053;
    goal.target_pose.pose.position.y = -3.36;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending 2nd goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Objects are dropped up");
      std_msgs::String drop_msg;
      drop_msg.data = "dropoff";
      pick_pub.publish(drop_msg);}
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");}
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  

  return 0;
}
