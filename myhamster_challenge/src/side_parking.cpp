#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "side_parking");
  ros::NodeHandle n;
  ros::Publisher task_finish_pub = n.advertise<std_msgs::UInt8>("task_finished", 1);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // fifth goal
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.z = 0.707;
  goal.target_pose.pose.orientation.w = 0.707;


  ROS_INFO("Sending Fifth Goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Fifth Goal Reached!");
  else
    ROS_INFO("Failed");


  // sixth goal
  goal.target_pose.pose.position.x = 2.4;
  goal.target_pose.pose.position.y = 0.25;
  goal.target_pose.pose.orientation.z = 0.707;
  goal.target_pose.pose.orientation.w = 0.707;

  ROS_INFO("Sending Sixth Goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Sixth Goal Reached!");
  else
    ROS_INFO("Failed");

  ros::Duration(5).sleep();

  // sixth goal
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 1.15;
  goal.target_pose.pose.orientation.z = 0.9854497;
  goal.target_pose.pose.orientation.w = 0.1699671;

  ROS_INFO("Sending Sixth Goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Sixth Goal Reached!");
  else
    ROS_INFO("Failed");



  std_msgs::UInt8 msg;
  msg.data = 1;
  task_finish_pub.publish(msg);

  return 0;
}