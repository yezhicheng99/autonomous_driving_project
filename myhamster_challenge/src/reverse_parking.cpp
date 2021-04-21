#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "reverse_parking");
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

  // first goal
  goal.target_pose.pose.position.x = -1.72;
  goal.target_pose.pose.position.y = -1.2;
  goal.target_pose.pose.orientation.z = 0.707;
  goal.target_pose.pose.orientation.w = 0.707;

  ROS_INFO("Sending First Goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("First Goal Reached!");
  else
    ROS_INFO("Failed");


//  second goal
  // goal.target_pose.pose.position.x = -1.72;
  // goal.target_pose.pose.position.y = -1.0;
  // goal.target_pose.pose.orientation.z = 0.707;
  // goal.target_pose.pose.orientation.w = 0.707;

  // ROS_INFO("Sending Second Goal");
  // ac.sendGoal(goal);

  // ac.waitForResult();

  // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   ROS_INFO("Second Goal Reached!");
  // else
  //   ROS_INFO("Failed");



  // third goal
  goal.target_pose.pose.position.x = -2.6;
  goal.target_pose.pose.position.y = -0.65;
  goal.target_pose.pose.orientation.z = 1;
  goal.target_pose.pose.orientation.w = 0;

  ROS_INFO("Sending Third Goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Third Goal Reached!");
  else
    ROS_INFO("Failed");



  // fourth goal
  goal.target_pose.pose.position.x = -1.72;
  goal.target_pose.pose.position.y = -1.2;
  goal.target_pose.pose.orientation.z = 0.707;
  goal.target_pose.pose.orientation.w = 0.707;

  ROS_INFO("Sending Fourth Goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Fourth Goal Reached!");
  else
    ROS_INFO("Failed");


  // fifth goal
  goal.target_pose.pose.position.x = -1.1;
  goal.target_pose.pose.position.y = -0.7;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;


  ROS_INFO("Sending Fifth Goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Fifth Goal Reached!");
  else
    ROS_INFO("Failed");


  std_msgs::UInt8 msg;
  msg.data = 1;
  task_finish_pub.publish(msg);

  return 0;
}