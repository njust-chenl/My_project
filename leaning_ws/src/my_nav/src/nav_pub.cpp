#include "nav.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_navigation_goals");
  my_nav nav;
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
  //  //等待回应
  //   ac.waitForResult();

  //   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //     ROS_INFO("Hooray, the base moved 1 meter forward");
  //   else
  //     ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
my_nav::my_nav() {
  move = new MoveBaseClient("move_base", true);
  //等待move_base Server启动
  ROS_INFO("Waiting for action server to start.");
  move->waitForServer();
  ROS_INFO("Action server started, sending goal.");
}
bool my_nav::Goal_Pub(double x, double y, double w) {
  //启动完成后，发送目标点信息。
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  ROS_INFO("Sending goal");
  // move->sendGoal(goal,
  //               MoveBaseClient::SimpleDoneCallback(),
  //               MoveBaseClient::SimpleActiveCallback(),
  //               MoveBaseClient::SimpleFeedbackCallback());
  move->sendGoal(goal, MoveBaseClient::SimpleDoneCallback(),
                 MoveBaseClient::SimpleActiveCallback(),
                 boost::bind(&my_nav::Nav_Pos, this, _1));
}

void my_nav::Nav_Pos(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {
  cur_pos.pose.position.x = feedback->base_position.pose.position.x;
  cur_pos.pose.position.y = feedback->base_position.pose.position.y;
  cur_pos.pose.orientation.w = feedback->base_position.pose.orientation.w;
  ROS_INFO("Action Recive Feedback SUCCESS");
}
