#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <boost/asio.hpp>         // 包含boost库函数
#include <boost/bind.hpp>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;
class my_nav {
public:
  my_nav();
  ~my_nav(){};
  void Para_Init();
  bool Goal_Pub(double x, double y, double w);
  void Nav_Pos(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

private:
  ros::NodeHandle n;
  MoveBaseClient *move;
  geometry_msgs::PoseStamped cur_pos;
};
