 #include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>

#define KEY_A     0
#define KEY_B     1
#define KEY_X   2
#define KEY_Y    3
#define KEY_LB      4 
#define KEY_RB      5
class Teleop
{
public:
    Teleop();
    void Para_Init();
private:
    /* data */
    void callback(const sensor_msgs::Joy::ConstPtr& Joy);
    ros::NodeHandle n;
    ros::Subscriber sub ;
    ros::Publisher pub ;
    double vlinear,vangular;
    int axis_ang,axis_lin,ton;
    geometry_msgs::Twist v;
};