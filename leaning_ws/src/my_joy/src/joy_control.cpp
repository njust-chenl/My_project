
#include "joy_control.h"

using namespace std;
float linear_max = 0.18;
float angular_max = 0.8;
Teleop::Teleop()
{
    pub = n.advertise<geometry_msgs::Twist>("/my_vel", 1);
    sub = n.subscribe<sensor_msgs::Joy>("/joy", 1, &Teleop::callback, this);
    this->Para_Init();
}

void Teleop::callback(const sensor_msgs::Joy::ConstPtr &Joy)
{
    static bool flag=0;
    if (Joy->axes[2] == 1&&Joy->axes[5]==1)
        flag=1;
    
    v.linear.x = (-Joy->axes[2] + 1) * linear_max;
    if(Joy->axes[2]==1.0)
        v.linear.x = (Joy->axes[5] - 1) * linear_max;
    v.angular.z = Joy->axes[0] * angular_max;
    if (Joy->buttons[KEY_RB])
    {
        if (Joy->buttons[KEY_Y])
            linear_max += vlinear;
        if (Joy->buttons[KEY_B])
            angular_max -= vangular;
        if (Joy->buttons[KEY_A])
            linear_max -= vlinear;
        if (Joy->buttons[KEY_X])
            angular_max += vangular;
    }
    if (flag==0||Joy->buttons[KEY_LB])
    {
        v.linear.x = 0;
        v.angular.z = 0;
    }

    pub.publish(v);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_control");
    Teleop telelog;
    ros::spin();
    return 0;
}
void Teleop::Para_Init()
{
    n.param<double>("joy_control/vel_linear", vlinear, 0.01);
    n.param<double>("joy_control/vel_angular", vangular, 0.01);
    std::cout << "Linear is " << vlinear << std::endl;
    std::cout << "Angular is " << vangular << std::endl;
}
