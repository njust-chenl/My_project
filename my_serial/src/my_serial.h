#include "ros/ros.h"
#include"std_msgs/UInt8MultiArray.h"
#include <serial/serial.h>


class my_serial : public serial::Serial
{
public:
    my_serial();
    ~my_serial(){};
    //void vel_Callback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    uint8_t Data_Handing(uint8_t *msg);
    uint8_t Joy_Data_Handing(uint8_t *msg);
    void Para_Init();
    ros::Publisher pub;

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    
    std::string dev;
    int baudrate;
    

};
