#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include "my_serial/arm.h"
#include "my_serial/incidence.h"


class M_serial : public serial::Serial
{
public:
    M_serial();
    ~M_serial(){};
    void arm_robot_Callback(const my_serial::arm::ConstPtr &angle);
     uint8_t Data_Decode(uint8_t* msg);
    void Para_Init();
    //AGV_MODEL my_agv;

private:
    ros::NodeHandle n;
    //ros::Subscriber vel_sub;
    ros::Subscriber arm_sub;
    ros::Publisher pub;
    std::string dev;
    int baudrate;
    // my_odom odom;
    // my_undm undm;
    // my_imu imu;
};
