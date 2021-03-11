#include "sensor.h"

my_odom::my_odom() {
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  this->Para_Init();
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  x=0;
  y=0;
  th=0;
}
void my_odom::Para_Init() {
 // n.param<double>("odom_fac", ODOM_FAC, 12);

  n.param<double>("robot_control/Wheel_Diameter", Wheel_Diameter, 0.21);
  n.param<double>("robot_control/Wheel_Track", Wheel_Track, 0.59);
  n.param<int>("robot_control/Reduction_Ratio", Reduction_Ratio,30);
  n.param<int>("robot_control/Encoder_Accuracy", Encoder_Accuracy, 10000);
  n.param<double>("robot_control/angle_FAC", angle_FAC, 1);
  n.param<double>("robot_control/move_FAC", move_FAC, 1);
  //n.param<int>("Velocity_Factor", Velocity_Factor, 240);
  ODOM_FAC = (PI * Wheel_Diameter) / (Reduction_Ratio * Encoder_Accuracy)*move_FAC;//转一圈的脉冲对应的圆周长度

  //VEL_FAC = (PI * Wheel_Diameter) / Reduction_Ratio;


  std::cout << "Wheel_Diameter is " << Wheel_Diameter << std::endl;
  std::cout << "wheel_track is " << Wheel_Track << std::endl;
  std::cout << "Reduction_Ratio is " << Reduction_Ratio << std::endl;
  std::cout << "Encoder_Accuracy is " << Encoder_Accuracy << std::endl;
  //std::cout << "Velocity_Factor is " << Velocity_Factor << std::endl;
  std::cout << "ODOM_FAC is " << ODOM_FAC << std::endl;
   std::cout << "angle_FAC is " << angle_FAC << std::endl;
   std::cout << "move_FAC is " << move_FAC << std::endl;
 // std::cout << "VEL_FAC is " << VEL_FAC << std::endl;
}

void my_odom::Odom_Release(ChassisMode my_agv)
{

    int32_t left_odom = my_agv.left_wheel.incre_odom;//需要处理left轮子对应的脉冲
    int32_t right_odom = my_agv.right_wheel.incre_odom;//需要处理right轮子对应的脉冲
    // int16_t left_vel = my_agv.left_wheel.actV;//处理left轮子的速度，转速现在是转速
    // int16_t right_vel = my_agv.right_wheel.actV;//处理right轮子对应的速度
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;
    // compute odometry in a typical way given the velocities of the robot
    double delta_x = (right_odom + left_odom) * ODOM_FAC * cos(th) / 2.0;
    double delta_y = (right_odom +left_odom) * ODOM_FAC * sin(th) / 2.0;
    double delta_th = (right_odom - left_odom) * ODOM_FAC / Wheel_Track*angle_FAC;
    double vx = (right_odom + left_odom) * ODOM_FAC / 2.0/dt;
    double vy = 0;
    double vth = (right_odom - left_odom) * ODOM_FAC / Wheel_Track / dt;
    x += delta_x;
    y += delta_y;
    th += delta_th;
    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    // send the transform
    odom_broadcaster.sendTransform(odom_trans);
    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    // publish the message
    odom_pub.publish(odom);
}

my_imu::my_imu() {
  imu_pub = n.advertise<sensor_msgs::Imu>("IMU_data", 20);
}


void my_imu::IMU_Release(uint16_t *q) {
  sensor_msgs::Imu imu_data;
  imu_data.header.stamp = ros::Time::now();
  imu_data.header.frame_id = "imu";
  imu_data.orientation.x = q[0];
  imu_data.orientation.y = q[1];
  imu_data.orientation.z = q[2];
  imu_data.orientation.w = q[3];
  imu_pub.publish(imu_data);
}

