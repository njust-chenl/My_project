#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#define PI 3.1415926
typedef struct{
	int32_t tarV;
	int32_t tarPos;
	int32_t actV;
	int32_t actPos;
	int32_t posInc;
	int32_t odom, last_odom, incre_odom;

	uint8_t ID;
	uint8_t ctrlMode;//控制模式:1-速度模式;2-位置模式(相对运动);3-位置模式(绝对运动)
}servoMotor;

typedef struct{
	servoMotor motor; 
	float tarAngle,actAngle;
	bool hasNewAngleCmd;
	bool setCmdStatus;
	bool isCurrentUpdated;
	bool isPosUpdated;
	uint8_t mode;//1-相对运动；2-绝对运动
}ARM;

typedef struct
{
    uint8_t mode, status;
    float tarV, tarW, tarACC, tarDEC;
    int32_t tarPos;
    servoMotor left_wheel ,right_wheel;
    
} ChassisMode;

class my_odom {
public:
  my_odom();
  ~my_odom(){};
  void Odom_Release(ChassisMode my_agv);
  void Para_Init();

private:
  ros::NodeHandle n;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  double x;
  double y;
  double th;
  ros::Time current_time, last_time;
  //std::string dev;
  double ODOM_FAC;
  double Wheel_Track, Wheel_Diameter,angle_FAC,move_FAC;
  int Reduction_Ratio, Encoder_Accuracy;
};
class my_imu 
{
public:
    my_imu();
    ~my_imu(){};
    void IMU_Release(uint16_t *q);
    
private:
    ros::NodeHandle n;
    ros::Publisher imu_pub;
};