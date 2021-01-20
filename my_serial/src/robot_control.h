#include "sensor.h"
#include "std_msgs/UInt8MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include "my_serial/arm.h"

#define ArmMotorPulsePerTurn 10000
#define ArmReductionRatio ((float)72)
#define ChassisWidth ((float)0.51)
#define ChassisReductionRatio ((float)30)
#define WheelDiameter1 ((float)0.147)
#define WheelDistance ((float)0.51)
#define MOTOR_LEFT 0x01
#define MOTOR_RIGHT 0x02
#define MOTOR_F 0x03
#define MOTOR_B 0x04

#define HIGH_RPM 2000
#define MIDDLE_RPM 1000
#define LOW_RPM 500

typedef enum {
  PRIM_MoveAbs = 0x28, //电机绝对运动指令
  PRIM_MoveRel = 0x29, //电机相对运动指令
  PRIM_Go = 0x32,
  PRIM_SeekLimit = 0x47,
  PRIM_EncRst = 0x50,
  PRIM_Homming = 0x48,
  PRIM_GetStatus = 0x64,
  VEL_CONTROL = 0x6f,
  MOTOR_ENABLE = 0x15,
  MOTOR_DISABLE = 0x16,
  PRIM_GetActualPos = 0x05,  //获取电机位置实际值  单位脉冲
  PRIM_GetActVelocity = 0x3f //获取电机实际速度单位是0.1rpm
} DriverCmdType;

uint8_t *setCmd(uint8_t cmdType, uint8_t addr, int32_t data);
uint8_t *motor_En(uint8_t motor_id);
uint8_t *motor_Move(uint8_t motor_id, int rpm);
uint8_t *motor_dis();

class MY_serial : public serial::Serial
{
public:
  MY_serial();
  ~MY_serial(){};
  void remote_control_Callback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
  void vel_robot_Callback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
  void arm_robot_Callback(const my_serial::arm::ConstPtr &angle);
  void Para_Init();
  //  uint8_t Data_Decode(uint8_t* msg);
  void getPosandVel();
  ChassisMode my_chassis;
  my_odom odom;
 


private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber vel_sub;
   ros::Subscriber arm_sub;
  std::string dev;
  int baudrate;
};
bool motorArm_init(uint8_t addr);
void motorArm_ctrl(uint8_t angle, uint8_t addr, uint8_t mode);
void angleSet(int tarAngle, uint8_t id, uint8_t mode);
bool statusGet(uint8_t addr);
bool formatcheck(uint8_t *cmd, uint8_t cmdtype, uint8_t addr);
// bool formatcheck(uint8_t *cmd, uint8_t cmdtype, uint8_t addr);