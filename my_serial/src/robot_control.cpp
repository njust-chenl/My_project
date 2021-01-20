#include "robot_control.h"

using namespace std;
bool vel_flag = false;
int robot_Lrpm = 0, robot_Rrpm = 0,
    direction = 0; //(direction=0 停止，direction=1
//原地转，direction=11前进，direction=12 后退，direction=21
//左转，direction=22右转)button按下对应0x20,松开对应0x00;

int main(int argc, char **argv) {

  ros::init(argc, argv, "robot_control");

  //创建一个serial类
  MY_serial sp;

  ros::Rate loop_rate(100);
  while (ros::ok()) {

    sp.getPosandVel(); //获取左you轮的位置里程计
    // usleep(10000);
    ros::spinOnce();
    size_t n1 = sp.available();
    if (n1) {
      uint8_t buffer2[8];
      //读出数据
      n1 = sp.read(buffer2, n1);
    }
    loop_rate.sleep();
  }

  sp.close();

  return 0;
}

void MY_serial::Para_Init() {
  nh.param<int>("robot_control/Baud_Rate", baudrate, 115200);
  nh.param<std::string>("robot_control/serial_dev", dev, "/dev/ttyS0");
  std::cout << "Port is " << dev << std::endl;
  std::cout << "Baud Rate is " << baudrate << std::endl;

  // 01 15 00 00 00 00 01 15
}
MY_serial::MY_serial() {
  vel_sub = nh.subscribe("/cmd_vel", 1, &MY_serial::vel_robot_Callback,this); //当导航包发布cmd_vel的时候，遥感就不发布数据，
  sub = nh.subscribe("/remote_control", 1, &MY_serial::remote_control_Callback,this);

  arm_sub = nh.subscribe("/arm_vel", 1, &MY_serial::arm_robot_Callback, this);
  this->Para_Init();
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  //设置要打开的串口名称
  this->setPort(dev);
  //设置串口通信的波特率
  this->setBaudrate(baudrate);
  //串口设置timeout
  this->setTimeout(to);

  try {
    //打开串口
    this->open();
  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port.");
  }

  //判断串口是否打开成功
  if (this->isOpen()) {
    ROS_INFO_STREAM("/dev/ttyS0 is opened.");
  }
}
void MY_serial::remote_control_Callback(
    const std_msgs::UInt8MultiArray::ConstPtr &msg) {
  int XX_value = 0, YY_value = 0, Button_value = 0, ZZ_value = 0;
  // x轴左侧角度范围(0x0020,0x01ff）右侧角度范围(0x0201,0x03e0),0x0200是停止位。y，z轴一样,
  // x:前进方向高速2000r/min（0x0020=32~0x00c0=192）中速1000r/min（
  // 0x00c1=193~0x0160=352）低速500r/min（0x0161=353~0x01ff=511)
  // x:后退方向低速500r/min（0x0201=513~0x0380=512+160+160）中速1000r/min（0x0381=512+160+161~
  // 0x03e0=512+480）
  uint8_t ch = 0;
  float ang;
  vel_flag = true;
  if (vel_flag) {
    for (size_t i = 1; i < 8; i++) {
      ch += msg->data[i];
    }
    // ROS_INFO("ch =,,,,,0x%x,    msg[8]=,,,,,,,,0x%x",ch,msg[8]);

    if (msg->data[0] == 0xff && msg->data[8] == ch) {

      ROS_INFO("receive data");
      YY_value = ((int)msg->data[1]) * 256 + ((int)msg->data[2]);
      XX_value = ((int)msg->data[3]) * 256 + ((int)msg->data[4]);
      ZZ_value = ((int)msg->data[5]) * 256 + ((int)msg->data[6]);
      Button_value = (int)msg->data[7];

      ROS_INFO("YY_value=%d  ,XX_value = %d ,ZZ_value = %d ,Button_value = %d",
               YY_value, XX_value, ZZ_value, Button_value);
    }
    if (992 > ZZ_value > 513 || 32 < ZZ_value < 511) {
      ang =-90 * (ZZ_value - 512) /480; //将遥感的数据转化为的角度,直接通过键盘输入即可。摆臂也可通过键盘的输入
    }

    // yuandizhuan
    if (YY_value == 0x0200) {
      if ((XX_value >= 0x0020) && (XX_value < 0x00c0)) {
        robot_Lrpm = robot_Rrpm = 2000;
        direction = 11;
      } else if ((XX_value >= 0x00C1) && (XX_value < 0x0160)) {
        robot_Lrpm = robot_Rrpm = 1000;
        direction = 11;
      } else if ((XX_value >= 0x0161) && (XX_value < 0x01ff)) {
        robot_Lrpm = robot_Rrpm = 500;
        direction = 11;
      } else if (XX_value == 0x0200) {
        robot_Lrpm = robot_Rrpm = 0;
        direction = 0;
      } else if ((XX_value >= 0x0201) && (XX_value < 0x0380)) {
        robot_Lrpm = robot_Rrpm = -500;
        direction = 12;
      } else {
        robot_Lrpm = robot_Rrpm = -1000;
        direction = 12;
      }
    } else if ((YY_value >= 0x0020) &&
               (YY_value < 0x01ff)) //左侧手柄，开始转向向左转，
    {
      robot_Lrpm = -1000;
      robot_Rrpm = 1000;
    } else if ((YY_value >= 0x0201) && (YY_value < 0x0380)) //向右转
    {
      robot_Lrpm = 1000;
      robot_Rrpm = -1000;
    }

    if (Button_value == 32) {
      robot_Lrpm = robot_Rrpm = 0;
      direction = 0;
    }

    ROS_INFO("robot_Lrpm = %d,  robot_Rrpm= %d,  direction = %d ", robot_Lrpm,
             robot_Rrpm, direction);

    this->write(motor_En(MOTOR_RIGHT), 8);
    usleep(5000);
    this->write(motor_Move(MOTOR_RIGHT, (int32_t)(10 * robot_Rrpm)), 8);
    usleep(5000);

    this->write(motor_En(MOTOR_LEFT), 8);
    usleep(5000);
    this->write(motor_Move(MOTOR_LEFT, (int32_t)(-10 * robot_Lrpm)), 8);
    usleep(5000);
  }
}

void MY_serial::vel_robot_Callback(
    const geometry_msgs::Twist::ConstPtr &cmd_vel) {
 
  // uint8_t a[8];
  static float lin_vel_min = 0, ang_vel_min = 0;
  static float ang_to_lin =
      0; //将角速度转换为左轮右轮的速度，叠加在原先的线速度上
  static float ML_rpm, MR_rpm; //左右电机转速
  if (!vel_flag) {

    lin_vel_min = cmd_vel->linear.x * 60; //单位转换，将线速度m/s转换为m/min
    ang_vel_min = cmd_vel->angular.z * 60; // rad/s --> rad/min
    ang_to_lin = ang_vel_min * WheelDistance / 2;
    ML_rpm = ChassisReductionRatio * (lin_vel_min - ang_to_lin) / PI /
             WheelDiameter1;
    MR_rpm = ChassisReductionRatio * (lin_vel_min + ang_to_lin) / PI /
             WheelDiameter1;
    ROS_INFO("robot_Lrpmcmd= %f,  robot_Rrpmcmd= %f", ML_rpm, MR_rpm);
    this->write(motor_En(MOTOR_RIGHT), 8);

    usleep(5000);
    this->write(motor_Move(MOTOR_RIGHT, (int32_t)(10 * MR_rpm)), 8);
    usleep(5000);
    // memcpy(&a,motor_Move(MOTOR_RIGHT, (int32_t)(10 * MR_rpm)),8);
    // for (size_t i = 0; i < 8; i++)
    // {
    //   ROS_INFO("%x",a[i]);
    // }

    ROS_INFO("xieruchenggong");

    this->write(motor_En(MOTOR_LEFT), 8);
    usleep(5000);
    this->write(motor_Move(MOTOR_LEFT, (int32_t)(-10 * ML_rpm)), 8);
    usleep(5000);
  }
  vel_flag = false;
}
// 01 05 00 5f 23 45 22 1f 位置
// 01 3f 00 00 03 e9 02 d6 速度
//处理buffer里面的数据
// uint8_t my_serial::Data_Decode(uint8_t *msg) {

//   memcpy(&my_chassis.left_wheel.incre_odom);

// }
void MY_serial::arm_robot_Callback(const my_serial::arm::ConstPtr &angle) {
  float ratioAngleToPulse = ArmMotorPulsePerTurn * ArmReductionRatio /360;
  uint8_t id=0, mode=0;
  int tarAngle=angle->data;
  
  switch (angle->addr) {//对应两个按键 fb 
  case 0:
    id = MOTOR_F;
    break;
  case 1:
    id = MOTOR_B;
    break;
  }
  switch (angle->mode) {//mode 有3个方式对应三个按键 数值：1,2,3  1 2 3  
  case 1:
    mode = PRIM_MoveRel; //相对运动
    break;
  case 2:
    mode = PRIM_MoveAbs; //绝对运动
    break;
  case 3:       //回零操作
    mode =PRIM_Homming;
    break;
  }
  ROS_INFO("mubiaojiaodu:%d id: %d mode: %d",tarAngle,id,mode);
 this->write(motor_En(id), 8);
  usleep(5000);
 if(mode!=PRIM_Homming){
    int32_t tarPos = tarAngle * ratioAngleToPulse;
    this->write(setCmd(mode, id, tarPos), 8);
      usleep(5000);
    this->write(setCmd(PRIM_Go, id, 0), 8);
     usleep(5000);
 }else {
    this->write(setCmd(mode, id, 0), 8);
     usleep(5000);
 }

}

uint8_t *setCmd(uint8_t cmdType, uint8_t addr, int32_t data) {
  static uint8_t msg[8] = {0};
  memset(msg, 0, 8);
  msg[0] = addr;
  msg[1] = cmdType;
  if (data != 0) {
    msg[5] = data & 0xFF;
    data >>= 8;
    msg[4] = data & 0xFF;
    data >>= 8;
    msg[3] = data & 0xFF;
    data >>= 8;
    msg[2] = data & 0xFF;
  }
  msg[6] = msg[0] ^ msg[2] ^ msg[4];
  msg[7] = msg[1] ^ msg[3] ^ msg[5];
  return msg;
}
uint8_t *motor_En(uint8_t motor_id) {
  return setCmd(MOTOR_ENABLE, motor_id, 0);
}
uint8_t *motor_Move(uint8_t motor_id, int32_t rpm) {
  return setCmd(VEL_CONTROL, motor_id, rpm);
  // ROS_INFO("tx_buf[5] = 0x%x,tx_buf[4] =0x%x ", tx_buf[5], tx_buf[4]);
}

bool formatcheck(uint8_t *cmd, uint8_t cmdtype, uint8_t addr) {
  if (cmd[0] != addr || cmd[1] != cmdtype)
    return false;
  if ((cmd[0] ^ cmd[2] ^ cmd[4]) != cmd[6] ||
      (cmd[1] ^ cmd[3] ^ cmd[5]) != cmd[7])
    return false;
  return true;
}

/*

TODO :  获得电机运动的距离和运行的速度
输入 ：电机的地址
返回   成功1
    失败 0

备注

*/

void MY_serial::getPosandVel() {
  uint8_t buffer1[8] = {0};
  // uint8_t buffer[8];
  // vector<uint8_t>   buffer(n);
  uint32_t left_pulse_current = 0, right_pulse_current = 0;
  static uint32_t left_pulse_last = 0, right_pulse_last = 0;
  //ROS_INFO("start lichengji ");
  this->write(setCmd(PRIM_GetActualPos, MOTOR_LEFT, 0), 8); //左轮实际位置
  usleep(5000);
  size_t n;
  n = this->available();
  uint8_t *buffer = new uint8_t[n]; //动态数组
  if (n) {
    //读出数据

    n = this->read(buffer, n);
    // for (size_t i = 0; i < n; i++) {
    //   ROS_INFO("%x", buffer[i]);
    // }

    if (formatcheck(buffer, PRIM_GetActualPos, MOTOR_LEFT) != 0) {
      for (size_t i = 0; i < 8; i++) {
        buffer1[8 - 1 - i] = buffer[i];
      }

      memcpy(&left_pulse_current, &buffer1[2], 4);
    }
  } // else ROS_INFO("DUQUDESHUJUBUGOU");
  for (size_t i = 0; i < n; i++) {
    buffer[i] = 0;
  }

  this->write(setCmd(PRIM_GetActualPos, MOTOR_RIGHT, 0), 8);
  usleep(5000);
  n = this->available();
  if (n) {
    //读出数据
    n = this->read(buffer, n);
    if (formatcheck(buffer, PRIM_GetActualPos, MOTOR_RIGHT) != 0) {
      for (size_t i = 0; i < 8; i++) {
        buffer1[8 - 1 - i] = buffer[i];
      }

      memcpy(&right_pulse_current, &buffer1[2], 4);
    }
  } // else
  // ROS_INFO("DUQUDESHUJUBUGOU");
  if (left_pulse_last == 0 || right_pulse_last == 0) {
    right_pulse_last = right_pulse_current;
    left_pulse_last = left_pulse_current;
    // ROS_INFO("485't get pulse");
  }
 // ROS_INFO("PULSE :%d,%d", left_pulse_current, right_pulse_current);
  my_chassis.left_wheel.incre_odom = left_pulse_current - left_pulse_last;
  my_chassis.right_wheel.incre_odom = right_pulse_current - right_pulse_last;
  odom.Odom_Release(my_chassis);

  delete[] buffer;
}
