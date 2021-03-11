
#include "arm_control.h"

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_control");
  //创建一个serial类
  M_serial sp;
  
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    //获取缓冲区内的字节数
    uint8_t buffer1[8]={0x01,0x04,0x00,0x00,0x00,0x08,0xf1,0xcc};
    sp.write(buffer1,8);
    size_t n = sp.available();
    if (n) {
      uint8_t buffer[21];
      //读出数据
      n = sp.read(buffer, n);
    sp.Data_Decode(buffer);   
    }
    loop_rate.sleep();
    ros::spinOnce();
     if (n) {
      uint8_t *buffer1 = new uint8_t[n];
      //读出数据,
      n = sp.read(buffer1, n);
      delete []buffer1;
    }
  
  }
 
  sp.close();
  return 0;
}



void M_serial::Para_Init() {
  n.param<int>("Baud_Rate", baudrate, 9600);
  n.param<std::string>("dev", dev, "/dev/ttyS1");
  std::cout << "Port is " << dev << std::endl;
  std::cout << "Baud Rate is " << baudrate << std::endl;
}

uint8_t M_serial::Data_Decode(uint8_t *msg) {
  my_serial::incidence msg1;
  uint8_t buffer2[4];
  uint16_t x1=0,y1=0;
  if(msg[0]==0x01){
    for (size_t i = 0; i < 4; i++)
    {
      buffer2[3-i]=msg[3+i];
    }

    
    memcpy(&x1,&buffer2[2],2);
    memcpy(&y1,&buffer2[0],2);
    //ROS_INFO("x=%d,y=%d",x1,y1);
  }
  msg1.x=(x1*180/16000)-135;
  msg1.y=(y1*180/16000)-135;
  pub.publish(msg1);
}


void M_serial::arm_robot_Callback(const my_serial::arm::ConstPtr &angle) {
  //float ratioAngleToPulse = ArmMotorPulsePerTurn * ArmReductionRatio / 360;
  
  uint8_t control_data[8][8]={{0x01,0x05,0x00,0x00,0xFF,0x00,0x8C,0x3A},
                             {0x01,0x05,0x00,0x00,0x00,0x00,0xCD,0xCA},
                             {0x01,0x05,0x00,0x01,0xFF,0x00,0xDD,0xFA},
                             {0x01,0x05,0x00,0x01,0x00,0x00,0x9c,0x0A},
                             {0x01,0x05,0x00,0x02,0xff,0x00,0x2d,0xfa},
                             {0x01,0x05,0x00,0x02,0x00,0x00,0x6c,0x0a},
                             {0x01,0x05,0x00,0x03,0xff,0x00,0x7c,0x3A},
                             {0x01,0x05,0x00,0x03,0x00,0x00,0x3d,0xca}
                             };
  uint8_t control_msg[8]={0};                       
 // ROS_INFO("shoudaoshuju=%d",angle->data);

  switch (angle->data) { //对应两个按键 fb
  case 1:
    for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[3][i]; 
        //ROS_INFO("3deshihoushuchu:%d",control_msg[i]);
    }
    this->write(control_msg, 8);
    usleep(30000);
    for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[0][i];
       //ROS_INFO("0deshi:%d",control_msg[i]); 
    } 
    this->write(control_msg, 8);
    usleep(30000);
   
    break;
  case 2:
     for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[1][i]; 
    } 
    this->write(control_msg, 8);
    usleep(30000);
       for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[2][i]; 
    }
    this->write(control_msg, 8);
    usleep(30000);
    break;

  case 3:
    for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[7][i]; 
    } 
    this->write(control_msg, 8);
    usleep(30000);
       for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[4][i]; 
    }
    this->write(control_msg, 8);
    usleep(30000);

    break;

  case 4:
        for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[5][i]; 
    } 
    this->write(control_msg, 8);
    usleep(30000);
       for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[6][i]; 
    }
    this->write(control_msg, 8);
    usleep(30000);

    break;

  case 10:
     for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[3][i]; 
    }  
    this->write(control_msg, 8);
    usleep(30000);
         for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[1][i]; 
    }

    this->write(control_msg, 8);
    usleep(30000);
           for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[5][i]; 
    }  
    this->write(control_msg, 8);
    usleep(30000);
         for (size_t i = 0; i < 8; i++)
    {
       control_msg[i]=control_data[7][i]; 
    }

    this->write(control_msg, 8);
    usleep(30000);
    break;

  case 11:

    break;


  }
 
  // this->write(motor_En(id), 8);
  // usleep(5000);
  // if (mode != PRIM_Homming) {
  //   int32_t tarPos = tarAngle * ratioAngleToPulse;
  //   this->write(setCmd(mode, id, tarPos), 8);
  //   usleep(5000);
  //   this->write(setCmd(PRIM_Go, id, 0), 8);
  //   usleep(5000);
  // } else {
  //   this->write(setCmd(mode, id, 0), 8);
  //   usleep(5000);
  // }
}
M_serial::M_serial() {
  arm_sub = n.subscribe("/arm_vel", 1, &M_serial::arm_robot_Callback, this);
  pub = n.advertise<my_serial::incidence>("/incidence", 1000);
  this->Para_Init();
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  //设置要打开的串口名称
  this->setPort("/dev/ttyS1");
  //设置串口通信的波特率
  this->setBaudrate(9600);
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
    ROS_INFO_STREAM("/dev/tty10 is opened.");
  }
}