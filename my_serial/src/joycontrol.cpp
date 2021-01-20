#include "my_serial.h"


using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "serial_485");
  my_serial sp;
  
  //ros::Rate loop_rate(5);
  //ros::Rate loop_rate(10);
  while (ros::ok()) {
         //获取缓冲区内的字节数
    uint8_t buffer1[10],buffer2[10],buffer[10];
    size_t n=sp.available();
    size_t k;
    if (n>0 && n<9) {

      //读出数据
      n = sp.read(buffer1, n);
      ROS_INFO("n=%d",(int)n);
      
      for (size_t i = 0; i < n; i++) { 
        buffer[i]=buffer1[i];
        //ROS_INFO("bnbmbn1=%d", n);
        ROS_INFO("%d", (int)buffer1[i]);
      }
      usleep(10000);
      k=sp.available();
      k = sp.read(buffer2, k);

      for (size_t i = 0; i < k; i++) {
          buffer[9-k+i]=buffer2[i];
          //ROS_INFO("bnbmbn2=%d", k);
          ROS_INFO("%d", (int)buffer2[i]);
      }     
    }
    if (n==9){
      n = sp.read(buffer, n);
      //ROS_INFO("ndezhi=%d",(int)n);
      
    } 

    if(n>0) sp.Data_Handing(buffer);
    //usleep(100000);
   // ros::spinOnce();
   // loop_rate.sleep();
    
  }

  sp.close();
  return 0;
}

void my_serial::Para_Init() {
  n.param<int>("my_serial/Baud_Rate", baudrate, 9600);
  n.param<std::string>("my_serial/dev", dev, "/dev/ttyS0"); // duankouhao
  std::cout << "Port is " << dev << std::endl;
  std::cout << "Baud Rate is " << baudrate << std::endl;
}

uint8_t my_serial::Data_Handing(uint8_t *msg) {
  ROS_INFO("in Data_handing");
  for (size_t m = 0; m < 9; m++) {
    ROS_INFO("%d", (int)msg[m]);
  }
  uint8_t ch=0;
  for (size_t i =1; i < 8; i++)
  {
    ch+=msg[i];
  }

  if ((msg[0] == 0xff) && (msg[8] == ch)) {

    std_msgs::UInt8MultiArray arr;
    arr.data.resize(10);
    //memcpy(&arr, &msg, 9);
    for (size_t j = 0; j < 9; j++)
    {
      arr.data[j]=msg[j];
      ROS_INFO("%d", (int)arr.data[j]);
    }
    
    pub.publish(arr);
    ROS_INFO("receive data");
  }
  return 67;
}



// uint8_t my_serial::Joy_Data_Handing(uint8_t *msg) {
//   ROS_INFO("in Data_handing");
//   uint8_t rpm;
//   //zuigaoduiying 2000r/min,1000r/min,500r/min
//   uint8_t ch=0;
//   for (size_t i =1; i < 8; i++)
//   {

//     ch+=msg[i];
//   }
//   //ROS_INFO("ch =,,,,,0x%x,    msg[8]=,,,,,,,,0x%x",ch,msg[8]);

//    if (msg[0] == 0xff && msg[8] == ch) {

//      ROS_INFO("receive data");
//      YY_value=((int)msg[1])*256+((int)msg[2]);
//      XX_value=((int)msg[3])*256+((int)msg[4]);
//      ZZ_value=((int)msg[5])*256+((int)msg[6]);
//      Button_value=(int)msg[7];
     
//      ROS_INFO("YY_value=%d  ,XX_value = %d ,ZZ_value = %d ,Button_value = %d",YY_value,XX_value,ZZ_value,Button_value);

   
//    }
// }


my_serial::my_serial() {
  pub = n.advertise<std_msgs::UInt8MultiArray>("remote_control", 1);
  //vel_sub = n.subscribe("/cmd_vel", 1, &my_serial::vel_Callback, this);
  this->Para_Init();
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  //设置要打开的串口名称
  this->setPort("/dev/ttyS0");
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
    ROS_INFO_STREAM("/dev/ttyS0 is opened.");
  }
}
