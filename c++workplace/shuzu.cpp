#include "hello.h"
#include "hello.cpp"
//单个文件的编译需要把对应的cpp文件也包含进来。

int main()
{
  checkCPUendian();
  helloword();
  
  uint8_t tx_buff[8] = {07,06,0xf5,04,03,0xf2,0xa7,0xfa}, bit[6];
  int32_t a=0 ;               //有4个字节用来存数据  05 04  03  02   1e73//低字节在后
  char c=1;
  unsigned int b;
  b=c;
  b>>=1;
  printf("%d",b);

  memcpy(&a, &tx_buff[4], 4); //115  73 ，， 30  1e  先复制低位

  // int arry[5]={1,3,7,8,10} ;
  // int *p=arry ;
  bit[0] = tx_buff[0] & 0x0f;//提取一个字节的低四位
  bit[1] = ((tx_buff[0] & 0xf0) >> 4);
  canshu ware1=PRIM_2;
  std::cout << "shuzu:" <<(int) a << ".....lingyige !" << (int)tx_buff[1] << "disiwei" << (int)bit[1] << std::endl;
  std::cout<< "canshu" <<ware1 <<std::endl;
 
}
