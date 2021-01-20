#include "hello.h"

void helloword(){

 std::cout<<"helloword"<<std::endl;
 

}
int checkCPUendian()
{
  union
  {
    char b; 
    float a;
    
  }c;
  c.b= 'j';
  printf("%d\n",c.a);
  return (c.b == 1); 
}