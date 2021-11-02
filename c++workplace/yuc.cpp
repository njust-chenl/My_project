#include "iostream"
#include <memory.h>
#include <string.h>
#include <vector>
//using namespace std;
int main(int argc, char const *argv[])
{
    // char buf[20];
    // std::cin.ignore(7);
    // std::cin.getline(buf,10);
    // std::cout<< buf <<std::endl;
    std::vector<int> nums;
    nums.push_back(1);
    nums.push_back(2);
    nums.push_back(5);
    nums.push_back(4);
    for (auto  i : nums)
    {
        std::cout << i<<std::endl;
    }
    char* str=NULL;
    str=(char*)malloc(16);
    std::cout<< sizeof(str)<<std::endl;
    memset(str,'0',16);
    strcpy(str,"12345678");
    std::cout<<str<<std::endl;
    return 0;
}


