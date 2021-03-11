#include "iostream"
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
    

    return 0;
}


