#include "iostream"
using namespace std;
int main(int argc, char const *argv[])
{
    char buf[20];
    cin.ignore(7);
    cin.getline(buf,10);
    cout<< buf <<endl;
    return 0;
}


