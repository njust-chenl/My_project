#include  "fstream"
#include  "iostream"
#include "cstdlib"
using namespace std;
int main (int argc , char* argv[] )
{
    
    ifstream inq1 ( argv[1],ios::in );//ifstream是读取的类
    if (!inq1)
    {
       cerr << "文件打开错误" << endl ;
       return 0 ;
       inq1.close();

    }
    ofstream ouq1 (argv[2],ios::out);//打开或创建一个可写入的文件，ofstream是一个写入的类
    if(!ouq1)
    {
        cerr<<"文件写入错误"<<endl ;
        return 0;
        ouq1.close();

    }
    
    char x;
    while (inq1>>x)
    {
       ouq1 << x ;
    
    }
    ouq1.close();
    inq1.close () ;
    
    return 0;
}
