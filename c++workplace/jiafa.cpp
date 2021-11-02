#include "iostream"
using namespace std ;
int main()
{

    float  sum =0.0 ;
    float  i ;
    while (cin >> i)
    {
        sum+=i;
        while(cin.peek()==' ')
        {
            cin.get();
        }
        if (cin.peek()=='\n')
        {
            break;
        }
        
    }
    cout << sum << endl ;
    

    return 0;
}



