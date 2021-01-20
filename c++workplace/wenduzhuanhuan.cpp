#include "iostream"
int main()
{
    const unsigned short add_subtract = 32;
    const double ratio = 9.0/5.0 ;
    double tempin , tempout  ;
    
    char typein ;
    char typeout ;
    std::cin >>tempin >> typein ;
    std:: cin.ignore(100,'\n');
    switch (typein)
    {
    case 'c':
    case 'C':
            tempout = tempin *ratio + add_subtract ;
            typeout ='F';
        break;
    case 'F':
    case 'f' :
            tempout = (tempin -add_subtract) / ratio ;
            typeout = 'C' ;
        break;

    default:  std::cout <<"输入错误"<< std::endl ;
        break;
    } 
    std:: cout << tempout << typeout <<std::endl ;
    return 0;
}
