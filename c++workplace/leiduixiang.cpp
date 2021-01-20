#include <iostream>
#include <string>
#include <fstream>
class BaseClass
{
    public:
    std::string quote, speaker;
    std::ofstream fileOutput;
    BaseClass();
    ~BaseClass();
    void inputQuoate();
    void inputspeaker();
    bool write();
};
/*
class SubClass : public BaseClass
{
    public:
    SubClass();
    ~SubClass();
    void dosomethings();
};
*/
BaseClass::BaseClass()
{
    fileOutput.open("test.txt",std::ios::app);

}
BaseClass::~BaseClass()
{
    fileOutput.close();

}
void BaseClass::inputQuoate()
{
    std::getline(std::cin,quote);

}
void BaseClass::inputspeaker()
{
    std::getline(std::cin,speaker);


}
bool BaseClass::write()
{
    if(fileOutput.is_open())
    {
        fileOutput<< quote << "zuozhe"<< speaker <<std::endl ;
        return true ;
    }
    else
    {
        return false;
    }
    

}






/*
SubClass::SubClass()
{
    std::cout<<"我进入子类构造器。。。\n";

}
SubClass::~SubClass ()
{
    std::cout <<"我进入子类析构器干了一些事。。。\n";

}
void SubClass::dosomethings ()
{
    std::cout << "wozai zilei zuolyixieshi \n";
}

*/


int main()
{
    BaseClass quote ;
    std::cout <<"请输入一句话：\n";
    quote.inputQuoate();
    std::cout <<"亲输入作者：\n";
    quote.inputspeaker();
    if(quote.write())
    {
        std::cout << "成功写入\n" ;

    }
    else
    {
        std::cout <<"写入失败\n" ;

    }
    
    return 0;
}
