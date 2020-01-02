#include <iostream>
double hmean(double a, double b);

int main()
{
    double x, y, z;

    std::cout << "Enter two numbers: ";
    while (std::cin >> x >> y)
    {
        try {                   // try 块开始
            z = hmean(x,y);
        }                       // try 块结束
        catch (const char * s)  // 异常处理函数
        {
            std::cout << s << std::endl;
            std::cout << "输入一对数: ";
            continue;
        }
        std::cout << "结果为： "<< z << std::endl;
        std::cout << "输入下一对数 <q to quit>: ";
    }
    std::cout << "Bye!\n";
    return 0;
}

double hmean(double a, double b)
{
    if (a == -b)
        throw "参数不合法: a = -b ";
    return 2.0 * a * b / (a + b);
}