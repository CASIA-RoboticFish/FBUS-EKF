#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <functional>

typedef std::function<int(int,int)> FuncType;

int Func(int x, int y)
{
    return x + y;
}

class B
{
    public:
    void RegisterFunc(FuncType f)
    {
        bfun = f; 
    }

    int BFunC()
    {
        return this->bfun(10,50);
    }

    private:
    FuncType bfun;
};

class A
{
public:
    A():m(10) {}
    ~A() {}

public:
    int Func(int x, int y)
    {
        return x + y + this->m;
    }

private:
    int m;
};

int main(int agec, char **argv)
{
    // auto bf1 = std::bind(Func, 10, std::placeholders::_1);
    // ///< same as Func(10, 20)
    // std::cout << "example 1:" << bf1(20) << std::endl;
    // A a;
    // auto bf2 = std::bind(&A::Func, a, std::placeholders::_1, std::placeholders::_2);
    // ///< same as a.Func(10, 20)
    // std::cout << "example 2:" << bf2(10, 20) << std::endl;

    // std::function<int(int)> bf3 = std::bind(&A::Func, a, std::placeholders::_1, 100);
    // ///< same as a.Func(10, 100
    // std::cout << "example 3:" << bf3(10) << std::endl;
    A a;
    FuncType bf2 = std::bind(&A::Func, a, std::placeholders::_1, std::placeholders::_2);
    B b;
    b.RegisterFunc(bf2);
    std::cout << "example 4:" << b.BFunC() << std::endl;
    return 0;
}