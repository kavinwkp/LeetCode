#include <iostream>
using namespace std;

class Account {
public:
    void calculate() { amount += amount * interestRate; }
    static double rate() { return interestRate; }
    static void rate(double);
private:
    string owner;
    double amount;
    static double interestRate;     // 静态成员变量
    static double initRate();       // 静态成员函数
};

void Account::rate(double newRate) {
    interestRate = newRate;
}

double Account::interestRate = initRate();

int main(int argc, char const *argv[])
{
    double r;
    r = Account::rate();
    Account ac1;
    Account *ac2 = &ac1;
    r = ac1.rate();
    r = ac2->rate();
    return 0;
}
