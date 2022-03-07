#include <iostream>

using namespace std;

class Quote {
public:
    Quote() = default;
    Quote(const std::string& book, double sales_price) :
        bookNo(book), price(sales_price) {}
    std::string isbn() const { return boolNo; }
    virtual doble net_price(std::size_t n) const {
        return n * price;
    }
    virtual ~Quote() = default;
private:
    std::string boolNo;
protected:
    double price = 0.0; // 普通状态下不打折的价格
};

class Bulk_quote : public Quote {
public:
    Bulk_quote() = default;
    Bulk_quote(const std::sting&, double, std::size_t, double);
    double net_price(std::size_t) const override;
privete:
    srd::size_t min_qty = 0;    // 使用折扣额的最低购买量
    double discount = 0.0;  // 以小数表示的折扣额
};

int main() {


    return 0;
}