#include <iostream>

using namespace std;

class Sales_data
{
friend Sales_data add(const Sales_data&, const Sales_data&);
friend istream& read(istream&, Sales_data&);
friend ostream& print(ostream&, const Sales_data&);
public:
    Sales_data() = default;
    Sales_data(const string& s): bookNo(s) {}
    Sales_data(const string& s, unsigned n, double p): bookNo(s), units_sold(n), revenus(p * n) {}
    Sales_data(istream&);
    string isbn() const { return bookNo; }
    Sales_data& combine(const Sales_data&);
    double avg_price() const;
private:
    string bookNo;
    unsigned units_sold = 0;
    double revenus = 0.0;
};

Sales_data::Sales_data(istream& is) {
    read(is, *this);
}

double Sales_data::avg_price() const {
    if (units_sold) 
        return revenus / units_sold;
    else 
        return 0;
}

Sales_data& Sales_data::combine(const Sales_data& rhs) {
    units_sold += rhs.units_sold;
    revenus += rhs.revenus;
    return *this;
}

Sales_data add(const Sales_data& rhs, const Sales_data& lhs) {
    Sales_data sum = lhs;
    sum.combine(lhs);
    return sum;
}

istream& read(istream& is, Sales_data& item) {
    double price;
    is >> item.bookNo >> item.units_sold >> price;
    item.revenus = price * item.units_sold;
    return is;
}

ostream& print(ostream& os, const Sales_data& item) {
    os << item.isbn() << " " << item.units_sold << " "
        << item.revenus << " " << item.avg_price();
    return os;
}

int main(int argc, char const *argv[])
{
    Sales_data data1("kavin");
    cout << data1.isbn() << endl;
    return 0;
}
