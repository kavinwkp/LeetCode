#include <iostream>
#include <vector>
using namespace std;

int main(int argc, char const *argv[])
{
    cout << "hello" << endl;
    vector<int> vi = {1, 2, 3};
    for (auto v : vi) 
        cout << v << " ";
    cout << endl;
    return 0;
}
