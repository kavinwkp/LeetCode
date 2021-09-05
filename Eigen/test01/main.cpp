#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

int main(int argc, char const *argv[])
{
    for (int i = 0; i < 5; i++) {
        cout << i << " ";
    }
    cout << endl;
    cout << "Vector3f:" << endl;
    Vector3f vi(0.f, 1.f, 2.f);
    cout << vi << endl;
    return 0;
}
