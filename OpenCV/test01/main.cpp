#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int artc, char** argv) {
    MatrixXd m(2, 2);
    m << 1, 2,
         3, 4;
    cout << m.reverse() << endl;
    Mat src = imread("/home/kavin/Pictures/test.png", IMREAD_COLOR);
    if (src.empty()) {
        printf("could not load image...\n");
        return -1;
    }
    namedWindow("input", WINDOW_AUTOSIZE);
    imshow("input", src);

    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    // imwrite("./gray.png", gray);
    namedWindow("gray", WINDOW_AUTOSIZE);
    imshow("gray", gray);
    waitKey(0);
    return 0;
}