#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {result += coeffs[i]*pow(x, i);}
    return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);
    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }
    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main() {
    Eigen::VectorXd xvals(8);
    Eigen::VectorXd yvals(8);
    xvals << 4, 5, 6, 7, 8, 9, 10, 11;
    yvals << 0.059726856, 0.0566536923, 0.0618597145, 0.0618597145, 0.0650315022, 0.0715808282, 0.0793805915, 0.1002273346;
    auto coeffs= polyfit(xvals,yvals,3);
    std::cout<< "Y(16)=" << polyeval(coeffs,16) << endl;
    std::cout<< coeffs << std::endl;
    Mat out(150, 500, CV_8UC3,Scalar::all(0));

    for (int i = 0; i < xvals.size(); ++i)
    {
        Point ipt = Point(xvals(i)*2,polyeval(coeffs,xvals(i))*1000);
        circle(out, ipt, 3, Scalar(0, 0, 255), CV_FILLED, CV_AA);
    }

    namedWindow( "DW", CV_WINDOW_AUTOSIZE );
    imshow("DW", out);
    waitKey(0);
    return 0;
}
