#include <opencv2/opencv.hpp>
#include <vector>
#include <stdlib.h>
#include <iomanip>

using namespace cv;

/**
  * @brief Threshold input image using adaptive thresholding
  */
static void _threshold(cv::InputArray _in, cv::OutputArray _out, int winSize, double constant) {

    CV_Assert(winSize >= 3);
    if(winSize % 2 == 0) winSize++; // win size must be odd
    adaptiveThreshold(_in, _out, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, winSize, constant);
}

int main()
{
    cv::Mat image;
    cv::Mat gray;
    cv::Mat imageblur;
    image = cv::imread("/home/likun/CLionProjects/solvePnP/9.jpg");

    cv::Size size(9,9);
    //cv::GaussianBlur(image, imageblur, size, 1 ,1);

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);



    double adaptiveThreshConstant = 7;
    int adaptiveThreshWinSize = 33;
    cv::Mat oneimage;
    _threshold(gray, oneimage, adaptiveThreshWinSize, adaptiveThreshConstant);

    std::vector< std::vector< cv::Point>> contours;
    cv::findContours(oneimage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    for (int i = 0; i < contours.size(); ++i)
    {
        for (int j = 0; j < contours[0].size(); ++j)
        {
            std::cout << contours[i][j] << "  ";
        }
        std::cout << std::endl;
    }

    //cv::drawContours(oneimage, contours, -1, cv::Scalar(0,0,255));

    double polygonalApproxAccuracyRate = 0.03;

    Mat dst = Mat::zeros(image.size(), CV_8UC3);
    std::vector< std::vector< cv::Point>> Points;
    if (!contours.empty()) {
        // 遍历所有顶层轮廓，顶层体现在 hierarchy[idx][0]，同层
        // 随机生成颜色值绘制给各连接组成部分
        int idx = 0;
        for (; idx < contours.size(); idx ++)
        {
            Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
            drawContours(dst, contours, idx, color, 1, 8); // 绘制填充轮廓

            std::vector< Point > approxCurve;
            approxPolyDP(contours[idx], approxCurve, double(contours[idx].size()) * polygonalApproxAccuracyRate, true);
            Points.push_back(approxCurve);
        }

    }

    cv::namedWindow("display", CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
    cv::imshow("display", dst);
    // cv::waitKey(0);

    cv::imwrite(" contourone.png ",oneimage);
    
    std::system("nomacs /home/likun/CLionProjects/solvePnP/9.jpg");

    return 0;
}