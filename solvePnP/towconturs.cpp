#include <opencv2/opencv.hpp>
#include <vector>
#include <stdlib.h>
#include <iomanip>

using namespace cv;
using std::vector;

/**
  * Threshold input image using adaptive thresholding
  */
static void _threshold(cv::InputArray _in, cv::OutputArray _out, int winSize, double constant) {

    CV_Assert(winSize >= 3);
    if(winSize % 2 == 0) winSize++; // win size must be odd
    adaptiveThreshold(_in, _out, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                      cv::THRESH_BINARY_INV, winSize, constant);
}

int main()
{
    cv::Mat image;
    cv::Mat gray;
    image = cv::imread("/home/likun/CLionProjects/solvePnP/twoimage/2.png");

//    cv::Mat imagecut;
//    cv::Rect rect(1000,30,500,400);
//    imagecut = image(rect);

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    double adaptiveThreshConstant = 7;
    int adaptiveThreshWinSize = 33;
    cv::Mat oneimage;
    _threshold(gray, oneimage, adaptiveThreshWinSize, adaptiveThreshConstant);

    cv::imwrite("../twoimage/thresholding_image.png",image);
    cv::imwrite("../twoimage/thresholding_image_13.png",oneimage);


    // 寻找轮廓，返回contours
    std::vector< std::vector< cv::Point>> contours;
    cv::findContours(oneimage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    Mat contourimg = Mat::zeros(image.size(), CV_8UC3);

    if (!contours.empty()) {
        int idx = 0;
        for (; idx < contours.size(); idx ++)
        {
            Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
            drawContours(contourimg, contours, idx, color, 1, 8);
        }
    }
    cv::imwrite("../twoimage/contours_found_image.png",contourimg);

    return 0;
}