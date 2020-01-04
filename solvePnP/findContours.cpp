#include <opencv2/opencv.hpp>
#include <vector>
#include <stdlib.h>
#include <iomanip>

using namespace cv;

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
    cv::Mat imageblur;
    image = cv::imread("/home/likun/CLionProjects/solvePnP/images/9.jpg");

    // cut image
    cv::Mat imagecut;
    cv::Rect rect(837,80,65,65);
    //imagecut = image(rect);

    // cv::Size size(9,9);
    // cv::GaussianBlur(image, imageblur, size, 1 ,1);

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    double adaptiveThreshConstant = 7;
    int adaptiveThreshWinSize = 23;
    cv::Mat oneimage;
    _threshold(gray, oneimage, adaptiveThreshWinSize, adaptiveThreshConstant);

    std::vector< std::vector< cv::Point>> contours;
    cv::findContours(oneimage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    /*
     * 这里的contour是连续密集的点组成的边线
     * 一个像素点也算contour
     */
    for (int i = 0; i < contours.size(); ++i)
    {
        for (int j = 0; j < contours[i].size(); ++j)
        {
            std::cout << contours[i][j];
        }
        std::cout << std::endl;
    }

    cv::drawContours(image, contours, -1, cv::Scalar(0,0,255));

    double polygonalApproxAccuracyRate = 0.03;

    Mat dst = Mat::zeros(imagecut.size(), CV_8UC3);
    std::vector< std::vector< cv::Point>> Points;
    if (!contours.empty()) {
        int idx = 0;
        for (; idx < contours.size(); idx ++)
        {
            Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
            drawContours(dst, contours, idx, color, 1, 8);

            std::vector< Point > approxCurve;
            approxPolyDP(contours[idx], approxCurve, double(contours[idx].size()) * polygonalApproxAccuracyRate, true);
            Points.push_back(approxCurve);
        }
    }

    cv::namedWindow("display", CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
    cv::imshow("display", image);
    cv::waitKey(0);

    cv::polylines(dst, Points[1], 1, cv::Scalar(255, 255, 255));
    //cv::imwrite("../images/dst23cut.png",dst);
    cv::imwrite("../images/image9.png",image);
    std::cout << "imageoutput" << std::endl;

    for (int i = 0; i < Points.size(); ++i)
    {
        for (int j = 0; j < Points[i].size(); ++j)
        {
            std::cout << Points[i][j];
        }
        std::cout << std::endl;
    }

    // std::system("nomacs /home/likun/CLionProjects/solvePnP/images/dst23cut.png");

    return 0;
}