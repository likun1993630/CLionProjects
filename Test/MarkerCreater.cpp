#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <string>

void Markerset();
void makesigleMarker(int id);

int main(int argc, char** argv){

//    for (int i = 0; i <= 8; ++i)
//        makesigleMarker(i);

//    Markerset();
    return 0;
}

void makesigleMarker(int id){
    int markerid = id;
    std::string markername = "Marker"+std::to_string(markerid)+".jpg";
    //生成 Aruco Marker
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::drawMarker(dictionary, markerid, 500, markerImage, 1);
    cv::imshow("Marker",markerImage); // 打印图像
    //cv::waitKey ( 0 ); // 程序暂停，等待输出图像
    cv::imwrite( markername, markerImage); //输出图像到文件
}

void Markerset(){
    //设置深蓝色背景画布
    cv::Mat image = cv::Mat::zeros(1300, 1300, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));
    //cv::imshow("original", image);

    //读取待复制图片
    cv::Mat roi = cv::imread("Marker0.jpg", cv::IMREAD_COLOR);
    cv::Mat roi1 = cv::imread("Marker1.jpg", cv::IMREAD_COLOR);
    cv::Mat roi2 = cv::imread("Marker2.jpg", cv::IMREAD_COLOR);
    cv::Mat roi3 = cv::imread("Marker3.jpg", cv::IMREAD_COLOR);

    //设置画布绘制区域并复制
    // Rect 矩形区域（区域左上角点x,y,区域大小x,y）
    cv::Rect roi_rect = cv::Rect(100, 100, roi.cols, roi.rows);
    cv::Rect roi_rect1 = cv::Rect(700, 100, roi.cols, roi.rows);
    cv::Rect roi_rect2 = cv::Rect(100, 700, roi.cols, roi.rows);
    cv::Rect roi_rect3 = cv::Rect(700, 700, roi.cols, roi.rows);

    roi.copyTo(image(roi_rect));
    roi1.copyTo(image(roi_rect1));
    roi2.copyTo(image(roi_rect2));
    roi3.copyTo(image(roi_rect3));

    //cv::imshow("result", image);
    //cv::waitKey(0);
    cv::imwrite( "set.jpg", image); //输出图像到文件
}