#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <string>

//void Markerset();
void makesigleMarker(int id);
//void Markerboard(int idbegin, int idend);

int main(int argc, char** argv){

    for (int i = 0; i <= 249; ++i)
        makesigleMarker(i);

//    Markerset();
//    for (int i = 200; i < 220; i=i+2) {
//        Markerboard(i,i+1);
//    }
//    for (int i = 200; i < 250; i=i+6) {
//        Markerboard(i,i+5);
//    }

    return 0;
}

void makesigleMarker(int id){
    int markerid = id;
    std::string markername = "Marker"+std::to_string(markerid)+".jpg";
    //生成 Aruco Marker
    cv::Mat markerImage;
    int markerpixels = 800;
    int imagepixels = 970;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::drawMarker(dictionary, markerid, markerpixels, markerImage, 1);

    // 生成全白背景
    //    cv::Mat background = cv::Mat::zeros(imagepixels, imagepixels, CV_8UC3);
    //    background.setTo(cv::Scalar(255, 255, 255));
    cv::Mat background = cv::Mat::zeros(imagepixels, imagepixels, CV_8UC1);
    background.setTo(cv::Scalar(255));

    int a = markerImage.channels();

    // 将marker嵌入image
    int x0 = (imagepixels-markerpixels)/2-1;
    int y0 = x0;
    cv::Rect roi_rect = cv::Rect(x0, y0, markerImage.cols, markerImage.rows);
    markerImage.copyTo(background(roi_rect));

    background.copyTo(markerImage);
    cv::imshow("Marker",markerImage); // 打印图像
    //cv::waitKey ( 0 ); // 程序暂停，等待输出图像
    cv::imwrite( markername, markerImage); //输出图像到文件
}
//
//void Markerset(){
//    //设置深蓝色背景画布
//    cv::Mat image = cv::Mat::zeros(1300, 1300, CV_8UC3);
//    image.setTo(cv::Scalar(255, 255, 255));
//    //cv::imshow("original", image);
//
//    //读取待复制图片
//    cv::Mat roi = cv::imread("Marker0.jpg", cv::IMREAD_COLOR);
//    cv::Mat roi1 = cv::imread("Marker1.jpg", cv::IMREAD_COLOR);
//    cv::Mat roi2 = cv::imread("Marker2.jpg", cv::IMREAD_COLOR);
//    cv::Mat roi3 = cv::imread("Marker3.jpg", cv::IMREAD_COLOR);
//
//    //设置画布绘制区域并复制
//    // Rect 矩形区域（区域左上角点x,y,区域大小x,y）
//    cv::Rect roi_rect = cv::Rect(100, 100, roi.cols, roi.rows);
//    cv::Rect roi_rect1 = cv::Rect(700, 100, roi.cols, roi.rows);
//    cv::Rect roi_rect2 = cv::Rect(100, 700, roi.cols, roi.rows);
//    cv::Rect roi_rect3 = cv::Rect(700, 700, roi.cols, roi.rows);
//
//    roi.copyTo(image(roi_rect));
//    roi1.copyTo(image(roi_rect1));
//    roi2.copyTo(image(roi_rect2));
//    roi3.copyTo(image(roi_rect3));
//
//    //cv::imshow("result", image);
//    //cv::waitKey(0);
//    cv::imwrite( "set.jpg", image); //输出图像到文件
//}

void Markerboard(int idbegin, int idend)
{
    int num = idend - idbegin + 1;
    int col;
    int row;
    if(num == 2)
    {
        col = 2;
        row = 1;
    }
    else if(num == 4)
    {
        col = 2;
        row = 2;
    }
    else if(num == 6)
    {
        col = 3;
        row = 2;
    }
    else
        std::cout << num << std::endl;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(col, row, 0.2, 0.025, dictionary);

    for(int i =0; i < num; i++)
    {
        board->ids[i] = idbegin + i;
    }
    cv::Mat boardImage;
    board->draw( cv::Size(1920, 1080), boardImage, 10, 1 );
    std::string markername = "Board"+std::to_string(idbegin)+"-"+std::to_string(idend)+".jpg";
    std::cout << markername << std::endl;
    cv::imwrite(markername, boardImage);
}