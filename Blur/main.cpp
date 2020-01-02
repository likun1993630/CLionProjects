#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;

int main() {

    std::string imageName = "/home/likun/CLionProjects/Blur/29.png";
    Mat image;
    Mat imageblur;

    image = imread( imageName, CV_LOAD_IMAGE_COLOR);

    if( !image.data )
    {
        printf( " No image data \n " );
        return -1;
    }

    cv::Size size(5,5);
    cv::GaussianBlur(image, imageblur, size, 1 ,1);

    imwrite("/home/likun/CLionProjects/Blur/29blur.png", imageblur);

//    namedWindow( imageName, CV_WINDOW_AUTOSIZE );
//    namedWindow( "blur image", CV_WINDOW_AUTOSIZE );
//    imshow( imageName, image );
//    imshow( "blur image", imageblur );
//    waitKey(0);
}