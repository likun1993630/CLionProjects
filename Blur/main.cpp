#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;

int main() {

    std::string imageName = "/home/likun/CLionProjects/Blur/161.jpg";
    Mat image;
    Mat imageblur;

    image = imread( imageName, CV_LOAD_IMAGE_COLOR);

    if( !image.data )
    {
        printf( " No image data \n " );
        return -1;
    }

    cv::Size size(5,5);
    // cv::GaussianBlur(image, imageblur, size, 0 ,0);
    // cv::medianBlur(image, imageblur, 5);
    cv::bilateralFilter(image, imageblur, 5, 10, 5);

    std::string imagename = "/home/likun/CLionProjects/Blur/161.png";
    imwrite(imagename, imageblur);

    std::string com = "nomacs " + imagename;
    std::system(com.c_str());

//    namedWindow( imageName, CV_WINDOW_AUTOSIZE );
//    namedWindow( "blur image", CV_WINDOW_AUTOSIZE );
//    imshow( imageName, image );
//    imshow( "blur image", imageblur );
//    waitKey(0);
}