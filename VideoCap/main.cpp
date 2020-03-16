#include <iostream>

#include <string>
#include <iostream>
#include "opencv2/videoio.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

int main()
{
//    cv::VideoCapture cap;
//    //cap.open(4);
//    cap.open(4, cv::CAP_DSHOW);
//    std::cout << "MJPG: " << cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
//    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
//    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
//    cv::Mat img;
//    while (cv::waitKey(10) != 'q') {
//        //if (cap.grab())
//        {
//            cap >> img;
//            cv::imshow("img", img);
//        }
//    }

    cv::VideoCapture inputVideo;
    inputVideo.open(4);
    inputVideo.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
    inputVideo.set(CV_CAP_PROP_FRAME_WIDTH ,1280);
    inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT ,720);
    cv::Mat img;
    int count = 0;

    double _frequency = cv::getTickFrequency(), _tick = cv::getTickCount();
    while (cv::waitKey(1) != 27) {
        inputVideo >> img;
        //std::cout << img.cols << std::endl;
        //std::cout << img.rows << std::endl;
        cv::imshow("img", img);
        std::string name = "/home/likun/image/test4/" + std::to_string(count) + ".png";
        cv::imwrite( name , img);
        count ++;
        std::cout << "FPS: " << cv::getTickFrequency() / (cv::getTickCount() - _tick) << std::endl;
        _tick = cv::getTickCount();
    }

//    while (inputVideo.grab()) {
//        cv::Mat image, imageCopy;
//        inputVideo.retrieve(image);
//        image.copyTo(imageCopy);
//        imageCopy.
//        cv::imshow("out", imageCopy);
//        char key = (char) cv::waitKey(10);
//        if (key == 27)
//            break;
//    }

//    cv::VideoCapture cap;
//    cap.open(4);
//    //cap.open(4, cv::CAP_DSHOW);
//    std::cout << "MJPG: " << cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
//    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
//    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
//    cv::Mat img;
//    while (cap.grab()) {
//        cv::Mat image, imageCopy;
//        cap.retrieve(image);
//        image.copyTo(imageCopy);
//        cv::imshow("out", imageCopy);
//        char key = (char) cv::waitKey(10);
//        if (key == 27)
//            break;
//    }

//    cv::VideoCapture cap;
//    cap.open(4);
//    //cap.open(4, cv::CAP_DSHOW);
//    std::cout << "MJPG: " << cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
//    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
//    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
//    cv::Mat img;
//    while (cap.grab()) {
//        cv::Mat image, imageCopy;
//        cap.retrieve(image);
//        image.copyTo(imageCopy);
//        cv::imshow("out", imageCopy);
//        char key = (char) cv::waitKey(27);
//        if (key == 27)
//            break;
//    }

//    cv::VideoCapture capture;
//    if(capture.open(4)) {
//        cv::Mat _framemat;
//        double _frequency = cv::getTickFrequency(), _tick = cv::getTickCount();
//        while(true) {
//            if(capture.read(_framemat)) {
//                std::cout << "FPS: " << cv::getTickFrequency() / (cv::getTickCount() - _tick) << std::endl;
//                _tick = cv::getTickCount();
//            }
//        }
//    }

    return 0;

}