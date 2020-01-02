#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    // @Input 3D model points.
    float markerLength = 0.25;
    // clockwise
    std::vector<cv::Point3d> model_points;
    model_points.push_back(cv::Point3d(-markerLength / 2.f, markerLength / 2.f, 0));
    model_points.push_back(cv::Point3d(markerLength / 2.f, markerLength / 2.f, 0));
    model_points.push_back(cv::Point3d(markerLength / 2.f, -markerLength / 2.f, 0));
    model_points.push_back(cv::Point3d(-markerLength / 2.f, -markerLength / 2.f, 0));

    // @Input Camera internals
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 913.653581, 0.000000, 637.711518, 0.000000, 914.594866, 348.727618, 0.000000, 0.000000, 1.000000);
    //cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.095512, -0.162003, 0.003557, -0.002188, 0.000000);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // @Input rotation and translation
    //cv::Mat intputrvec = (cv::Mat_<double>(1,3) << 0,-0.785,0);
    cv::Mat intputrvec = (cv::Mat_<double>(1,3) << 0,0,0);
    cv::Mat intputtvec = (cv::Mat_<double>(1,3) << 0,0,3);

    // @Output four corners of marker in image
    // Clockwise 1234
    std::vector<cv::Point2d> image_output;

    cv::projectPoints(model_points, intputrvec, intputtvec, cameraMatrix, distCoeffs, image_output);

    int width = 1280;
    int length = 720;
    cv::Mat imagep = Mat::zeros(length,width,CV_8UC3);

    std::vector<cv::Point2i> image_outputint(4);
    for (int i = 0; i < image_output.size(); ++i) {
        image_outputint[i].x = static_cast<int>(image_output[i].x);
        image_outputint[i].y = static_cast<int>(image_output[i].y);
    }

    cv::fillConvexPoly(imagep, image_outputint, Scalar(255,0,0));

    cv::namedWindow("Display", CV_WINDOW_NORMAL);
    cv::imshow("Display", imagep);

    std::cout << image_output << std::endl;

    waitKey(0);
    return 0;
}