#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    // 2D image points. If you change the image, you need to change vector
    std::vector<cv::Point2d> image_points;
//    image_points.push_back( cv::Point2d(960+0, 540+0) );     // Left top corner
//    image_points.push_back( cv::Point2d(960+60, 540+0) );    // Right top corner
//    image_points.push_back( cv::Point2d(960+0, 540+57) );    // Left bottom corner
//    image_points.push_back( cv::Point2d(960+60, 540+57) );    // Right bottom corner
    image_points.push_back( cv::Point2d(614.8583972119726, 371.6058481510529) );     // Left top corner
    image_points.push_back( cv::Point2d(660.5546434178511, 371.6008453171359) );    // Right top corner
    image_points.push_back( cv::Point2d(660.5465187533821, 325.8706567525559) );    // Left bottom corner
    image_points.push_back( cv::Point2d(610.8665218764417, 320.856539186389) );    // Right bottom corner

    // 3D model points.
    double markerLength = 0.25;
    std::vector<cv::Point3d> model_points;
    model_points.push_back(cv::Point3d(-markerLength / 2.f, markerLength / 2.f, 0));       // Left eye left corner
    model_points.push_back(cv::Point3d(markerLength / 2.f, markerLength / 2.f, 0));        // Right eye right corner
    model_points.push_back(cv::Point3d(markerLength / 2.f, -markerLength / 2.f, 0));      // Left Mouth corner
    model_points.push_back(cv::Point3d(-markerLength / 2.f, -markerLength / 2.f, 0));       // Right mouth corner

    // Camera internals
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 913.653581, 0.000000, 637.711518, 0.000000, 914.594866, 348.727618, 0.000000, 0.000000, 1.000000);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.095512, -0.162003, 0.003557, -0.002188, 0.000000);


    // Output rotation and translation
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;

    cv::Mat intputrvec = (cv::Mat_<double>(1,3) << 0,0,0);
    cv::Mat intputtvec = (cv::Mat_<double>(1,3) << 0,0,20);

    std::vector<cv::Point2d> image_output;
    cv::projectPoints(model_points,intputrvec, intputtvec, cameraMatrix, distCoeffs, image_output);

//    cv::Matx12d m(1.0,2.0);
//    //cv::Mat m3 = (cv::Matx12d << 1,2);
//    cv::Matx<double, 1, 2> m1 (22,11);
//    cv::Mat_<double> m2(2,2);
//
//    m2.type();


    // Solve for pose
    // SOLVEPNP_P3P
    // SOLVEPNP_AP3P
    // SOLVEPNP_EPNP xx
    // SOLVEPNP_IPPE
    // SOLVEPNP_IPPE_SQUARE
    cv::solvePnP(model_points, image_points, cameraMatrix, distCoeffs,
            rotation_vector, translation_vector, false ,SOLVEPNP_IPPE_SQUARE);

    std::cout << translation_vector << std::endl;
    std::cout << rotation_vector << std::endl;
    std::cout << image_output << std::endl;

    double x = image_output[1].x - image_output[0].x;
    std::cout << x << std::endl;

//    std::cout << m << std::endl;
//    std::cout << m1 << std::endl;
//    std::cout << m2.channels() << std::endl;
}
