#include <iostream>
#include <opencv2/core.hpp>
#include <vector>
using namespace cv;

static void _getSingleMarkerObjectPoints(float markerLength, OutputArray _objPoints) {

    CV_Assert(markerLength > 0);

    _objPoints.create(4, 1, CV_32FC3);
    Mat objPoints = _objPoints.getMat();
    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.ptr< Vec3f >(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
}

class SinglePoseEstimationParallel : public ParallelLoopBody {
public:
    SinglePoseEstimationParallel(Mat& _markerObjPoints, InputArrayOfArrays _corners,
                                 InputArray _cameraMatrix, InputArray _distCoeffs,
                                 Mat& _rvecs, Mat& _tvecs)
            : markerObjPoints(_markerObjPoints), corners(_corners), cameraMatrix(_cameraMatrix),
              distCoeffs(_distCoeffs), rvecs(_rvecs), tvecs(_tvecs) {}

    void operator()(const Range &range) const CV_OVERRIDE {
        const int begin = range.start;
        const int end = range.end;

        for(int i = begin; i < end; i++) {
            cv::solvePnP(markerObjPoints, corners.getMat(i), cameraMatrix, distCoeffs,
                     rvecs.at<Vec3d>(i), tvecs.at<Vec3d>(i));
        }
    }

private:
    SinglePoseEstimationParallel &operator=(const SinglePoseEstimationParallel &); // to quiet MSVC

    Mat& markerObjPoints;
    InputArrayOfArrays corners;
    InputArray cameraMatrix, distCoeffs;
    Mat& rvecs, tvecs;
};


void estimatePoseSingleMarkers(InputArrayOfArrays _corners, float markerLength,
                               InputArray _cameraMatrix, InputArray _distCoeffs,
                               OutputArray _rvecs, OutputArray _tvecs, OutputArray _objPoints) {

    CV_Assert(markerLength > 0);

    Mat markerObjPoints;
    _getSingleMarkerObjectPoints(markerLength, markerObjPoints);
    int nMarkers = (int)_corners.total();
    _rvecs.create(nMarkers, 1, CV_64FC3);
    _tvecs.create(nMarkers, 1, CV_64FC3);

    Mat rvecs = _rvecs.getMat(), tvecs = _tvecs.getMat();

    //// for each marker, calculate its pose
    // for (int i = 0; i < nMarkers; i++) {
    //    solvePnP(markerObjPoints, _corners.getMat(i), _cameraMatrix, _distCoeffs,
    //             _rvecs.getMat(i), _tvecs.getMat(i));
    //}

    // this is the parallel call for the previous commented loop (result is equivalent)
    parallel_for_(Range(0, nMarkers),
                  SinglePoseEstimationParallel(markerObjPoints, _corners, _cameraMatrix,
                                               _distCoeffs, rvecs, tvecs));
    if(_objPoints.needed()){
        markerObjPoints.convertTo(_objPoints, -1);
    }
}





int main() {

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 913.653581, 0.000000, 637.711518, 0.000000, 914.594866, 348.727618, 0.000000, 0.000000, 1.000000);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.095512, -0.162003, 0.003557, -0.002188, 0.000000);

    std::vector<cv::Vec3d> rvecs, tvecs;

    std::vector<std::vector<cv::Point2f> > corners;



    cv::aruco::estimatePoseSingleMarkers(corners, 0.25, cameraMatrix, distCoeffs, rvecs, tvecs);
    return 0;
}