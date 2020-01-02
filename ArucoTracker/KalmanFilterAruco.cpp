#include "KalmanFilterAruco.h"


KalmanFilterAruco::KalmanFilterAruco()
{
	model.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
	cv::setIdentity(model.processNoiseCov, cv::Scalar::all(process_noise));        // set process noise 1e-5
	cv::setIdentity(model.measurementNoiseCov, cv::Scalar::all(measure_noise));   // set measurement noise 1e-4
	cv::setIdentity(model.errorCovPost, cv::Scalar::all(1));             // error covariance
				   /* DYNAMIC MODEL */
	//  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
	// position
	model.transitionMatrix.at<double>(0, 3) = dt;
	model.transitionMatrix.at<double>(1, 4) = dt;
	model.transitionMatrix.at<double>(2, 5) = dt;
	model.transitionMatrix.at<double>(3, 6) = dt;
	model.transitionMatrix.at<double>(4, 7) = dt;
	model.transitionMatrix.at<double>(5, 8) = dt;
	model.transitionMatrix.at<double>(0, 6) = 0.5 * pow(dt, 2);
	model.transitionMatrix.at<double>(1, 7) = 0.5 * pow(dt, 2);
	model.transitionMatrix.at<double>(2, 8) = 0.5 * pow(dt, 2);
	// orientation
	model.transitionMatrix.at<double>(9, 12) = dt;
	model.transitionMatrix.at<double>(10, 13) = dt;
	model.transitionMatrix.at<double>(11, 14) = dt;
	model.transitionMatrix.at<double>(12, 15) = dt;
	model.transitionMatrix.at<double>(13, 16) = dt;
	model.transitionMatrix.at<double>(14, 17) = dt;
	model.transitionMatrix.at<double>(9, 15) = 0.5 * pow(dt, 2);
	model.transitionMatrix.at<double>(10, 16) = 0.5 * pow(dt, 2);
	model.transitionMatrix.at<double>(11, 17) = 0.5 * pow(dt, 2);
	/* MEASUREMENT MODEL */
//  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
	model.measurementMatrix.at<double>(0, 0) = 1;  // x
	model.measurementMatrix.at<double>(1, 1) = 1;  // y
	model.measurementMatrix.at<double>(2, 2) = 1;  // z
	model.measurementMatrix.at<double>(3, 9) = 1;  // roll
	model.measurementMatrix.at<double>(4, 10) = 1; // pitch
	model.measurementMatrix.at<double>(5, 11) = 1; // yaw

}

