#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ctime>
#include "KalmanFilterAruco.h"
#include <thread>
#include "opencv2/aruco/dictionary.hpp"

using namespace std;
double markerLength = 0.157; // [m]

//Camera Calibration values
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 746.346, 0.0, 295.408, 0.0, 746.137, 234.525, 0.0, 0.0, 1.0);
cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 0.0695, 0.135712, -0.00251498, -0.0034567, -0.60530046);

////Aruco Dictionary
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
//cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
//parametcv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;


//cv::aruco::DetectorParameters dec;
//static dec<cv::aruco::DetectorParameters > 	create()
//Create Kalman Filter
KalmanFilterAruco KF;

//Variables
cv::VideoCapture cap;
cv::Mat frame, image_copy;
cv::Mat gray;
cv::Mat rvecs;
cv::Mat tvecs;
cv::Mat RotM;
cv::Mat rel_pos;
cv::Mat objPoints;
std::vector<int> ids;
std::vector<std::vector<cv::Point2f> > corners;
cv::Mat measurements;
cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);{
vector<double> all_tvecs_x;
vector<double> all_tvecs_y;
vector<double> all_tvecs_z;
vector<double> all_rvecs_x;
vector<double> all_rvecs_y;
vector<double> all_rvecs_z;

void displayPosition(cv::Mat frame, cv::Mat rel_pos) {
	cv::String x_text = "y: " + to_string(rel_pos.at<double>(0) * 100);
	cv::String y_text = to_string(rel_pos.at<double>(1) * 100);
	cv::String z_text = "x: " + to_string(rel_pos.at<double>(2) * 100);
	cv::putText(frame, z_text, cv::Point2f(10, 25), 5, 1, cv::Scalar(255, 0, 0), 2, 9, false);
	cv::putText(frame, x_text, cv::Point2f(10, 50), 5, 1, cv::Scalar(0, 0, 255), 2, 9, false);

	//cv::putText(frame, z_text, cv::Point2f(10, 75), 5, 1, cv::Scalar(0, 255, 0), 2, 9, false);
}




cv::Mat  calcRelativePosition(cv::Mat rvec, cv::Mat tvec) {
	cv::Rodrigues(rvec, RotM);
	rel_pos = RotM * tvec;
	return rel_pos;
}


void predict_aruco() {


	//Kalman Predict
	cv::Mat prediction = KF.model.predict();
	//Create rvec and tvec for every found marker
	tvec = (cv::Mat_<double>(3, 1) << prediction.at<double>(0), prediction.at<double>(1), prediction.at<double>(2));
	rvec = (cv::Mat_<double>(3, 1) << prediction.at<double>(9), prediction.at<double>(10), prediction.at<double>(11));

}

double median_filter(vector<double> vec)
{
	// sort temperatures
	std::sort(vec.begin(), vec.end());

	// Compute Median temperature
	if (vec.size() % 2 == 1) //Number of elements are odd
	{
		return vec[vec.size() / 2];
	}
	else // Number of elements are even
	{
		int index = vec.size() / 2;
		return (vec[index - 1] + vec[index]) / 2;
	}
	return 0;
}
int weight = 15;
void update_aruco() {
	using namespace literals::chrono_literals;

	while (true) {
		//predict_aruco();
		
		this_thread::sleep_for(chrono::duration<double>(KF.dt2));

		//if (ids.size() > 0) {
		//	cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs, rvecs, tvecs, objPoints);
		//	for (int i = 0; i < ids.size(); i++) {
		//		if (ids[i] == 0) {
		//			////Kalman Filter
		//			measurements = (cv::Mat_<double>(6, 1) << tvecs.at<double>(i, 0), tvecs.at<double>(i, 1),
		//	    		tvecs.at<double>(i, 2), rvecs.at<double>(i, 0), rvecs.at<double>(i, 1), rvecs.at<double>(i, 2));
		//			// The "correct" phase that is going to use the predicted value and our measurement
		//			cv::Mat estimated = KF.model.correct(measurements);
		//			//Create rvec and tvec for every found marker
		//			//tvec = (cv::Mat_<double>(3, 1) << estimated.at<double>(0), estimated.at<double>(1), estimated.at<double>(2));
		//			//rvec = (cv::Mat_<double>(3, 1) << estimated.at<double>(9), estimated.at<double>(10), estimated.at<double>(11));
		//		}
		//	}
		//}
		try {
			if (ids.size() > 0) {
				cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs, rvecs, tvecs, objPoints);
				for (int i = 0; i < ids.size(); i++) {
					if (ids[i] == 0) {
						all_tvecs_x.push_back(tvecs.at<double>(i, 0));
						all_tvecs_y.push_back(tvecs.at<double>(i, 1));
						all_tvecs_z.push_back(tvecs.at<double>(i, 2));
						all_rvecs_x.push_back(rvecs.at<double>(i, 0));
						all_rvecs_y.push_back(rvecs.at<double>(i, 1));
						all_rvecs_z.push_back(rvecs.at<double>(i, 2));
						if (all_tvecs_x.size() == 15) {
							predict_aruco();
							all_tvecs_x.erase(all_tvecs_x.begin());
							all_tvecs_y.erase(all_tvecs_y.begin());
							all_tvecs_z.erase(all_tvecs_z.begin());
							all_rvecs_x.erase(all_rvecs_x.begin());
							all_rvecs_y.erase(all_rvecs_y.begin());
							all_rvecs_z.erase(all_rvecs_z.begin());
							tvec = (cv::Mat_<double>(3, 1) << median_filter(all_tvecs_x), median_filter(all_tvecs_y), median_filter(all_tvecs_z));
							rvec = (cv::Mat_<double>(3, 1) << median_filter(all_rvecs_x), median_filter(all_rvecs_y), median_filter(all_rvecs_z));
							////Kalman Filter
							measurements = (cv::Mat_<double>(6, 1) << median_filter(all_tvecs_x),
							        median_filter(all_tvecs_y), median_filter(all_tvecs_z), median_filter(all_rvecs_x), median_filter(all_rvecs_y), median_filter(all_rvecs_z));
							// The "correct" phase that is going to use the predicted value and our measurement
							cv::Mat estimated = KF.model.correct(measurements);
							////Create rvec and tvec for every found marker
							//tvec = (cv::Mat_<double>(3, 1) << estimated.at<double>(0), estimated.at<double>(1), estimated.at<double>(2));
							//rvec = (cv::Mat_<double>(3, 1) << estimated.at<double>(9), estimated.at<double>(10), estimated.at<double>(11));

						}

					}
				}

			}
			else {
				predict_aruco();
			}
		}
		catch (const std::exception & e) {
			cout << &e << endl;
		}


	}

}


int main()
{
	predict_aruco();
	//Check if camera is available
	if (!cap.open(0))
		return 0;
	//Kalman update every dt discrete seconds
	thread updater(update_aruco);
	//thread predictor(predict_aruco);
	while (true) {
		//Read image
		cap >> frame;
		frame.copyTo(image_copy);
		if (frame.empty()) break; // end of video streamd
		// Grayscale image
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		//Detect Aruco Marker
		cv::aruco::detectMarkers(gray, dictionary, corners, ids);
		//Draw Axis for every marker
		cv::aruco::drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.2);
		//Calculate relativ position between camera and marker
		rel_pos = calcRelativePosition(rvec, tvec);
		//Show position of marker on display
		displayPosition(frame, rel_pos);
		//Display Image
		cv::imshow("Aruco Tracker", frame);
		cv::waitKey(1);
		//Predict Kalman
		//predict_aruco();
	}

}

