#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

class KalmanFilterAruco {


public:
	KalmanFilterAruco();

public:
	cv::KalmanFilter model;
	static constexpr double dt = 0.02;
	static constexpr double dt2 = 0.02;
private:
	static constexpr int nStates = 18;
	static constexpr int nMeasurements = 6;
	static constexpr int nInputs = 0;
	static constexpr double process_noise = 1e-1;
	static constexpr double measure_noise = 1e-8;

};
