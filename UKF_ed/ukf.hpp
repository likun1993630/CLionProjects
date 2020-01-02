#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;


class MeasurementPackage {
 public:
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;
};


class UKF {
 public:

  UKF();
  virtual ~UKF(){};

  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateMeasurement(MeasurementPackage meas_package);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_px_;

  // Laser measurement noise standard deviation position2 in m
  double std_py_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
};

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  is_initialized_ = false;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

  
  //@ measurement noise values are provided by the sensor manufacturer.
  //@ measurement noise standard deviation
  //  position1 in m
  std_px_ = 0.15;
  // position2 in m
  std_py_ = 0.15;
  // radius in m
  std_radr_ = 0.3;
  // angle in rad
  std_radphi_ = 0.03;
  // radius change in m/s
  std_radrd_ = 0.3;
  

  time_us_ = 0.0;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  weights_ = VectorXd(2 * n_aug_ + 1);

  
    P_ << 1000,0,0,0,0,
          0,1000,0,0,0,
          0,0,1000,0,0,
          0,0,0,1000,0,
          0,0,0,0,1000;

    x_ << 1, 1, 1, 1, 0.1;
  
}


void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (!is_initialized_)
  {

    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);

      time_us_ = meas_package.timestamp_;

    }
    else
    {
      float iniro = meas_package.raw_measurements_(0);
      float initheta = meas_package.raw_measurements_(1);
      float iniro_dot = meas_package.raw_measurements_(2);

      x_(0) = iniro * cos(initheta);
      x_(1) = iniro * sin(initheta);
      x_(2) = iniro_dot;
      x_(3) = initheta;

      time_us_ = meas_package.timestamp_;

    }

    is_initialized_ = true;

    return;
  }

  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  { 

    UpdateLidar(meas_package);

  }
  else
  {

    UpdateRadar(meas_package);

  }

}



void UKF::Prediction(double delta_t) {

  /*****************************************************************************
  *  Generate Sigma Points
  ****************************************************************************/
  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set lambda for non-augmented sigma points
  lambda_ = 3 - n_x_;

  //set first column of sigma point matrix
  Xsig.col(0) = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }

  /*****************************************************************************
  *  Augment Sigma Points
  ****************************************************************************/
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //set lambda for augmented sigma points
  lambda_ = 3 - n_aug_;

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  /*****************************************************************************
  *  Predict Sigma Points
  ****************************************************************************/
  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    double p_x      = Xsig_aug(0, i);
    double p_y      = Xsig_aug(1, i);
    double v        = Xsig_aug(2, i);
    double yaw      = Xsig_aug(3, i);
    double yawd     = Xsig_aug(4, i);
    double nu_a     = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  /*****************************************************************************
  *  Convert Predicted Sigma Points to Mean/Covariance
  ****************************************************************************/

  // set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);             //******* necessary? *********
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);             //******* necessary? *********
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

}



void UKF::UpdateMeasurement(MeasurementPackage meas_package) {

  VectorXd z(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
  MatrixXd H(2,5);
  H << 1,0,0,0,0,
       0,1,0,0,0;
  MatrixXd R(2,2);
  R << std_px_ * std_px_ , 0,
       0, std_py_ * std_py_;

  VectorXd y = z - H * x_;
  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(5,5) - K * H) * P_;


}