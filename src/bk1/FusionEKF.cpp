#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises //MY_MS: ?
  */
    
  //create a 4D state vector, we don't know yet the values of the x state
  VectorXd x_ = VectorXd(4);

  //state covariance matrix P
  MatrixXd P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
    
  //the initial state transition matrix F_
  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
  // process noise covariance matrix
  MatrixXd Q_ = MatrixXd(4, 4);
    
  //measurement matrix
  MatrixXd H_ = MatrixXd(2, 4); //MY_MS: not really used
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;
  
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  //measurement covariance
  MatrixXd R_ = MatrixXd(2, 2);
  R_ << 0.0225, 0,
        0, 0.0225;
    
  ekf_.Init(x_, P_, F_, H_, R_, Q_);
    
  noise_ax = 9.0;
  noise_ay = 9.0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
//    cout << "EKF: " << endl;
//    ekf_.x_ = VectorXd(4);
//    ekf_.x_ << 1, 1, 1, 1;
      
      
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        
        double rho    = measurement_pack.raw_measurements_[0];
        double phi    = measurement_pack.raw_measurements_[1];
        double rhodot = measurement_pack.raw_measurements_[2];
        
        double px = rho * cos(phi);
        double py = rho * sin(phi);
      double vx = 0; //MY_MS_NEW rhodot * cos(phi);
      double vy = 0; //MY_MS_NEW rhodot * sin(phi);
        
        ekf_.x_ << px, py, vx, vy;
      
//      cout << "1: x_ = " << ekf_.x_ << endl; //MY_MS
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      
//      cout << "1: x_ = " << ekf_.x_ << endl; //MY_MS
    }
    else
    {
//        cout << "Error: Invalid data received .... Other than Radar or Lidar \n";
        return;
    }
    
    // Init covariance (Already initialized above in constructor)
//    ekf_.P_ << 1, 0, 0, 0,
//               0, 1, 0, 0,
//               0, 0, 1000, 0,
//               0, 0, 0, 1000;

    previous_timestamp_ = measurement_pack.timestamp_;
    
//    cout << "3: x_ = " << ekf_.x_ << endl; //MY_MS
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
//  cout << "4: x_ = " << ekf_.x_ << endl; //MY_MS
  
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
    
  //state transition matrix F according to the new elapsed time.
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
    
  //Update the process noise covariance matrix.
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
    
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ax;

  //MY_MS_HACK
//  ekf_.Q_ << dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
//            0, dt_4 * noise_ay / 4, 0, dt_3 * noise_ay /2,
//            dt_3 * noise_ax / 2, 0, dt_2 * noise_ax, 0,
//            0, dt_3 * noise_ay / 2, 0, dt_2 * noise_ay;

  //MY_MS: Check for 0.001 ?
  
//  cout << "5: x_ = " << ekf_.x_ << endl; //MY_MS
  ekf_.Predict();
//  cout << "6: x_ = " << ekf_.x_ << endl; //MY_MS

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
//    cout << "7: x_ = " << ekf_.x_ << endl; //MY_MS
    Hj_ = tools.CalculateJacobian(ekf_.x_);
//    cout << "8: x_ = " << ekf_.x_ << endl; //MY_MS
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
//    cout << "9: x_ = " << ekf_.x_ << endl; //MY_MS
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
//    cout << "10: x_ = " << ekf_.x_ << endl; //MY_MS
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
//    cout << "11: x_ = " << ekf_.x_ << endl; //MY_MS
    ekf_.Update(measurement_pack.raw_measurements_);
//    cout << "12: x_ = " << ekf_.x_ << endl; //MY_MS
  }

#if 1 //MY_MS
#else
  // print the output
  cout << "Last - x_ = " << ekf_.x_ << endl;
  cout << "Last - P_ = " << ekf_.P_ << endl;
#endif
}