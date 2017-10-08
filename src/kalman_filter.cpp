#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_          = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_          = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y      = z - z_pred;
  MatrixXd Ht     = H_.transpose();
  MatrixXd S      = H_ * P_ * Ht + R_;
  MatrixXd Si     = S.inverse();
  MatrixXd PHt    = P_ * Ht;
  MatrixXd K      = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  
  long x_size = x_.size();
  MatrixXd I  = MatrixXd::Identity(x_size, x_size);
  P_          = (I - K * H_) * P_;
}

inline double normalize(double phi)
{
  while (!((phi > -M_PI) && (phi < M_PI)))
  {
    if (phi > M_PI)
      phi -= 2*M_PI;
    else if (phi < -M_PI)
      phi += 2*M_PI;
  }
  
  return phi;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  double px  = x_(0);
  double px2 = px * px;
  double py  = x_(1);
  double py2 = py * py;
  double vx  = x_(2);
  double vy  = x_(3);
  
  if ((px == 0) && (py == 0))
  {
    std::cout<<"Bad value...\n";
    return;
  }
  
  VectorXd h_x = VectorXd(3);
  h_x[0]       = sqrt(px2 + py2);
  h_x[1]       = atan2(py, px);
  h_x[2]       = (px*vx + py*vy)/sqrt(px2 + py2);
  
  VectorXd y = z - h_x;
  
  // Normalize phi
  // When calculating y with radar sensor data, the second value in polar coord, needs to be
  // normalized so the angle is between -PI and PI
  y[1] = normalize(y[1]);
  
  MatrixXd Ht  = H_.transpose();
  MatrixXd S   = H_ * P_ * Ht + R_;
  MatrixXd Si  = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K   = PHt * Si;
  
  //new estimate
  x_          = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I  = MatrixXd::Identity(x_size, x_size);
  P_          = (I - K * H_) * P_;
}
