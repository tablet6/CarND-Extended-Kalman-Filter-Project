#include "kalman_filter.h"
#include <iostream> //MY_MS

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
//  std::cout << "KF1: x_ = " << x_ << "\n"; //MY_MS
//  std::cout << "H_ = " << H_ << "\n"; //MY_MS
//  std::cout << "P_ = " << P_ << "\n"; //MY_MS
//  std::cout << "Ht = " << Ht << "\n"; //MY_MS
//  std::cout << "PHt = " << PHt << "\n"; //MY_MS
//  std::cout << "R_ = " << R_ << "\n"; //MY_MS
//  std::cout << "S = " << S << "\n"; //MY_MS
//  std::cout << "Si = " << Si << "\n"; //MY_MS
//  std::cout << "K = " << K << "\n"; //MY_MS
//  std::cout << "y = " << y << "\n"; //MY_MS
  //new estimate
  x_ = x_ + (K * y);
//  std::cout << "KF2: x_ = " << x_ << "\n"; //MY_MS
  
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
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
  
  //MY_MS: Check for divide by 0
  if ((px == 0) && (py == 0))
  {
    std::cout<<"Bad value...\n";
    return;
  }
  
  VectorXd h_x = VectorXd(3);
  h_x[0] = sqrt(px2 + py2);
  h_x[1] = atan2(py, px); //MY_MS: Normalize -pi to pi
  h_x[2] = (px*vx + py*vy)/sqrt(px2 + py2);
  
  VectorXd y = z - h_x;
  
//  std::cout<<"y[1]: "<<y[1]; //MY_MS
//  std::cout<<" h_x[1]: "<<h_x[1]<<"\n"; //MY_MS
  
  while (!((y[1] > -M_PI) && (y[1] < M_PI)))
  {
    //MY)MS
    {
      std::cout<<"Needs Adjustment...y[1]: "<<y[1]<<"\n";
    }

    
    if (y[1] > M_PI)
      y[1] -= 2*M_PI;
    else if (y[1] < -M_PI)
      y[1] += 2*M_PI;
  }
  
  //MY_MS
  if ((y[1] > M_PI) || (y[1] < -M_PI))
  {
    std::cout<<"HIT A BAD CASE....\n";
  }
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
