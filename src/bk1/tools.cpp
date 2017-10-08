#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  /**
  TODO:
    * Calculate the RMSE here.
  */
    
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    
    if (estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
        cout<<"Invalid estimation or ground truth data \n";
        return rmse;
    }
    
    // Accumulate Squared Residuals
    for (int i=0; i<estimations.size(); i++)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        
        // co-efficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    
    // Calculate the mean
    rmse = rmse/estimations.size();
    
    // Calculate the Squared Root
    rmse = rmse.array().sqrt();
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    
    MatrixXd Hj(3, 4);
    
    //Recover State Parameters
    float px = x_state[0];
    float py = x_state[1];
    float vx = x_state[2];
    float vy = x_state[3];
    
    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px + py*py;
    float c2 = sqrt(c1);
    float c3 = c1*c2;
    
    if (fabs(c1) < 0.0001)
    {
        cout<<"Jacobian hit a divide by zero error \n";
        return Hj;
    }
    
    Hj << (px/c2),  (py/c2), 0, 0,
          (-py/c1), (px/c1), 0, 0,
          (py*(vx*py - vy*px)/c3), (px*(px*vy - py*vx)/c3), px/c2, py/c2;
    
    return Hj;
}
