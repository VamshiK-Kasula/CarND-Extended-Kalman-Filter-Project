#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
   VectorXd rmse(4);
   rmse << 0,0,0,0;
   if ((estimations.size() > 0) && (ground_truth.size() == estimations.size())) 
   {
      for (uint i = 0; i < estimations.size(); i++) 
      {
         VectorXd diff = estimations[i] - ground_truth[i];
         diff = diff.array() * diff.array();
         rmse += diff;
      }
      rmse /= estimations.size();
      return rmse.array().sqrt();
   } 
   else 
   {
      std::cout << "RMSE Error: inconsistent vector size" << std::endl;
      return rmse;
   }
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
   MatrixXd Hj(3, 4);
   float px = x_state(0);
   float py = x_state(1);
   float vx  = x_state(2);
   float vy = x_state(3);

   float p_xy = px * px + py * py;
   float p_sqrt = sqrt(p_xy);

   // check division by zero
   if (fabs(p_xy) > 0.0) 
   {
      Hj << (px / p_sqrt), (py / p_sqrt), 0, 0, 
            (-py / p_xy), (px / p_xy), 0, 0,
            (py * (vx * py - vy * px) / (p_xy * p_sqrt)), (px * (vy * px - vx * py) / (p_xy * p_sqrt)), (px / p_sqrt), (py / p_sqrt);
   } 
   else 
   {
      std::cout << "Jacobian Error: cannot calculate" << std::endl;
   }
   return Hj;
}
