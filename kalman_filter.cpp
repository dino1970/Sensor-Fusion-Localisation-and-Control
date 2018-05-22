#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
	Tools tools;
	VectorXd xpol = VectorXd(3);

	

	xpol[0] = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
	
	if (xpol[0] < 0.0001) {
		cout << "UpdateEKF () - Error - Division by Zero" << endl;
		return;
	}
	xpol[1] = atan2(x_[1],x_[0]);

	xpol[2] = (x_[0] * x_[2] + x_[1] * x_[3]) / (xpol[0]);
      

	VectorXd y = z - xpol;
	
	while (y[1] < -M_PI || y[1] > M_PI ) {
		if (y[1] < -M_PI ) {
			y[1] = y[1] + 2 * M_PI;
		}
		if (y[1] > M_PI ) {
			y[1] = y[1] - 2 * M_PI;
		}
	}
	
	
	MatrixXd Hj = tools.CalculateJacobian(x_);
	MatrixXd Htj = Hj.transpose();
	MatrixXd S = Hj * P_ * Htj + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Htj;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj) * P_;
	
}
