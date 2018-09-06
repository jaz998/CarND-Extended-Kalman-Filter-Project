#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


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
	P_ = F_ * P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:	
    * update the state by using Kalman Filter equations (Laser updates)
  */
	cout << "Calling EKF update function" << endl;

	// Measurement Update
	VectorXd y = z - H_ * x_;
	MatrixXd S = H_ * P_*H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose()*S.inverse();

	x_ = x_ + K * y; // Updating the state variables
	MatrixXd I = MatrixXd::Identity(4, 4);
	P_ = (I - K * H_)*P_;



}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	cout << "Calling Update EKF Function " << endl; 
	cout << "Declaring variables" << endl; 
	cout << "px to vy " << endl; 
	double px = x_[0];
	double py = x_[1];
	cout << "line 66" << endl; 
	double vx = x_[2];
	double vy = x_[3];

	cout << "rho, phi, rho dot" << endl ; 
	double rho = sqrt(pow(px, 2) + pow(py, 2));
	double phi = atan2(py,px);
	cout << "line 73" << endl; 
	double rho_dot = (px*vx) + (py*vy) / sqrt(pow(px, 2) + pow(py, 2));
	cout << "Declaring vector h" << endl; 
	VectorXd h = VectorXd(3);
	cout << "h" << endl; 
	h << rho, phi, rho_dot;
	cout << "line 79 " << endl;
	cout << "z size is " << z.size() << endl;
	cout << "h size is " << h.size() << endl; 
	cout << "x_ size is " << x_.size() << endl;
	VectorXd y = z - h;
	cout << "line 81" << endl;
	MatrixXd S = H_ * P_*H_.transpose() + R_;
	cout << "line 83" << endl;	
	MatrixXd K = P_ * H_.transpose()*S.transpose();
	cout << "line 85" << endl; 
	x_ = x_ + K * y;
	cout << "line 87 " << endl; 
	MatrixXd I = MatrixXd::Identity(4, 4);
	cout << "line 89" << endl; 
	P_ = (I - K * H_)*P_;
	cout << "line 91" << endl; 




}
 