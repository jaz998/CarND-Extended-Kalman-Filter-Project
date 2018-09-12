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

	x_ = x_ + (K * y); // Updating the state variables
	MatrixXd I = MatrixXd::Identity(4, 4);
	P_ = (I - K * H_)*P_;



}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	cout << "Calling Update EKF Function " << endl; 
	double px = x_[0];
	double py = x_[1];
	double vx = x_[2];
	double vy = x_[3];
	cout << "px, py, vx, vy: " << px << py << vx << vy << endl;

	double rho = sqrt(pow(px, 2) + pow(py, 2));
	// Check division by zero
	if (rho < 0.00001) {
		rho = sqrt((pow(px, 2) + 0.001) + (pow(py, 2) + 0.001)); 
	}

	double phi = atan2(py,px);
	double rho_dot = ((px*vx) + (py*vy)) / sqrt(pow(px, 2) + pow(py, 2));



	cout << "rho, phi, rho_dot " << rho << "," << phi << "," << rho_dot << endl;
	VectorXd h = VectorXd(3);
	h << rho, phi, rho_dot;
	VectorXd y = z - h;
	cout << "y (before normalization) " << endl;
	cout << y << endl;
	cout << "Value of M_PI:" << M_PI << endl;

	// Normalize the angle
	while (y(1)>M_PI) {
		y(1) -= 2 * M_PI;
	}
	while (y(1)<-M_PI) {
		y(1) += 2 * M_PI;
	}

	cout << "y (after normalizing) " << endl;
	cout << y << endl;


	MatrixXd S = H_ * P_*H_.transpose() + R_;
	cout << "S " << endl;
	cout << S << endl;

	MatrixXd K = P_ * H_.transpose()*S.inverse();
	cout << "K " << endl;
	cout << K << endl;

	x_ = x_ + K * y;
	cout << "x_ " << x_;
	MatrixXd I = MatrixXd::Identity(4, 4);
	P_ = (I - K * H_)*P_;
	cout << "P_ " << P_ << endl;




}
 