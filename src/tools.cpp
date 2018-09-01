#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd RMSE(4);
	RMSE << 0, 0, 0, 0;

	//Perform validation of the input data
	if (estimations.size() == 0 || ground_truth.size() == 0) {
		cout << "Input validation failed: either estimations or ground truth has zero size" << endl;
		return RMSE;
	}
	if (estimations.size() != ground_truth.size()) {
		cout << "Input validation failed: estimations and groud truths have different sizes" << endl;
		return RMSE;
	}

	for (unsigned int i; i < estimations.size(); i++) {
		VectorXd diff = estimations[i] - ground_truth[i];
		diff = pow(diff.array(), 2);
		RMSE = diff + RMSE;
	}
	RMSE = RMSE / estimations.size();
	RMSE = sqrt(RMSE.array());
	return RMSE;




}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
