#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in) {
	p_error = 0;
	i_error = 0;
	d_error = 0;

	/*
	* Coefficients
	*/
	Kp = Kp_in;
	Ki = Ki_in;
	Kd = Kd_in;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error = i_error + cte;
}

double PID::TotalError() {

	double total_error = -Kp * p_error - Ki * i_error - Kd * d_error;
	return total_error;
}

