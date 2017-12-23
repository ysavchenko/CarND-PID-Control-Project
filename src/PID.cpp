#include "PID.h"
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    i_error = 0;
    prev_cte = numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
    p_error = cte;
    i_error += cte;
    if (prev_cte == numeric_limits<double>::max()) {
        d_error = 0;
    } else {
        d_error = cte - prev_cte;
    }
    prev_cte = cte;
}

double PID::GetCorrection() {
    return -(Kd * d_error + Kp * p_error + Ki * i_error);
}
