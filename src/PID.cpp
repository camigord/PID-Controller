#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double K_p, double K_d, double K_i, double evaluation_steps) {
  Kp = K_p;
  Ki = K_i;
  Kd = K_d;

  i_error = 0.0;
  p_error = 0.0;
  d_error = 0.0;
  prev_cte = 0.0;
  initialized = false;

  step_counter = evaluation_steps;
  cum_error = 0.0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte;

  if(!initialized){
    initialized = true;
    prev_cte = cte;
  }

  d_error = cte - prev_cte;
  prev_cte = cte;

  cum_error += cte * cte;
  step_counter--;
}

double PID::TotalError() {
  return cum_error;
}

double PID::get_ControlValue() {
  return (-1.0*Kp*p_error) + (-1.0*Kd*d_error) + (-1.0*Ki*i_error);
}
