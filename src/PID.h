#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double cum_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  bool initialized;
  double prev_cte;

  double step_counter;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double K_p, double K_d, double K_i, double evaluation_steps);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Returns the control action of the PID
  */
  double get_ControlValue();

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
