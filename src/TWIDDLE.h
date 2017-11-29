#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

class TWIDDLE {
public:

  // Vector of parameters in scale of change
  std::vector<double> params;
  std::vector<double> d_params;

  // Best error
  double best_error;

  // Tolerance used to stop
  double tolerance;

  // NUmber of parameters to tune
  int n_parameters;

  // Used to control in which part of the algorithm we are
  int auxiliar;

  // Used to control which parameter is being tuned
  int idx_control;

  bool initialized;

    /*
  * Constructor
  */
  TWIDDLE();

  /*
  * Destructor.
  */
  virtual ~TWIDDLE();

  /*
  * Initialize the optimizer
  */
  void Init(double tol, int num_param);

  /*
  * Returns a set of parameters to test given the error in the last trial
  */
  std::vector<double> get_Parameters();

  void UpdateError(double error);

  /*
  * Returns true if the parameters have already been tuned
  */
  bool is_Over();
};

#endif /* TWIDDLE_H */
