#include "TWIDDLE.h"
#include <math.h>
#include <vector>
#include <algorithm>

using namespace std;

TWIDDLE::TWIDDLE() {}

TWIDDLE::~TWIDDLE() {}

void TWIDDLE::Init(double tol, int num_param){
  tolerance = tol;
  n_parameters = num_param;

  for(int i=0;i<num_param;++i){
    params.push_back(0.0);
    d_params.push_back(1.0);
  }

  initialized = false;

  idx_control = 0;
  best_error = numeric_limits<double>::max();
  auxiliar = 0;
}

void TWIDDLE::UpdateError(double error){
  if(!initialized){
    initialized = true;
    best_error = error;
  }

  if(auxiliar == 0){
    params[idx_control] += d_params[idx_control];
    auxiliar += 1;
  }
  else{
    if(auxiliar == 1){
      if(error < best_error){
        best_error = error;
        d_params[idx_control] *= 1.1;

        auxiliar = 0;

        // Cycle parameters
        idx_control = (idx_control+1) % n_parameters;
      }
      else{
        params[idx_control] -= 2*d_params[idx_control];
        auxiliar += 1;
      }
    }
    else{
      if(error < best_error){
        best_error = error;
        d_params[idx_control] *= 1.1;
      }
      else{
        params[idx_control] += d_params[idx_control];
        d_params[idx_control] *= 0.9;
      }
      auxiliar = 0;

      // Cycle parameters
      idx_control = (idx_control+1) % n_parameters;
    }
  }
}

vector<double> TWIDDLE::get_Parameters(){
  return params;
}

bool TWIDDLE::is_Over(){
  double sum_dp = 0.0;
  for(int i=0; i<n_parameters; ++i){
    sum_dp += d_params[i];
  }

  return (sum_dp < tolerance);
}
