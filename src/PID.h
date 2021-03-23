#ifndef PID_H
#define PID_H

#include <math.h>
#include <limits>
#include <iostream>


class PID {
 public:

  const double TOL = 0.00001;

  const int UPDATE_CURRENT_COEFFICIENT = 0;
  const int EVALUATE_NEXT_ERROR = 1;
  const int REEVALUATE_NEXT_ERROR = 2;
  const int SWITCH_TO_NEXT_COEFFICIENT = 3;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp, double Kd, double Ki);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Twiddle the coefficients given the performance of the next step
   * with respect to the error.
   * @param cte The current cross track error
   */
  void Twiddle(double cte);


  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /**
   * PID Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;

  double total_error_;

  /**
   * PID Coefficients
   */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
   * Twiddle:
   */

  double dp_[3];
  double best_error_;
  int idx_;
  int state_;
};

#endif  // PID_H
