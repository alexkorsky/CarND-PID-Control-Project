#include "PID.h"

using std::string;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki)
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

	Kp_ = Kp;
	Kd_ = Kd;
	Ki_ = Ki;

	p_error_ = 0;
	i_error_ = 0;
	d_error_ = 0;


	// TWIDDLE:

	dp_[0] = 0.1;
	dp_[1] = 1;
	dp_[2] = 0.001;

//	dp_[0] = 1;
//	dp_[1] = 1;
//	dp_[2] = 1;

	best_error_ = std::numeric_limits<double>::max();
	idx_ = 0;
	state_ = UPDATE_CURRENT_COEFFICIENT;
}

void PID::UpdateError(double cte)
{
  /**
   * TODO: Update PID errors based on cte.
   */

	  d_error_ = cte - p_error_;
	  p_error_ = cte;
	  i_error_ += cte;
}

double PID::TotalError()
{
  /**
   * TODO: Calculate and return the total error
   */


//	double result;
//
//
//	// progressive PID
//    if (fabs(p_error_) < 0.2)
//    	result = 0;
//    else if (fabs(p_error_) < 2) // 70% steer
//    	result = -Kp_ * 0.7 * p_error_ - Kd_ * 0.7 * d_error_ - Ki_ * i_error_;
//    else if (fabs(p_error_) < 5) // 80% steer
//    	result = -Kp_ * 0.8 * p_error_ - Kd_ * 0.8 * d_error_ - Ki_ * i_error_;
//    else if (fabs(p_error_) < 8) // 90% steer
//    	result = -Kp_ * 0.9 * p_error_ - Kd_ * 0.9 * d_error_ - Ki_ * i_error_;
//    else // 100%
//    	result = -Kp_ * 1.0 * p_error_ - Kd_ * 1.0 * d_error_ - Ki_ * i_error_;
//
//    if(result > 1.0)
//    	result = 1.0;
//    else if(result < -1.0)
//    	result = -1.0;
//
//
//    return result;


    // non-progressive PID
    return -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;

}

void PID::Twiddle(double cte)
{
	double p[] = {Kp_, Kd_, Ki_};


	std::string  STATE_STR;
	if (state_ == UPDATE_CURRENT_COEFFICIENT)
	{
		STATE_STR = "UPDATE_CURRENT_COEFFICIENT";
	}
	else if (state_ ==  EVALUATE_NEXT_ERROR)
	{
		STATE_STR = "EVALUATE_NEXT_ERROR";
	}
	else if (state_ ==  REEVALUATE_NEXT_ERROR)
	{
		STATE_STR = "REEVALUATE_NEXT_ERROR";
	}
	else if (state_ == SWITCH_TO_NEXT_COEFFICIENT)
	{
		STATE_STR = "SWITCH_TO_NEXT_COEFFICIENT";
	}

	std::string  INDEX_STR;
	if (idx_ == 0)
	{
		INDEX_STR = "P";
	}
	else if (idx_ ==  1)
	{
		INDEX_STR = "D";
	}
	else if (idx_ ==  2)
	{
		INDEX_STR = "I";
	}

	std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << "p_error_: " << p_error_ << std::endl;
    std::cout << "d_error_: " << d_error_ << std::endl;
    std::cout << "i_error_: " << i_error_ << std::endl;

    std::cout << "CTE: " << cte << std::endl;

    std::cout << "state_: " << STATE_STR << std::endl;
    std::cout << "idx_: " << INDEX_STR  << std::endl;


    std::cout << "BestError: " << best_error_ << std::endl;

	if ((fabs(dp_[0]) + fabs(dp_[1]) + fabs(dp_[2])) > TOL)
	{

		bool getNextTelemetry = false;

		while (!getNextTelemetry)
		{
			if (state_ == UPDATE_CURRENT_COEFFICIENT)
			{
				p[idx_] += dp_[idx_];

				state_ = EVALUATE_NEXT_ERROR;
				getNextTelemetry = true;
			}
			else if (state_ ==  EVALUATE_NEXT_ERROR)
			{
				if (fabs(cte) < best_error_)
				{
					best_error_ = fabs(cte);
					dp_[idx_] *= 1.1;

					state_ = SWITCH_TO_NEXT_COEFFICIENT;
				}
				else
				{
					p[idx_] -= 2 * dp_[idx_];

					state_ = REEVALUATE_NEXT_ERROR;
					getNextTelemetry = true;
				}
			}
			else if (state_ ==  REEVALUATE_NEXT_ERROR)
			{
				if (fabs(cte) < best_error_)
				{
					best_error_ = fabs(cte);
					dp_[idx_] *= 1.1;
				}
				else
				{
					p[idx_] += dp_[idx_];
					dp_[idx_] *= 0.9;
				}

				state_ = SWITCH_TO_NEXT_COEFFICIENT;
			}


			if (state_ == SWITCH_TO_NEXT_COEFFICIENT)
			{
				idx_ = (idx_ + 1) % 2;


				state_ = UPDATE_CURRENT_COEFFICIENT;

				if (idx_== 0)
					getNextTelemetry = true;
			}
		}

		Kp_ = p[0];
		Kd_ = p[1];
		Ki_ = p[2];

		std::cout << "Adjusters [ dp[0]: " << dp_[0] << " | dp[1]: " << dp_[1] << " | dp[2]: " << dp_[2] << "]" << std::endl;
		std::cout << "TWIDDLE [ Kp: " << Kp_ << " | Kd: " << Kd_ << " | Ki: " << Ki_ << "]" << std::endl;
	}
}
