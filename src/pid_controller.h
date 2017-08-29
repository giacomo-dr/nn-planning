// pid_controller.h
// Author: Giacomo Del Rio
// Date: 14 Apr 2017

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <iostream>
#include <memory>


typedef long pidTime;       // Time in milliseconds


class PIDController
{
public:
    PIDController( double k_p, double k_i, double k_d, int integral_window_size );
    virtual ~PIDController(){};

    double controlStep( double error, pidTime step_time );
    void setGains( double k_p, double k_i, double k_d );

private:
    double k_p, k_i, k_d;                 // Gains
    double prev_error;                    // Previous error
    pidTime prev_step_time;               // Time of last control step
    int integral_window_size;             // Size of the integral window (in steps)
    int next_window_pos;                  // Position of the next slot to be filled in the integral_window
    double error_integral;                // Cumulative value of integral_window
    std::unique_ptr<double[]> integral_window;  // Integral window
};

#endif // PID_CONTROLLER_H
