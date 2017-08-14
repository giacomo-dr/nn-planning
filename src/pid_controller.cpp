// pid_controller.cpp
// Author: Giacomo Del Rio
// Date: 14 Apr 2017

// TODO Implements fixed point arithmetic for integral window

#include "pid_controller.h"


PIDController::PIDController( double k_p, double k_i, double k_d,
                              int integralWindowSize ){
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
    prev_error = 0.0;
    prev_step_time = -1.0;
    integral_window_size = integralWindowSize;
    if( integral_window_size > 0 ){
        integral_window.reset( new double[integral_window_size] );
        for( int i = 0 ; i < integral_window_size ; ++i )
            integral_window[i] = 0.0;
    }
    next_window_pos = 0;
    error_integral = 0.0;
}

double PIDController::controlStep( double error, const pidTime step_time ){
    // On first call, simply setup time
    if( prev_step_time == -1.0 ) {
        prev_step_time = step_time;
        return 0;
    }

    // Compute delta time
    double dt = (step_time - prev_step_time) / 1000.0;
    prev_step_time = step_time;

    // Compute error derivative
    double d_error = (error - prev_error) / dt;

    // Compute error integral
    if( integral_window_size > 0 ){
        error_integral -= integral_window[next_window_pos];
        error_integral += error * dt;
        integral_window[next_window_pos] = error * dt;
        next_window_pos = (next_window_pos + 1) % integral_window_size;
    }

//    std::cout << "PID::Window: [";
//    for( int i = 0 ; i < integral_window_size ; i++ )
//        std::cout << integral_window[i] << ", ";
//    std::cout << "] = " << error_integral << std::endl;
//    std::cout << "PID::Errors: " << error << ", " << d_error << ", " << error_integral << std::endl;
//    std::cout << "PID::Gains:  " << k_p << ", " << k_d << ", " << k_i << std::endl;
//    std::cout << "PID::Comp:   " << (k_p * error) << ", " << (k_d * d_error) << ", " << (k_i * error_integral) << std::endl;

    return k_p * error + k_d * d_error + k_i * error_integral;
}

void PIDController::setGains( double k_p, double k_i, double k_d )
{
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;
}
