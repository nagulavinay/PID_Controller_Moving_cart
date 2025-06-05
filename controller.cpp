/**
 * @file controller.cpp
 * @brief Implementation file for the PIDController class.
 *
 * This file contains the implementation of the PIDController class,
 * which implements a PID (Proportional-Integral-Derivative) controller
 * for controlling the inverted pendulum system.
 *
 * @author Vinay Nagula
 * @date 05/06/2024
 */

#include "controller.h"
#include <iostream>

PIDController::PIDController() { update_params(kp, kd, ki); }

double max_output;
double min_output;

void PIDController::setClamp(double max, double min) {
  ///@todo Implement setClamp for setting the output limits
  setClamp(100,-100); 
  if (u > max) {
    u = max_output;
  } else if (u < min) {
    u = min_output;
  }
}


double PIDController::output(double error) {
  
  ///@todo Implement the PID controller output calculation
  if (step_count == 0) {
    // First timestep
    u = (b0 / a0) * error;
    e_prev[0] = error;
    u_prev[0] = u;
    step_count++;

  } else if (step_count == 1) {
    // Second timestep
    u = (b0 / a0) * error + (b1 / a0) * e_prev[0] - (a1 / a0) * u_prev[0];
    e_prev[1] = e_prev[0];
    e_prev[0]=error;
    u_prev[1] = u_prev[0];
    u_prev[0]=u;
    step_count++;
  } else {
    // Subsequent timesteps
    u = (b0 / a0) * error +
        (b1 / a0) * e_prev[0] +
        (b2 / a0) * e_prev[1] -
        (a1 / a0) * u_prev[0] -
        (a2 / a0) * u_prev[1];

    e_prev[1] = e_prev[0];
    e_prev[0]=error;
    u_prev[1] = u_prev[0];
    u_prev[0]=u;
  }

  // Apply output limits
  
  return u;
    
}

void PIDController::update_params(double kp_, double kd_, double ki_) {
  ///@todo Implement the update_params function for PID controller
  kp = kp_;
  kd = kd_;
  ki = ki_;
  int N = 8, Ts = 0.0005;
  a0 = (1 + N * Ts);
  a1 = -(2 + N * Ts);
  a2 = 1;
  b0 = kp*(1+N*Ts)+ki*Ts*(1+N*Ts) +kd*N;
  b1 = -(kp * (2 + N * Ts) + ki * Ts + 2 * kd * N);
  b2 = kp + kd * N;

}
