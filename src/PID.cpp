#include "PID.h"

PID::PID(double Kp_, double Ki_, double Kd_) {
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
}

PID::~PID() {}

void PID::UpdateError(double cte) {
  this->p_error = cte;
  this->i_error += cte;
  this->d_error = cte - prev_cte;
  this->prev_cte = cte;
}

double PID::TotalError() {
  return Kp*p_error + Ki*i_error + Kd*d_error;
}