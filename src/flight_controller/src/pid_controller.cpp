#include "flight_controller/pid_controller.hpp"

namespace flight_controller {

PIDController::PIDController()
    : error_prev_(0.0), integral_(0.0), derivative_(0.0), output_(0.0),
      deriv_lpf_alpha_(0.0) {
    params_.kp = 0.0;
    params_.ki = 0.0;
    params_.kd = 0.0;
    params_.imax = 0.0;
    params_.deriv_lpf_freq = 20.0;
    params_.dt = 0.0025;
}

PIDController::PIDController(const Params& params)
    : params_(params), error_prev_(0.0), integral_(0.0), 
      derivative_(0.0), output_(0.0) {
    double wc = 2.0 * M_PI * params_.deriv_lpf_freq * params_.dt;
    deriv_lpf_alpha_ = wc / (wc + 1.0);
}

double PIDController::update(double error, double dt) {
    if (dt > 0.0) {
        params_.dt = dt;
    }
    
    double p_term = params_.kp * error;
    
    integral_ += params_.ki * error * params_.dt;
    integral_ = std::clamp(integral_, -params_.imax, params_.imax);
    
    double raw_deriv = (error - error_prev_) / params_.dt;
    derivative_ = apply_deriv_lpf(raw_deriv);
    double d_term = params_.kd * derivative_;
    
    output_ = p_term + integral_ + d_term;
    
    error_prev_ = error;
    
    return output_;
}

void PIDController::reset() {
    error_prev_ = 0.0;
    integral_ = 0.0;
    derivative_ = 0.0;
    output_ = 0.0;
}

void PIDController::set_params(const Params& params) {
    params_ = params;
    double wc = 2.0 * M_PI * params_.deriv_lpf_freq * params_.dt;
    deriv_lpf_alpha_ = wc / (wc + 1.0);
}

void PIDController::set_imax(double imax) {
    params_.imax = imax;
}

double PIDController::apply_deriv_lpf(double deriv) {
    static double deriv_filtered = 0.0;
    deriv_filtered = deriv_lpf_alpha_ * deriv + (1.0 - deriv_lpf_alpha_) * deriv_filtered;
    return deriv_filtered;
}

}  // namespace flight_controller
