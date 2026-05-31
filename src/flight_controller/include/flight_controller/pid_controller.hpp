#pragma once

#include <cmath>
#include <algorithm>

namespace flight_controller {

class PIDController {
public:
    struct Params {
        double kp;
        double ki;
        double kd;
        double imax;
        double deriv_lpf_freq;
        double dt;
    };

    PIDController();
    PIDController(const Params& params);
    
    double update(double error, double dt = 0.0);
    void reset();
    void set_params(const Params& params);
    void set_imax(double imax);
    
    double get_output() const { return output_; }
    double get_integral() const { return integral_; }
    double get_derivative() const { return derivative_; }

private:
    Params params_;
    double error_prev_;
    double integral_;
    double derivative_;
    double output_;
    double deriv_lpf_alpha_;
    
    double apply_deriv_lpf(double deriv);
};

}  // namespace flight_controller
