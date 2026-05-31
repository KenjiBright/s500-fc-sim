#pragma once

#include <array>
#include <cmath>

namespace flight_controller {

struct MotorCommands {
    std::array<double, 4> pwm;
    
    MotorCommands() : pwm({1000, 1000, 1000, 1000}) {}
};

struct AttitudeSetpoints {
    double roll_rad;
    double pitch_rad;
    double yaw_rate_rad;
    double throttle;
};

class MotorMixer {
public:
    struct Config {
        double max_tilt_angle_rad;
        double min_pwm;
        double max_pwm;
        double hover_pwm;
    };

    MotorMixer();
    explicit MotorMixer(const Config& config);
    
    MotorCommands mix(
        const AttitudeSetpoints& setpoints,
        double roll_rad,
        double pitch_rad,
        double yaw_rad
    );
    
    void set_config(const Config& config) { config_ = config; }
    const MotorCommands& get_motor_commands() const { return motor_commands_; }
    
    static void clamp_motor_outputs(
        MotorCommands& commands,
        double min_pwm,
        double max_pwm
    );

private:
    Config config_;
    MotorCommands motor_commands_;
    
    static double constrain(double value, double min_val, double max_val);
    
    std::array<double, 4> calculate_motor_outputs(
        double throttle,
        double roll,
        double pitch,
        double yaw
    );
};

}  // namespace flight_controller
