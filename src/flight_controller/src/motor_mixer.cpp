#include "flight_controller/motor_mixer.hpp"
#include <cmath>
#include <algorithm>

namespace flight_controller {

MotorMixer::MotorMixer() {
    config_.max_tilt_angle_rad = 45.0 * M_PI / 180.0;
    config_.min_pwm = 1000.0;
    config_.max_pwm = 2000.0;
    config_.hover_pwm = 1500.0;
}

MotorMixer::MotorMixer(const Config& config)
    : config_(config) {}

MotorCommands MotorMixer::mix(
    const AttitudeSetpoints& setpoints,
    double roll_rad,
    double pitch_rad,
    double yaw_rad) {
    
    double throttle_pwm = config_.hover_pwm + 
                         (setpoints.throttle - 0.5) * (config_.max_pwm - config_.min_pwm);
    throttle_pwm = constrain(throttle_pwm, config_.min_pwm, config_.max_pwm);
    
    double desired_roll = constrain(setpoints.roll_rad, 
                                    -config_.max_tilt_angle_rad, 
                                    config_.max_tilt_angle_rad);
    double desired_pitch = constrain(setpoints.pitch_rad, 
                                     -config_.max_tilt_angle_rad, 
                                     config_.max_tilt_angle_rad);
    
    double roll_cmd = desired_roll / config_.max_tilt_angle_rad;
    double pitch_cmd = desired_pitch / config_.max_tilt_angle_rad;
    double yaw_cmd = setpoints.yaw_rate_rad / (360.0 * M_PI / 180.0);
    
    roll_cmd = constrain(roll_cmd, -1.0, 1.0);
    pitch_cmd = constrain(pitch_cmd, -1.0, 1.0);
    yaw_cmd = constrain(yaw_cmd, -1.0, 1.0);
    
    double throttle_norm = (throttle_pwm - config_.min_pwm) / 
                          (config_.max_pwm - config_.min_pwm);
    throttle_norm = constrain(throttle_norm, 0.0, 1.0);
    
    auto motor_outputs = calculate_motor_outputs(throttle_norm, roll_cmd, pitch_cmd, yaw_cmd);
    
    for (int i = 0; i < 4; ++i) {
        motor_commands_.pwm[i] = config_.min_pwm + 
                                 motor_outputs[i] * (config_.max_pwm - config_.min_pwm);
    }
    
    clamp_motor_outputs(motor_commands_, config_.min_pwm, config_.max_pwm);
    
    return motor_commands_;
}

std::array<double, 4> MotorMixer::calculate_motor_outputs(
    double throttle,
    double roll,
    double pitch,
    double yaw) {
    
    std::array<double, 4> outputs;
    
    // X-frame mixing: FR(CW), FL(CCW), RR(CCW), RL(CW)
    // +roll = roll right (left side up): FL/RL increase, FR/RR decrease
    // +pitch = nose up: RR/RL increase, FR/FL decrease
    // +yaw = CCW from above: CW motors (FR,RL) increase, CCW motors (FL,RR) decrease
    outputs[0] = throttle - roll - pitch + yaw;  // FR
    outputs[1] = throttle + roll - pitch - yaw;  // FL
    outputs[2] = throttle - roll + pitch - yaw;  // RR
    outputs[3] = throttle + roll + pitch + yaw;  // RL
    
    double max_output = 0.0;
    for (int i = 0; i < 4; ++i) {
        max_output = std::max(max_output, std::abs(outputs[i]));
    }
    
    if (max_output > 1.0) {
        for (int i = 0; i < 4; ++i) {
            outputs[i] /= max_output;
        }
    }
    
    for (int i = 0; i < 4; ++i) {
        outputs[i] = constrain(outputs[i], 0.0, 1.0);
    }
    
    return outputs;
}

void MotorMixer::clamp_motor_outputs(
    MotorCommands& commands,
    double min_pwm,
    double max_pwm) {
    
    for (int i = 0; i < 4; ++i) {
        commands.pwm[i] = constrain(commands.pwm[i], min_pwm, max_pwm);
    }
}

double MotorMixer::constrain(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

}  // namespace flight_controller
