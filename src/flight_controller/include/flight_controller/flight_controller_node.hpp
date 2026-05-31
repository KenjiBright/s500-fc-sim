#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <array>

#include "pid_controller.hpp"
#include "motor_mixer.hpp"
#include "imu_processor.hpp"

namespace flight_controller {

class FlightControllerNode : public rclcpp::Node {
public:
    FlightControllerNode();
    virtual ~FlightControllerNode() = default;

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pwm_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::unique_ptr<IMUProcessor> imu_processor_;
    std::unique_ptr<MotorMixer> motor_mixer_;
    
    PIDController pid_roll_rate_;
    PIDController pid_pitch_rate_;
    PIDController pid_yaw_rate_;
    PIDController pid_roll_angle_;
    PIDController pid_pitch_angle_;
    
    struct {
        EulerAngles attitude;
        std::array<double, 3> gyro;
        std::array<double, 3> accel;
        double throttle = 0.0;
        double target_roll = 0.0;
        double target_pitch = 0.0;
        double target_yaw_rate = 0.0;
    } state_;
    
    struct {
        double control_loop_rate = 400.0;
        double max_angle_deg = 45.0;
        bool stabilize_mode = true;
    } config_;
    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void control_loop();
    void load_config();
};

}  // namespace flight_controller
