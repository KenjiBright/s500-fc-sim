#include "flight_controller/flight_controller_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace flight_controller {

FlightControllerNode::FlightControllerNode()
    : rclcpp::Node("flight_controller") {
    
    RCLCPP_INFO(this->get_logger(), "Initializing Flight Controller Node");
    
    load_config();
    
    IMUProcessor::Config imu_config;
    imu_config.complementary_filter_gain = 0.02;
    imu_processor_ = std::make_unique<IMUProcessor>(imu_config);
    
    MotorMixer::Config mixer_config;
    mixer_config.max_tilt_angle_rad = 45.0 * M_PI / 180.0;
    mixer_config.min_pwm = 1000.0;
    mixer_config.max_pwm = 2000.0;
    mixer_config.hover_pwm = 1500.0;
    motor_mixer_ = std::make_unique<MotorMixer>(mixer_config);
    
    PIDController::Params pid_roll_rate_params{0.15, 0.08, 0.004, 100.0, 20.0, 0.0025};
    pid_roll_rate_ = PIDController(pid_roll_rate_params);
    
    PIDController::Params pid_pitch_rate_params{0.15, 0.08, 0.004, 100.0, 20.0, 0.0025};
    pid_pitch_rate_ = PIDController(pid_pitch_rate_params);
    
    PIDController::Params pid_yaw_rate_params{0.3, 0.05, 0.0, 100.0, 20.0, 0.0025};
    pid_yaw_rate_ = PIDController(pid_yaw_rate_params);
    
    PIDController::Params pid_roll_angle_params{4.5, 0.0, 0.0, 100.0, 20.0, 0.0025};
    pid_roll_angle_ = PIDController(pid_roll_angle_params);
    
    PIDController::Params pid_pitch_angle_params{4.5, 0.0, 0.0, 100.0, 20.0, 0.0025};
    pid_pitch_angle_ = PIDController(pid_pitch_angle_params);
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data",
        rclcpp::SensorDataQoS(),
        std::bind(&FlightControllerNode::imu_callback, this, std::placeholders::_1)
    );
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::SensorDataQoS(),
        std::bind(&FlightControllerNode::cmd_vel_callback, this, std::placeholders::_1)
    );
    
    motor_pwm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "motor_pwm_command",
        rclcpp::SensorDataQoS()
    );
    
    double dt = 1.0 / config_.control_loop_rate;
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&FlightControllerNode::control_loop, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Flight Controller initialized successfully");
}

void FlightControllerNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    IMUData imu_data;
    imu_data.gyro_x = msg->angular_velocity.x;
    imu_data.gyro_y = msg->angular_velocity.y;
    imu_data.gyro_z = msg->angular_velocity.z;
    imu_data.accel_x = msg->linear_acceleration.x;
    imu_data.accel_y = msg->linear_acceleration.y;
    imu_data.accel_z = msg->linear_acceleration.z;
    
    rclcpp::Time msg_time = msg->header.stamp;
    double dt = 1.0 / 100.0;
    if (imu_time_initialized_) {
        double computed_dt = (msg_time - last_imu_time_).seconds();
        if (computed_dt > 0.0 && computed_dt < 0.1) {
            dt = computed_dt;
        }
    }
    last_imu_time_ = msg_time;
    imu_time_initialized_ = true;

    imu_processor_->update(imu_data, dt);
    
    state_.attitude = imu_processor_->get_attitude();
    state_.gyro = imu_processor_->get_angular_velocity();
    state_.accel = imu_processor_->get_linear_acceleration();
}

void FlightControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double max_angle = config_.max_angle_deg * M_PI / 180.0;
    double max_vel = 2.0;
    
    state_.target_pitch = (msg->linear.x / max_vel) * max_angle;
    state_.target_roll = (msg->linear.y / max_vel) * max_angle;
    state_.target_yaw_rate = msg->angular.z;
    
    state_.throttle = 0.5 + (msg->linear.z / 2.0);
    state_.throttle = std::clamp(state_.throttle, 0.0, 1.0);
}

void FlightControllerNode::control_loop() {
    double dt = 1.0 / config_.control_loop_rate;
    
    if (config_.stabilize_mode) {
        double roll_rate_cmd = pid_roll_angle_.update(state_.target_roll - state_.attitude.roll, dt);
        double pitch_rate_cmd = pid_pitch_angle_.update(state_.target_pitch - state_.attitude.pitch, dt);
        
        double roll_out  = pid_roll_rate_.update(roll_rate_cmd - state_.gyro[0], dt);
        double pitch_out = pid_pitch_rate_.update(pitch_rate_cmd - state_.gyro[1], dt);
        double yaw_out   = pid_yaw_rate_.update(state_.target_yaw_rate - state_.gyro[2], dt);
        
        AttitudeSetpoints setpoints;
        setpoints.roll_rad = roll_out;
        setpoints.pitch_rad = pitch_out;
        setpoints.yaw_rate_rad = yaw_out;
        setpoints.throttle = state_.throttle;
        
        MotorCommands motor_cmd = motor_mixer_->mix(
            setpoints,
            state_.attitude.roll,
            state_.attitude.pitch,
            state_.attitude.yaw
        );
        
        auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        msg->data.resize(4);
        for (int i = 0; i < 4; ++i) {
            msg->data[i] = motor_cmd.pwm[i];
        }
        motor_pwm_pub_->publish(std::move(msg));
    }
}

void FlightControllerNode::load_config() {
    this->declare_parameter("control_loop_rate", 400.0);
    this->declare_parameter("max_angle_deg", 45.0);
    this->declare_parameter("stabilize_mode", true);
    
    config_.control_loop_rate = this->get_parameter("control_loop_rate").as_double();
    config_.max_angle_deg = this->get_parameter("max_angle_deg").as_double();
    config_.stabilize_mode = this->get_parameter("stabilize_mode").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Config loaded: rate=%.1f Hz, max_angle=%.1f deg",
                config_.control_loop_rate, config_.max_angle_deg);
}

}  // namespace flight_controller

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<flight_controller::FlightControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
