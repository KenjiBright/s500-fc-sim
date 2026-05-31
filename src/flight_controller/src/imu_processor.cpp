#include "flight_controller/imu_processor.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace flight_controller {

void Quaternion::normalize() {
    double norm = std::sqrt(w*w + x*x + y*y + z*z);
    if (norm > 0.0) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
}

IMUProcessor::IMUProcessor() {
    config_.gyro_bias_x = 0.0;
    config_.gyro_bias_y = 0.0;
    config_.gyro_bias_z = 0.0;
    config_.accel_bias_x = 0.0;
    config_.accel_bias_y = 0.0;
    config_.accel_bias_z = 0.0;
    config_.complementary_filter_gain = 0.02;
}

IMUProcessor::IMUProcessor(const Config& config) : config_(config) {
    attitude_ = EulerAngles{0.0, 0.0, 0.0};
    quaternion_ = Quaternion(1.0, 0.0, 0.0, 0.0);
}

void IMUProcessor::update(const IMUData& imu_data, double dt) {
    IMUData corrected_imu = imu_data;
    corrected_imu.gyro_x -= config_.gyro_bias_x;
    corrected_imu.gyro_y -= config_.gyro_bias_y;
    corrected_imu.gyro_z -= config_.gyro_bias_z;
    corrected_imu.accel_x -= config_.accel_bias_x;
    corrected_imu.accel_y -= config_.accel_bias_y;
    corrected_imu.accel_z -= config_.accel_bias_z;
    
    rotate_quaternion_by_gyro(corrected_imu, dt);
    
    EulerAngles accel_attitude = compute_accel_attitude(corrected_imu);
    Quaternion accel_quat = euler_to_quaternion(accel_attitude);
    
    double alpha = config_.complementary_filter_gain;
    
    Quaternion filtered_quat;
    filtered_quat.w = (1.0 - alpha) * quaternion_.w + alpha * accel_quat.w;
    filtered_quat.x = (1.0 - alpha) * quaternion_.x + alpha * accel_quat.x;
    filtered_quat.y = (1.0 - alpha) * quaternion_.y + alpha * accel_quat.y;
    filtered_quat.z = (1.0 - alpha) * quaternion_.z + alpha * accel_quat.z;
    filtered_quat.normalize();
    
    quaternion_ = filtered_quat;
    attitude_ = quaternion_to_euler(quaternion_);
    last_imu_data_ = imu_data;
}

EulerAngles IMUProcessor::compute_accel_attitude(const IMUData& imu_data) {
    EulerAngles angles;
    
    angles.roll = std::atan2(imu_data.accel_y, imu_data.accel_z);
    
    double accel_mag = std::sqrt(imu_data.accel_x * imu_data.accel_x +
                                 imu_data.accel_y * imu_data.accel_y +
                                 imu_data.accel_z * imu_data.accel_z);
    if (accel_mag > 0.1) {
        angles.pitch = -std::asin(imu_data.accel_x / accel_mag);
    }
    
    angles.yaw = attitude_.yaw;
    
    return angles;
}

void IMUProcessor::rotate_quaternion_by_gyro(const IMUData& imu_data, double dt) {
    double half_dt = 0.5 * dt;
    double wx = imu_data.gyro_x * half_dt;
    double wy = imu_data.gyro_y * half_dt;
    double wz = imu_data.gyro_z * half_dt;
    
    Quaternion delta_quat;
    delta_quat.w = 1.0;
    delta_quat.x = wx;
    delta_quat.y = wy;
    delta_quat.z = wz;
    
    quaternion_ = quaternion_multiply(quaternion_, delta_quat);
    quaternion_.normalize();
}

EulerAngles IMUProcessor::quaternion_to_euler(const Quaternion& q) {
    EulerAngles angles;
    
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp);
    else
        angles.pitch = std::asin(sinp);
    
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return angles;
}

Quaternion IMUProcessor::euler_to_quaternion(const EulerAngles& e) {
    double cy = std::cos(e.yaw * 0.5);
    double sy = std::sin(e.yaw * 0.5);
    double cp = std::cos(e.pitch * 0.5);
    double sp = std::sin(e.pitch * 0.5);
    double cr = std::cos(e.roll * 0.5);
    double sr = std::sin(e.roll * 0.5);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

Quaternion IMUProcessor::quaternion_multiply(const Quaternion& a, const Quaternion& b) {
    Quaternion result;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return result;
}

std::array<double, 3> IMUProcessor::get_angular_velocity() const {
    return {last_imu_data_.gyro_x, last_imu_data_.gyro_y, last_imu_data_.gyro_z};
}

std::array<double, 3> IMUProcessor::get_linear_acceleration() const {
    return {last_imu_data_.accel_x, last_imu_data_.accel_y, last_imu_data_.accel_z};
}

void IMUProcessor::set_biases(double gx, double gy, double gz,
                              double ax, double ay, double az) {
    config_.gyro_bias_x = gx;
    config_.gyro_bias_y = gy;
    config_.gyro_bias_z = gz;
    config_.accel_bias_x = ax;
    config_.accel_bias_y = ay;
    config_.accel_bias_z = az;
}

void IMUProcessor::calibrate_gyro(const std::vector<IMUData>& samples) {
    if (samples.empty()) return;
    
    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    for (const auto& sample : samples) {
        sum_gx += sample.gyro_x;
        sum_gy += sample.gyro_y;
        sum_gz += sample.gyro_z;
    }
    
    config_.gyro_bias_x = sum_gx / samples.size();
    config_.gyro_bias_y = sum_gy / samples.size();
    config_.gyro_bias_z = sum_gz / samples.size();
}

double IMUProcessor::clamp(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

}  // namespace flight_controller
