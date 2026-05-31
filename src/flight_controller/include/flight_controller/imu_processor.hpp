#pragma once

#include <array>
#include <cmath>
#include <vector>

namespace flight_controller {

struct IMUData {
    double gyro_x, gyro_y, gyro_z;
    double accel_x, accel_y, accel_z;
    double mag_x, mag_y, mag_z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct Quaternion {
    double w, x, y, z;
    
    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    Quaternion(double w_, double x_, double y_, double z_) 
        : w(w_), x(x_), y(y_), z(z_) {}
    
    void normalize();
};

class IMUProcessor {
public:
    struct Config {
        double gyro_bias_x, gyro_bias_y, gyro_bias_z;
        double accel_bias_x, accel_bias_y, accel_bias_z;
        double complementary_filter_gain;
    };

    IMUProcessor();
    explicit IMUProcessor(const Config& config);
    
    void update(const IMUData& imu_data, double dt);
    
    EulerAngles get_attitude() const { return attitude_; }
    Quaternion get_quaternion() const { return quaternion_; }
    
    std::array<double, 3> get_angular_velocity() const;
    std::array<double, 3> get_linear_acceleration() const;
    
    void set_biases(double gx, double gy, double gz, 
                   double ax, double ay, double az);
    
    void calibrate_gyro(const std::vector<IMUData>& samples);
    
    void set_config(const Config& config) { config_ = config; }

private:
    Config config_;
    EulerAngles attitude_;
    Quaternion quaternion_;
    IMUData last_imu_data_;
    
    EulerAngles compute_accel_attitude(const IMUData& imu_data);
    void rotate_quaternion_by_gyro(const IMUData& imu_data, double dt);
    
    static EulerAngles quaternion_to_euler(const Quaternion& q);
    static Quaternion euler_to_quaternion(const EulerAngles& e);
    static Quaternion quaternion_multiply(const Quaternion& a, const Quaternion& b);
    static double clamp(double value, double min_val, double max_val);
};

}  // namespace flight_controller
