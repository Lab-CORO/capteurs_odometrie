/*
    MPU6050 Interfacing with Raspberry Pi - ROS2 Version
    Adapted from ROS1 code by jproberge (January 2022)
*/

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <memory>

// WiringPi includes
#include <wiringPiI2C.h>
#include <wiringPi.h>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// tf2 includes for quaternion and matrix operations
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

// Macros and constants
#define Device_Address 0x68   /* Device Address/Identifier for MPU6050 */
const int MPU6050_GndPin = 4;

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

using namespace std::chrono_literals;

class MPU6050Node : public rclcpp::Node
{
public:
  MPU6050Node() : Node("centrale_inertielle"),
                  got_imu_data_(false),
                  fixed_imu_data_(false),
                  init_yaw_(0.0),
                  yaw_bias_(0.0),
                  bias_calculation_iterations_(500),
                  bias_calculation_iterator_(0),
                  norm_bias_(0.0),
                  ax_mean_(0.0), ay_mean_(0.0), az_mean_(0.0),
                  acc_coeff_(0.0),
                  gx_bias_(0.0), gy_bias_(0.0), gz_bias_(0.0)
  {
    // Create publishers and subscribers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
    fixed_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_reoriented", 1);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 1,
      std::bind(&MPU6050Node::imu_orientation_fixer_callback, this, std::placeholders::_1));
    imu_yaw_bias_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "set_pose", 1,
      std::bind(&MPU6050Node::imu_adjust_yaw_callback, this, std::placeholders::_1));

    // Create a Transform Broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize wiringPi and MPU6050 hardware
    fd_ = wiringPiI2CSetup(Device_Address);
    wiringPiSetupGpio();
    pinMode(MPU6050_GndPin, OUTPUT);
    digitalWrite(MPU6050_GndPin, LOW);
    delay(200);  // allow time for sensor power-up
    MPU6050_Init();

    // Initialize a transform stamp (for broadcasting)
    transform_stamped_.header.frame_id = "r_robot";
    transform_stamped_.child_frame_id = "imu_frame";
    transform_stamped_.transform.translation.x = 0.0;
    transform_stamped_.transform.translation.y = 0.0;
    transform_stamped_.transform.translation.z = 0.0;

    // Start a timer to run at 100Hz
    timer_ = this->create_wall_timer(50ms, std::bind(&MPU6050Node::timer_callback, this));
  }

private:
  // Publisher and Subscriber objects
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fixed_imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr imu_yaw_bias_sub_;

  // Timer for main loop
  rclcpp::TimerBase::SharedPtr timer_;

  // tf2 Transform Broadcaster and message to broadcast transform
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_stamped_;

  // State variables
  bool got_imu_data_;
  bool fixed_imu_data_;
  double init_yaw_;
  double yaw_bias_;
  sensor_msgs::msg::Imu fixed_imu_msg_;

  // Bias calculation variables
  const int bias_calculation_iterations_;
  int bias_calculation_iterator_;
  double norm_bias_;
  double ax_mean_, ay_mean_, az_mean_, acc_coeff_;
  double gx_bias_, gy_bias_, gz_bias_;

  // I2C file descriptor
  int fd_;

  // Initialize MPU6050 registers
  void MPU6050_Init()
  {
    wiringPiI2CWriteReg8(fd_, SMPLRT_DIV, 0x07);    // Sample rate register
    wiringPiI2CWriteReg8(fd_, PWR_MGMT_1, 0x01);      // Power management register
    wiringPiI2CWriteReg8(fd_, CONFIG, 0);             // Configuration register
    wiringPiI2CWriteReg8(fd_, ACCEL_CONFIG, 0x00);    // Accel Configuration register
    wiringPiI2CWriteReg8(fd_, GYRO_CONFIG, 0x00);     // Gyro Configuration register
    wiringPiI2CWriteReg8(fd_, INT_ENABLE, 0x01);      // Interrupt enable register
  }

  // Read two bytes from a register and combine into a short
  short read_raw_data(int addr)
  {
    short high_byte = wiringPiI2CReadReg8(fd_, addr);
    short low_byte = wiringPiI2CReadReg8(fd_, addr + 1);
    short value = (high_byte << 8) | low_byte;
    return value;
  }

  // Callback for IMU orientation fixer (subscribed to "imu/data")
  void imu_orientation_fixer_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!got_imu_data_) {
      RCLCPP_INFO(this->get_logger(), "Received first IMU data");
      // Set transform origin (remains zero) and store initial yaw
      tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);
      double pitch, roll, yaw;
      tf2::Matrix3x3 mat(q);
      mat.getEulerYPR(init_yaw_, pitch, roll);
      // Reset quaternion with roll and pitch only (zero yaw)
      q.setRPY(roll, pitch, 0);
      // Save rotation in transform_stamped_
      transform_stamped_.transform.rotation.x = q.x();
      transform_stamped_.transform.rotation.y = q.y();
      transform_stamped_.transform.rotation.z = q.z();
      transform_stamped_.transform.rotation.w = q.w();
      got_imu_data_ = true;
    } else {
      // Process new IMU data and update fixed orientation
      fixed_imu_msg_ = *msg;
      tf2::Quaternion fixed_q(msg->orientation.x, msg->orientation.y,
                              msg->orientation.z, msg->orientation.w);
      double fixed_pitch, fixed_roll, fixed_yaw;
      tf2::Matrix3x3 fixed_mat(fixed_q);
      fixed_mat.getEulerYPR(fixed_yaw, fixed_pitch, fixed_roll);
      fixed_yaw = fixed_yaw - init_yaw_ + yaw_bias_;
      fixed_q.setRPY(fixed_roll, fixed_pitch, fixed_yaw);
      fixed_imu_msg_.orientation.x = fixed_q.x();
      fixed_imu_msg_.orientation.y = fixed_q.y();
      fixed_imu_msg_.orientation.z = fixed_q.z();
      fixed_imu_msg_.orientation.w = fixed_q.w();
      // Set orientation covariance as in original code
      fixed_imu_msg_.orientation_covariance[0] = 0.0025;
      fixed_imu_msg_.orientation_covariance[1] = 0.0;
      fixed_imu_msg_.orientation_covariance[2] = 0.0;
      fixed_imu_msg_.orientation_covariance[3] = 0.0;
      fixed_imu_msg_.orientation_covariance[4] = 0.0025;
      fixed_imu_msg_.orientation_covariance[5] = 0.0;
      fixed_imu_msg_.orientation_covariance[6] = 0.0;
      fixed_imu_msg_.orientation_covariance[7] = 0.0;
      fixed_imu_msg_.orientation_covariance[8] = 0.0025;
      fixed_imu_data_ = true;
    }
  }

  // Callback for yaw bias adjustment (subscribed to "set_pose")
  void imu_adjust_yaw_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double pitch, roll, yaw;
    tf2::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);

    tf2::Quaternion q2(fixed_imu_msg_.orientation.x, fixed_imu_msg_.orientation.y,
                         fixed_imu_msg_.orientation.z, fixed_imu_msg_.orientation.w);
    double pitch2, roll2, yaw2;
    tf2::Matrix3x3 mat2(q2);
    mat2.getEulerYPR(yaw2, pitch2, roll2);

    yaw_bias_ += yaw - yaw2;
  }

  // Timer callback executed at 100 Hz
  void timer_callback()
  {
    // Bias calculation phase
    if (bias_calculation_iterator_ < bias_calculation_iterations_) {
      gx_bias_ += read_raw_data(GYRO_XOUT_H) / 131.0 * 0.01745329;
      gy_bias_ += read_raw_data(GYRO_YOUT_H) / 131.0 * 0.01745329;
      gz_bias_ += read_raw_data(GYRO_ZOUT_H) / 131.0 * 0.01745329;
      ax_mean_ += read_raw_data(ACCEL_XOUT_H) / 16384.0;
      ay_mean_ += read_raw_data(ACCEL_YOUT_H) / 16384.0;
      az_mean_ += read_raw_data(ACCEL_ZOUT_H) / 16384.0;
      bias_calculation_iterator_++;
    } else if (bias_calculation_iterator_ == bias_calculation_iterations_) {
      // Finalize bias computation
      gx_bias_ /= bias_calculation_iterations_;
      gy_bias_ /= bias_calculation_iterations_;
      gz_bias_ /= bias_calculation_iterations_;
      ax_mean_ /= bias_calculation_iterations_;
      ay_mean_ /= bias_calculation_iterations_;
      az_mean_ /= bias_calculation_iterations_;
      norm_bias_ = std::sqrt(ax_mean_ * ax_mean_ + ay_mean_ * ay_mean_ + az_mean_ * az_mean_);
      acc_coeff_ = 1.0 / norm_bias_;
      bias_calculation_iterator_++;  // Increment to enter sensor reading phase
      RCLCPP_INFO(this->get_logger(), "Bias calculation complete.");
    } else {
      // Sensor reading and message publishing phase
      double acc_x = -(read_raw_data(ACCEL_YOUT_H) / 16384.0 * 9.80665 * acc_coeff_);
      double acc_y = read_raw_data(ACCEL_XOUT_H) / 16384.0 * 9.80665 * acc_coeff_;
      double acc_z = read_raw_data(ACCEL_ZOUT_H) / 16384.0 * 9.80665 * acc_coeff_;
      double gyro_x = -(read_raw_data(GYRO_YOUT_H) / 131.0 * 0.01745329 - gy_bias_);
      double gyro_y = read_raw_data(GYRO_XOUT_H) / 131.0 * 0.01745329 - gx_bias_;
      double gyro_z = read_raw_data(GYRO_ZOUT_H) / 131.0 * 0.01745329 - gz_bias_;

      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = this->now();
      imu_msg.header.frame_id = "imu_frame";
      imu_msg.angular_velocity.x = gyro_x;
      imu_msg.angular_velocity.y = gyro_y;
      imu_msg.angular_velocity.z = gyro_z;
      imu_msg.linear_acceleration.x = acc_x;
      imu_msg.linear_acceleration.y = acc_y;
      imu_msg.linear_acceleration.z = acc_z;
      // Set covariances as in original code
      imu_msg.angular_velocity_covariance[0] = 0.02;
      imu_msg.angular_velocity_covariance[1] = 0.0;
      imu_msg.angular_velocity_covariance[2] = 0.0;
      imu_msg.angular_velocity_covariance[3] = 0.0;
      imu_msg.angular_velocity_covariance[4] = 0.02;
      imu_msg.angular_velocity_covariance[5] = 0.0;
      imu_msg.angular_velocity_covariance[6] = 0.0;
      imu_msg.angular_velocity_covariance[7] = 0.0;
      imu_msg.angular_velocity_covariance[8] = 0.02;
      imu_msg.linear_acceleration_covariance[0] = 0.04;
      imu_msg.linear_acceleration_covariance[1] = 0.0;
      imu_msg.linear_acceleration_covariance[2] = 0.0;
      imu_msg.linear_acceleration_covariance[3] = 0.0;
      imu_msg.linear_acceleration_covariance[4] = 0.04;
      imu_msg.linear_acceleration_covariance[5] = 0.0;
      imu_msg.linear_acceleration_covariance[6] = 0.0;
      imu_msg.linear_acceleration_covariance[7] = 0.0;
      imu_msg.linear_acceleration_covariance[8] = 0.04;

      imu_pub_->publish(imu_msg);

      if (got_imu_data_) {
        if (fixed_imu_data_) {
          fixed_imu_msg_.header.stamp = this->now();
          fixed_imu_msg_.header.frame_id = "imu_frame";
          fixed_imu_pub_->publish(fixed_imu_msg_);
        }
        // Update and broadcast transform (using the latest transform_stamped_)
        transform_stamped_.header.stamp = this->now();
        tf_broadcaster_->sendTransform(transform_stamped_);
      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for filtered IMU data...");
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPU6050Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
