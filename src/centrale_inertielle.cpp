#include <memory>
#include <chrono>
#include <cmath>
#include <iostream>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// WiringPi

#include <wiringPi.h>
#include <wiringPiI2C.h>
// Constants matching your original code
#define Device_Address 0x68
const int MPU6050_GndPin = 4;

// MPU6050 registers
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

class CentraleInertielleNode : public rclcpp::Node
{
public:
  CentraleInertielleNode()
  : Node("centrale_inertielle_node"),
    got_imu_data_(false),
    fixed_imu_data_(false),
    init_yaw_(0.0),
    yaw_bias_(0.0)
  {
    // 1) Initialize Publishers & Subscribers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
    fixed_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_reoriented", 1);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 1,
        std::bind(&CentraleInertielleNode::imuOrientationFixerCallback, this, std::placeholders::_1));

    imu_yaw_bias_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "set_pose", 1,
        std::bind(&CentraleInertielleNode::imuAdjustYawCallback, this, std::placeholders::_1));

    // 2) Set up wiringPi, I2C, and MPU6050
    fd_ = wiringPiI2CSetup(Device_Address);
    wiringPiSetupGpio();
    pinMode(MPU6050_GndPin, OUTPUT);
    digitalWrite(MPU6050_GndPin, LOW);
    delay(200);
    MPU6050Init();

    // 3) Initialize IMU messages and covariance
    initImuMsg(imu_msg_);
    initImuMsg(fixed_imu_msg_);

    // 4) Set up a timer at 100 Hz to replicate the while-loop behavior
    loop_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CentraleInertielleNode::loopTimerCallback, this));

    // 5) Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 6) Bias calculation variables
    bias_calc_iterations_ = 500;  // Number of samples for bias
    bias_calc_iterator_   = 0;
    norm_bias_           = 0.0;
    ax_mean_ = ay_mean_ = az_mean_ = 0.0;
    gx_bias_ = gy_bias_ = gz_bias_ = 0.0;
    acc_coeff_           = 0.0;

    RCLCPP_INFO(this->get_logger(), "CentraleInertielleNode started.");
  }

private:
  // ==========================================================
  //  MPU6050 Configuration
  // ==========================================================
  void MPU6050Init()
  {
    wiringPiI2CWriteReg8(fd_, SMPLRT_DIV, 0x07);
    wiringPiI2CWriteReg8(fd_, PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(fd_, CONFIG, 0x00);
    wiringPiI2CWriteReg8(fd_, ACCEL_CONFIG, 0x00);
    wiringPiI2CWriteReg8(fd_, GYRO_CONFIG, 0x00);
    wiringPiI2CWriteReg8(fd_, INT_ENABLE, 0x01);
  }

  short readRawData(int addr)
  {
    short high_byte = wiringPiI2CReadReg8(fd_, addr);
    short low_byte  = wiringPiI2CReadReg8(fd_, addr + 1);
    short value     = (high_byte << 8) | low_byte;
    return value;
  }

  // ==========================================================
  //  IMU message initialization
  // ==========================================================
  void initImuMsg(sensor_msgs::msg::Imu &msg)
  {
    // Angular velocity covariance
    msg.angular_velocity_covariance[0] = 0.02;
    msg.angular_velocity_covariance[1] = 0.0;
    msg.angular_velocity_covariance[2] = 0.0;
    msg.angular_velocity_covariance[3] = 0.0;
    msg.angular_velocity_covariance[4] = 0.02;
    msg.angular_velocity_covariance[5] = 0.0;
    msg.angular_velocity_covariance[6] = 0.0;
    msg.angular_velocity_covariance[7] = 0.0;
    msg.angular_velocity_covariance[8] = 0.02;

    // Linear acceleration covariance
    msg.linear_acceleration_covariance[0] = 0.04;
    msg.linear_acceleration_covariance[1] = 0.0;
    msg.linear_acceleration_covariance[2] = 0.0;
    msg.linear_acceleration_covariance[3] = 0.0;
    msg.linear_acceleration_covariance[4] = 0.04;
    msg.linear_acceleration_covariance[5] = 0.0;
    msg.linear_acceleration_covariance[6] = 0.0;
    msg.linear_acceleration_covariance[7] = 0.0;
    msg.linear_acceleration_covariance[8] = 0.04;

    // Orientation covariance will be set/updated for fixed_imu_msg_
  }

  // ==========================================================
  //  Subscribers
  // ==========================================================
  void imuOrientationFixerCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!got_imu_data_)
    {
      RCLCPP_INFO(this->get_logger(), "Received first IMU data. Reorienting...");

      // Extract initial yaw from msg->orientation
      tf2::Quaternion q(msg->orientation.x,
                        msg->orientation.y,
                        msg->orientation.z,
                        msg->orientation.w);
      tf2::Matrix3x3 mat(q);

      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);  // RCLCPP: returns roll, pitch, yaw
      init_yaw_ = yaw;               // We store the initial yaw

      // You might do additional transforms if needed
      got_imu_data_ = true;
    }
    else
    {
      // Build fixed_imu_msg_ from incoming IMU
      fixed_imu_msg_ = *msg;

      tf2::Quaternion fixed_q(msg->orientation.x,
                              msg->orientation.y,
                              msg->orientation.z,
                              msg->orientation.w);

      tf2::Matrix3x3 fixed_mat(fixed_q);
      double roll, pitch, yaw;
      fixed_mat.getRPY(roll, pitch, yaw);

      // Apply the yaw reorientation:
      yaw = yaw - init_yaw_ + yaw_bias_;

      // Rebuild quaternion
      tf2::Quaternion new_q;
      new_q.setRPY(roll, pitch, yaw);

      // Store back into fixed_imu_msg_
      fixed_imu_msg_.orientation.x = new_q.x();
      fixed_imu_msg_.orientation.y = new_q.y();
      fixed_imu_msg_.orientation.z = new_q.z();
      fixed_imu_msg_.orientation.w = new_q.w();

      // Orientation covariance
      fixed_imu_msg_.orientation_covariance[0] = 0.0025;
      fixed_imu_msg_.orientation_covariance[4] = 0.0025;
      fixed_imu_msg_.orientation_covariance[8] = 0.0025;

      fixed_imu_data_ = true;
    }
  }

  void imuAdjustYawCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // Extract yaw from PoseWithCovarianceStamped
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // Extract yaw from the current fixed_imu_msg_
    tf2::Quaternion q2(fixed_imu_msg_.orientation.x,
                       fixed_imu_msg_.orientation.y,
                       fixed_imu_msg_.orientation.z,
                       fixed_imu_msg_.orientation.w);
    tf2::Matrix3x3 mat2(q2);
    double roll2, pitch2, yaw2;
    mat2.getRPY(roll2, pitch2, yaw2);

    // Adjust yaw_bias_ so that fixed_imu_msg_ aligns with the new pose
    yaw_bias_ += (yaw - yaw2);
  }

  // ==========================================================
  //  Main loop logic (Timer Callback @ 100Hz)
  // ==========================================================
  void loopTimerCallback()
  {
    // 1) Perform bias calculation if needed
    if (bias_calc_iterator_ > bias_calc_iterations_)
    {
      // Already computed bias => read IMU, publish
      double acc_x = -( readRawData(ACCEL_YOUT_H) / 16384.0 * 9.80665 * acc_coeff_ );
      double acc_y =  ( readRawData(ACCEL_XOUT_H) / 16384.0 * 9.80665 * acc_coeff_ );
      double acc_z =  ( readRawData(ACCEL_ZOUT_H) / 16384.0 * 9.80665 * acc_coeff_ );

      double gyro_x = ( readRawData(GYRO_XOUT_H)/131.0 * 0.01745329 - gx_bias_ );
      double gyro_y = -(readRawData(GYRO_YOUT_H)/131.0 * 0.01745329 - gy_bias_ );
      double gyro_z = ( readRawData(GYRO_ZOUT_H)/131.0 * 0.01745329 - gz_bias_ );

      // Fill imu_msg_
      imu_msg_.header.stamp = this->now();
      imu_msg_.header.frame_id = "imu_frame";
      imu_msg_.angular_velocity.x    = gyro_x;
      imu_msg_.angular_velocity.y    = gyro_y;
      imu_msg_.angular_velocity.z    = gyro_z;
      imu_msg_.linear_acceleration.x = acc_x;
      imu_msg_.linear_acceleration.y = acc_y;
      imu_msg_.linear_acceleration.z = acc_z;

      // Publish raw IMU
      imu_pub_->publish(imu_msg_);

      // 2) If we have re-oriented IMU data, publish that as well
      // if (got_imu_data_ && fixed_imu_data_)
      // {
      //   fixed_imu_msg_.header.stamp = this->now();
      //   fixed_imu_msg_.header.frame_id = "imu_frame";
      //   fixed_imu_pub_->publish(fixed_imu_msg_);

      //   // Broadcast a transform from "r_robot" to "imu_frame" (if desired)
      //   geometry_msgs::msg::TransformStamped t;
      //   t.header.stamp = this->now();
      //   t.header.frame_id = "r_robot";
      //   t.child_frame_id  = "imu_frame";
      //   // Just an identity transform in translation
      //   t.transform.translation.x = 0.0;
      //   t.transform.translation.y = 0.0;
      //   t.transform.translation.z = 0.0;
      //   // No rotation in this example, but you could embed your rolling/pitching
      //   tf2::Quaternion rot;
      //   rot.setRPY(0, 0, 0);
      //   t.transform.rotation.x = rot.x();
      //   t.transform.rotation.y = rot.y();
      //   t.transform.rotation.z = rot.z();
      //   t.transform.rotation.w = rot.w();

      //   // tf_broadcaster_->sendTransform(t);
      // }
      else
      {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,  // 5s
          "Waiting for re-oriented IMU data from the filter..."
        );
      }
    }
    else if (bias_calc_iterator_ == bias_calc_iterations_)
    {
      // Finalize bias calculation
      gx_bias_ /= bias_calc_iterations_;
      gy_bias_ /= bias_calc_iterations_;
      gz_bias_ /= bias_calc_iterations_;

      ax_mean_ /= bias_calc_iterations_;
      ay_mean_ /= bias_calc_iterations_;
      az_mean_ /= bias_calc_iterations_;

      norm_bias_ = std::sqrt(std::pow(ax_mean_, 2.0) +
                             std::pow(ay_mean_, 2.0) +
                             std::pow(az_mean_, 2.0));

      // We assume 1-g => multiply by inverse to normalize
      acc_coeff_ = 1.0 / norm_bias_;

      RCLCPP_INFO(this->get_logger(),
                  "Bias computed: gx=%.4f, gy=%.4f, gz=%.4f, acc_coeff=%.4f",
                  gx_bias_, gy_bias_, gz_bias_, acc_coeff_);

      bias_calc_iterator_++;
    }
    else
    {
      // Collect data for bias calculation
      gx_bias_ +=  ( readRawData(GYRO_XOUT_H)/131.0 * 0.01745329 );
      gy_bias_ += -( readRawData(GYRO_YOUT_H)/131.0 * 0.01745329 );
      gz_bias_ +=  ( readRawData(GYRO_ZOUT_H)/131.0 * 0.01745329 );

      ax_mean_ +=  ( readRawData(ACCEL_XOUT_H)/16384.0 );
      ay_mean_ +=  ( readRawData(ACCEL_YOUT_H)/16384.0 );
      az_mean_ +=  ( readRawData(ACCEL_ZOUT_H)/16384.0 );

      bias_calc_iterator_++;
    }
  }

  // ==========================================================
  //  Member Variables
  // ==========================================================
  // Publishers / Subscribers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fixed_imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr imu_yaw_bias_sub_;

  // Timer for 100 Hz loop
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // IMU data
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::Imu fixed_imu_msg_;

  bool  got_imu_data_;
  bool  fixed_imu_data_;
  double init_yaw_;
  double yaw_bias_;

  // MPU6050
  int fd_;
//   void MPU6050Init();
//   short readRawData(int addr);

  // Bias calculation
  int    bias_calc_iterations_;
  int    bias_calc_iterator_;
  double norm_bias_;
  double ax_mean_, ay_mean_, az_mean_, acc_coeff_;
  double gx_bias_, gy_bias_, gz_bias_;
};

/* ===============================================
 *                 MAIN
 * ===============================================*/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CentraleInertielleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
