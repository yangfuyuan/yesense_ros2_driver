#include <io_context/io_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "analysis_data.h"

enum PARSE_STATES { FIND_HEAD_0, FIND_HEAD_1, FIND_LENGTH, FIND_END };

class ImuSerialNode : public rclcpp::Node {
 public:
  ImuSerialNode() : Node("imu_serial_node"), state_(FIND_HEAD_0), data_len_(0) {
    port_ = this->declare_parameter<std::string>("port", "/dev/imu");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 460800);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
    io_ctx_ = std::make_shared<drivers::common::IoContext>(1);
    drivers::serial_driver::SerialPortConfig config(
        baud_rate_, drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);
    serial_port_ = std::make_shared<drivers::serial_driver::SerialPort>(
        *io_ctx_, port_, config);
    try {
      serial_port_->open();
      serial_port_->async_receive(
          [this](std::vector<uint8_t>& data, const size_t& length) {
            this->parseReceivedData(data, length);
          });
      RCLCPP_INFO(this->get_logger(),
                  "IMU serial node initialized and started reading");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port[%s]: %s",
                   port_.c_str(), e.what());
    }
  }

  ~ImuSerialNode() {
    if (serial_port_->is_open()) {
      serial_port_->close();
    }
    io_ctx_->waitForExit();
  }

 private:
  void parseReceivedData(std::vector<uint8_t>& data, const size_t& length) {
    for (size_t i = 0; i < length; i++) {
      uint8_t c = data[i];
      switch (state_) {
        case FIND_HEAD_0:
          if (c == PROTOCOL_FIRST_BYTE) {
            input_buffer_.clear();
            input_buffer_.push_back(c);
            state_ = FIND_HEAD_1;
          }
          break;
        case FIND_HEAD_1:
          if (c == PROTOCOL_SECOND_BYTE) {
            input_buffer_.push_back(c);
            state_ = FIND_LENGTH;
          } else {
            state_ = FIND_HEAD_0;
          }
          break;
        case FIND_LENGTH:
          input_buffer_.push_back(c);
          if (input_buffer_.size() > PROTOCOL_MIN_LEN) {
            data_len_ = input_buffer_[4] + PROTOCOL_MIN_LEN;
            state_ = FIND_END;
          }
          break;
        case FIND_END:
          input_buffer_.push_back(c);
          if (input_buffer_.size() >= data_len_) {
            int ret = analysis_data(input_buffer_.data(), input_buffer_.size());
            if (ret != analysis_ok) {
              RCLCPP_WARN(this->get_logger(),
                          "analysis_data failed with error %d", ret);
            } else {
              publishImu();
            }
            state_ = FIND_HEAD_0;
          }
          break;
      }
    }
  }

  void publishImu() {
    RCLCPP_INFO_ONCE(this->get_logger(), "Publishing IMU data... frame_id: %s",
                     frame_id_.c_str());
    auto now = this->get_clock()->now();
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = frame_id_;

    tf2::Quaternion q;
    q.setRPY(g_output_info.roll / 180.0 * M_PI,
             g_output_info.pitch / 180.0 * M_PI,
             g_output_info.yaw / 180.0 * M_PI);
    q.normalize();

    imu_msg.orientation = tf2::toMsg(q);
    imu_msg.angular_velocity.x = g_output_info.angle_x / 180.0 * M_PI;
    imu_msg.angular_velocity.y = g_output_info.angle_y / 180.0 * M_PI;
    imu_msg.angular_velocity.z = g_output_info.angle_z / 180.0 * M_PI;
    imu_msg.linear_acceleration.x = g_output_info.accel_x;
    imu_msg.linear_acceleration.y = g_output_info.accel_y;
    imu_msg.linear_acceleration.z = g_output_info.accel_z;

    imu_pub_->publish(imu_msg);
  }

 private:
  std::string port_;
  int baud_rate_;
  std::string frame_id_;
  std::shared_ptr<drivers::common::IoContext> io_ctx_;
  std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  PARSE_STATES state_;
  size_t data_len_;
  std::vector<uint8_t> input_buffer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
