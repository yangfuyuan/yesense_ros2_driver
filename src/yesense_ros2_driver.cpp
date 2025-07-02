#include <yesense_ros2_driver/analysis_data.h>

#include <yesense_ros2_driver/quaternion.hpp>
#include <yesense_ros2_driver/yesense_ros2_driver.hpp>

ImuSerialNode::ImuSerialNode()
    : Node("imu_serial_node"), state_(FindHead0), data_len_(0) {
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
  startSerialReceive();
  reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&ImuSerialNode::tryReconnect, this));
}

ImuSerialNode::~ImuSerialNode() {
  if (serial_port_->is_open()) {
    serial_port_->close();
  }
  io_ctx_->waitForExit();
}

bool ImuSerialNode::startSerialReceive() {
  try {
    serial_port_->open();
    serial_port_->async_receive(
        [this](std::vector<uint8_t>& data, const size_t& length) {
          this->parseReceivedData(data, length);
        });
    RCLCPP_INFO(this->get_logger(),
                "IMU serial node initialized and started reading");
    has_logged_startup_ = true;
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port[%s]: %s",
                 port_.c_str(), e.what());
  }
  return false;
}

void ImuSerialNode::tryReconnect() {
  if (serial_port_ && serial_port_->is_open()) {
    if (access(port_.c_str(), F_OK) == 0) {
      return;
    }
    serial_port_->close();
  }
  if (access(port_.c_str(), F_OK) == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Detected device [%s], trying to reconnect...", port_.c_str());
    startSerialReceive();
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for device [%s] to be available...",
                         port_.c_str());
  }
}

void ImuSerialNode::parseReceivedData(std::vector<uint8_t>& data,
                                      const size_t& length) {
  for (size_t i = 0; i < length; i++) {
    uint8_t c = data[i];
    switch (state_) {
      case FindHead0:
        if (c == PROTOCOL_FIRST_BYTE) {
          input_buffer_.clear();
          input_buffer_.push_back(c);
          state_ = FindHead1;
        }
        break;
      case FindHead1:
        if (c == PROTOCOL_SECOND_BYTE) {
          input_buffer_.push_back(c);
          state_ = FindLength;
        } else {
          state_ = (c == PROTOCOL_FIRST_BYTE) ? FindHead1 : FindHead0;
          input_buffer_.clear();
          if (c == PROTOCOL_FIRST_BYTE) {
            input_buffer_.push_back(c);
          }
        }
        break;
      case FindLength:
        input_buffer_.push_back(c);
        if (input_buffer_.size() > PROTOCOL_MIN_LEN) {
          data_len_ = input_buffer_[4] + PROTOCOL_MIN_LEN;
          state_ = FindEnd;
        }
        break;
      case FindEnd:
        input_buffer_.push_back(c);
        if (input_buffer_.size() >= data_len_) {
          int ret = analysis_data(input_buffer_.data(), input_buffer_.size());
          if (ret != analysis_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "analysis_data failed with error %d", ret);
          } else {
            publishImu();
          }
          state_ = FindHead0;
          input_buffer_.clear();
        }
        break;
    }
  }
}

void ImuSerialNode::publishImu() {
  if (has_logged_startup_) {
    has_logged_startup_ = false;
    RCLCPP_INFO(this->get_logger(), "Publishing IMU data... frame_id: %s",
                frame_id_.c_str());
  }
  auto now = this->get_clock()->now();
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = now;
  imu_msg.header.frame_id = frame_id_;
  Quaterniond q;
  q.setRPY(from_degrees(g_output_info.roll), from_degrees(g_output_info.pitch),
           from_degrees(g_output_info.yaw));
  q.normalize();

  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();
  imu_msg.orientation.w = q.w();
  imu_msg.angular_velocity.x = from_degrees(g_output_info.angle_x);
  imu_msg.angular_velocity.y = from_degrees(g_output_info.angle_y);
  imu_msg.angular_velocity.z = from_degrees(g_output_info.angle_z);
  imu_msg.linear_acceleration.x = g_output_info.accel_x;
  imu_msg.linear_acceleration.y = g_output_info.accel_y;
  imu_msg.linear_acceleration.z = g_output_info.accel_z;

  imu_pub_->publish(std::move(imu_msg));
}
