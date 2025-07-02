#include <io_context/io_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial_driver/serial_driver.hpp>
#include <vector>
class ImuSerialNode : public rclcpp::Node {
 public:
  enum ParseState { FindHead0, FindHead1, FindLength, FindEnd };

  ImuSerialNode();
  ~ImuSerialNode();

 private:
  void parseReceivedData(std::vector<uint8_t>& data, const size_t& length);
  void publishImu();

 private:
  std::string port_;
  int baud_rate_;
  std::string frame_id_;
  std::shared_ptr<drivers::common::IoContext> io_ctx_;
  std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  ParseState state_;
  size_t data_len_;
  std::vector<uint8_t> input_buffer_;
};
