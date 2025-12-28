#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <std_srvs/srv/trigger.hpp>

class LeptrinoForceSensorNode : public rclcpp::Node
{
public:
  LeptrinoForceSensorNode()
  : Node("leptrino_force_sensor_node"), serial_fd_(-1)
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 460800);
    this->declare_parameter<std::string>("frame_id", "force_sensor_link");
    this->declare_parameter<std::string>("offset_file", "");

    this->get_parameter("port", port_);
    this->get_parameter("baudrate", baudrate_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("offset_file", offset_file_);

    pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("force_data", 10);

    offset_.fill(0.0f);
    calib_sum_.fill(0.0f);

    if (!offset_file_.empty()) {
      loadOffsetFromFile();
    }

    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_offset",
      std::bind(&LeptrinoForceSensorNode::handleSaveOffset, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    recalib_service_ = this->create_service<std_srvs::srv::Trigger>(
      "recalibrate",
      std::bind(&LeptrinoForceSensorNode::handleRecalibrate, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    if (!openSerialPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
      rclcpp::shutdown();
      return;
    }

    sendStartSignal();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&LeptrinoForceSensorNode::readSensorData, this)
    );
  }

  ~LeptrinoForceSensorNode() {
    if (serial_fd_ >= 0) close(serial_fd_);
  }

private:
  bool openSerialPort()
  {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) return false;

    termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    speed_t speed = B460800;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    return (tcsetattr(serial_fd_, TCSANOW, &tty) == 0);
  }

  void sendStartSignal()
  {
    uint8_t start_signal[] = {0x10, 0x02, 0x04, 0xFF, 0x32, 0x00, 0x10, 0x03, 0xCA};
    write(serial_fd_, start_signal, sizeof(start_signal));
    RCLCPP_INFO(this->get_logger(), "Start signal sent.");
  }

  void readSensorData()
  {
    uint8_t buf[256];
    int n = read(serial_fd_, buf, sizeof(buf));
    if (n > 0) {
      recv_buffer_.insert(recv_buffer_.end(), buf, buf + n);
    }

    std::array<float, 6> wrench_data;
    bool found = false;
    while (parseFrame(recv_buffer_, wrench_data)) {
      found = true;
    }
    if (!found) return;

    if (!calibrated_) {
      for (int i = 0; i < 6; ++i) {
        calib_sum_[i] += wrench_data[i];
      }
      calib_count_++;

      if (calib_count_ >= CALIB_SAMPLES) {
        for (int i = 0; i < 6; ++i) {
          offset_[i] = calib_sum_[i] / CALIB_SAMPLES;
        }
        calibrated_ = true;
        RCLCPP_INFO(this->get_logger(), "Force sensor calibration completed.");
      }
      return;
    }

    for (int i = 0; i < 6; ++i)
      wrench_data[i] -= offset_[i];

    auto msg = geometry_msgs::msg::WrenchStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.wrench.force.x  = wrench_data[0];
    msg.wrench.force.y  = wrench_data[1];
    msg.wrench.force.z  = wrench_data[2];
    msg.wrench.torque.x = wrench_data[3];
    msg.wrench.torque.y = wrench_data[4];
    msg.wrench.torque.z = wrench_data[5];
    pub_->publish(msg);
  }

  void startRecalibration()
  {
    calibrated_ = false;
    calib_count_ = 0;
    calib_sum_.fill(0.0f);
    RCLCPP_INFO(this->get_logger(), "Recalibration started.");
  }

  void handleSaveOffset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (offset_file_.empty()) {
      res->success = false;
      res->message = "offset_file is empty.";
      return;
    }
    saveOffsetToFile();
    res->success = true;
    res->message = "Offset saved.";
  }

  void handleRecalibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    startRecalibration();
    res->success = true;
    res->message = "Recalibration triggered.";
  }

  void saveOffsetToFile()
  {
    std::ofstream ofs(offset_file_);
    if (!ofs.is_open()) return;
    for (float v : offset_) ofs << v << "\n";
  }

  void loadOffsetFromFile()
  {
    std::ifstream ifs(offset_file_);
    if (!ifs.is_open()) return;
    for (float &v : offset_) ifs >> v;
    calibrated_ = true;
    RCLCPP_INFO(this->get_logger(), "Offset loaded from file.");
  }

  bool parseFrame(std::vector<uint8_t>& buffer, std::array<float, 6>& out)
  {
    const float MAX_FORCE = 500.0f;
    const float MAX_TORQUE = 4.0f;
    const int MAX_VAL = 10000;

    if (buffer.size() < 26) return false;

    size_t index = 0;
    while (index + 1 < buffer.size()) {
      if (buffer[index] == 0x10 && buffer[index + 1] == 0x02) break;
      ++index;
    }

    if (index + 26 > buffer.size()) return false;

    std::vector<uint8_t> data;
    size_t i = index + 2;
    while (data.size() < 16 && i < buffer.size()) {
      if (buffer[i] == 0x10) ++i;  // ダブル10回避
      if (i < buffer.size()) {
        data.push_back(buffer[i]);
        ++i;
      }
    }

    if (data.size() != 16) return false;

    for (int j = 0; j < 6; ++j) {
      int16_t raw = static_cast<int16_t>(data[j*2 + 4] | (data[j*2 + 5] << 8));
      float scaled = (j < 3) ?
        (static_cast<float>(raw) / MAX_VAL) * MAX_FORCE :
        (static_cast<float>(raw) / MAX_VAL) * MAX_TORQUE;
      out[j] = scaled;
    }

    buffer.erase(buffer.begin(), buffer.begin() + (i + 1));
    return true;
  }

  std::string port_;
  int baudrate_;
  std::string frame_id_;
  std::string offset_file_;
  int serial_fd_;
  std::vector<uint8_t> recv_buffer_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static constexpr int CALIB_SAMPLES = 200;
  bool calibrated_ = false;
  int calib_count_ = 0;
  std::array<float,6> calib_sum_, offset_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_, recalib_service_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeptrinoForceSensorNode>());
  rclcpp::shutdown();
  return 0;
}
