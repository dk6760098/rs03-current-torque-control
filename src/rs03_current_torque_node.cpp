#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kTypeMotion = 0x01;
constexpr uint8_t kTypeFeedback = 0x02;
constexpr uint8_t kTypeEnable = 0x03;
constexpr uint8_t kTypeStop = 0x04;
constexpr uint8_t kTypeWriteParam = 0x12;
constexpr uint16_t kRunMode = 0x7005;
constexpr uint16_t kIqRef = 0x7006;
constexpr float kProtocolCurrentMaxA = 43.0F;
constexpr float kProtocolTorqueMaxNm = 60.0F;
constexpr float kPositionMaxRad = 4.0F * static_cast<float>(M_PI);
constexpr float kVelocityMaxRadS = 50.0F;
constexpr float kKpMax = 5000.0F;
constexpr float kKdMax = 100.0F;

uint16_t encode_u16(float value, float low, float high) {
  value = std::clamp(value, low, high);
  return static_cast<uint16_t>(std::lround((value - low) * 65535.0F / (high - low)));
}

float decode_u16(uint16_t value, float low, float high) {
  return static_cast<float>(value) * (high - low) / 65535.0F + low;
}
}  // namespace

class Rs03Can {
 public:
  Rs03Can(const std::string &iface, uint8_t master_id, uint8_t motor_id,
          int receive_timeout_ms)
      : master_id_(master_id), motor_id_(motor_id) {
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("cannot create CAN socket");

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      close(fd_); fd_ = -1;
      throw std::runtime_error("CAN interface not found: " + iface);
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      close(fd_); fd_ = -1;
      throw std::runtime_error("cannot bind CAN interface: " + iface);
    }
    timeval timeout{};
    timeout.tv_sec = receive_timeout_ms / 1000;
    timeout.tv_usec = (receive_timeout_ms % 1000) * 1000;
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

  ~Rs03Can() { if (fd_ >= 0) close(fd_); }

  void set_mode(uint8_t mode) {
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(kRunMode & 0xff);
    data[1] = static_cast<uint8_t>(kRunMode >> 8);
    data[4] = mode;
    send(kTypeWriteParam, master_id_, data);
  }

  void set_iq(float current_a) {
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(kIqRef & 0xff);
    data[1] = static_cast<uint8_t>(kIqRef >> 8);
    static_assert(sizeof(float) == 4, "RS03 protocol requires 32-bit float");
    std::memcpy(data.data() + 4, &current_a, sizeof(current_a));
    send(kTypeWriteParam, master_id_, data);
  }

  void set_torque(float torque_nm) {
    can_frame frame{};
    const uint16_t torque_raw = encode_u16(torque_nm, -kProtocolTorqueMaxNm,
                                            kProtocolTorqueMaxNm);
    frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(kTypeMotion) << 24) |
                   (static_cast<uint32_t>(torque_raw) << 8) | motor_id_;
    frame.can_dlc = 8;
    // Pure feed-forward torque: desired position/velocity and Kp/Kd are zero.
    const uint16_t pos = encode_u16(0.0F, -kPositionMaxRad, kPositionMaxRad);
    const uint16_t vel = encode_u16(0.0F, -kVelocityMaxRadS, kVelocityMaxRadS);
    const uint16_t kp = encode_u16(0.0F, 0.0F, kKpMax);
    const uint16_t kd = encode_u16(0.0F, 0.0F, kKdMax);
    frame.data[0] = pos >> 8; frame.data[1] = pos & 0xff;
    frame.data[2] = vel >> 8; frame.data[3] = vel & 0xff;
    frame.data[4] = kp >> 8;  frame.data[5] = kp & 0xff;
    frame.data[6] = kd >> 8;  frame.data[7] = kd & 0xff;
    write_frame(frame);
  }

  void enable() { send(kTypeEnable, master_id_, {}); }
  void stop() { send(kTypeStop, master_id_, {}); }

  struct Feedback { float position_rad, velocity_rad_s, torque_nm, temperature_c; };
  bool receive_feedback(Feedback &out) {
    can_frame frame{};
    const ssize_t count = recv(fd_, &frame, sizeof(frame), 0);
    if (count != sizeof(frame) || !(frame.can_id & CAN_EFF_FLAG)) return false;
    const uint32_t id = frame.can_id & CAN_EFF_MASK;
    if (((id >> 24) & 0x1f) != kTypeFeedback || frame.can_dlc < 8) return false;
    const uint16_t p = (frame.data[0] << 8) | frame.data[1];
    const uint16_t v = (frame.data[2] << 8) | frame.data[3];
    const uint16_t t = (frame.data[4] << 8) | frame.data[5];
    const uint16_t temp = (frame.data[6] << 8) | frame.data[7];
    out = {decode_u16(p, -kPositionMaxRad, kPositionMaxRad),
           decode_u16(v, -kVelocityMaxRadS, kVelocityMaxRadS),
           decode_u16(t, -kProtocolTorqueMaxNm, kProtocolTorqueMaxNm),
           static_cast<float>(temp) * 0.1F};
    return true;
  }

 private:
  void send(uint8_t type, uint16_t extra, const std::array<uint8_t, 8> &data) {
    can_frame frame{};
    frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(type) << 24) |
                   (static_cast<uint32_t>(extra) << 8) | motor_id_;
    frame.can_dlc = 8;
    std::copy(data.begin(), data.end(), frame.data);
    write_frame(frame);
  }

  void write_frame(const can_frame &frame) {
    if (write(fd_, &frame, sizeof(frame)) != sizeof(frame))
      throw std::runtime_error("CAN frame write failed");
  }

  int fd_{-1};
  uint8_t master_id_;
  uint8_t motor_id_;
};

class Rs03Node final : public rclcpp::Node {
 public:
  Rs03Node() : Node("rs03_current_torque") {
    const auto iface = declare_parameter("can_interface", "can0");
    const auto motor_id = declare_parameter("motor_id", 1);
    const auto master_id = declare_parameter("master_id", 255);
    mode_ = declare_parameter("control_mode", "current");
    const bool auto_enable = declare_parameter("auto_enable", false);
    timeout_s_ = declare_parameter("command_timeout_s", 0.10);
    max_current_a_ = std::clamp(declare_parameter("max_current_a", 1.0),
                                0.0, static_cast<double>(kProtocolCurrentMaxA));
    max_torque_nm_ = std::clamp(declare_parameter("max_torque_nm", 2.0),
                                0.0, static_cast<double>(kProtocolTorqueMaxNm));
    const auto receive_timeout = declare_parameter("receive_timeout_ms", 20);
    if (motor_id < 0 || motor_id > 255 || master_id < 0 || master_id > 255)
      throw std::invalid_argument("motor_id and master_id must be in [0, 255]");
    if (timeout_s_ <= 0.0 || receive_timeout < 0)
      throw std::invalid_argument("timeouts must be positive");
    if (mode_ != "current" && mode_ != "torque")
      throw std::invalid_argument("control_mode must be current or torque");
    can_ = std::make_unique<Rs03Can>(iface, static_cast<uint8_t>(master_id),
                                     static_cast<uint8_t>(motor_id), receive_timeout);

    command_sub_ = create_subscription<std_msgs::msg::Float32>(
        mode_ == "current" ? "~/current_command_a" : "~/torque_command_nm", 10,
        [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
          command_ = msg->data;
          last_command_ = now();
          command_seen_ = true;
        });
    torque_pub_ = create_publisher<std_msgs::msg::Float32>("~/estimated_torque_nm", 10);
    timer_ = create_wall_timer(10ms, [this] { update(); });

    if (auto_enable) {
      can_->stop();
      can_->set_mode(mode_ == "current" ? 3 : 0);
      can_->enable();
      enabled_ = true;
      last_command_ = now();
    } else {
      RCLCPP_WARN(get_logger(), "auto_enable=false: motor remains stopped; set true only after bench checks");
      can_->stop();
    }
  }

  ~Rs03Node() override {
    try { zero_and_stop(); } catch (...) {}
  }

 private:
  void update() {
    if (!enabled_) return;
    const bool fresh = command_seen_ && (now() - last_command_).seconds() <= timeout_s_;
    const float requested = fresh ? command_ : 0.0F;
    if (mode_ == "current")
      can_->set_iq(std::clamp(requested, -static_cast<float>(max_current_a_),
                              static_cast<float>(max_current_a_)));
    else
      can_->set_torque(std::clamp(requested, -static_cast<float>(max_torque_nm_),
                                  static_cast<float>(max_torque_nm_)));
    if (!fresh && command_seen_ && !timeout_reported_) {
      RCLCPP_ERROR(get_logger(), "command timeout: output forced to zero");
      timeout_reported_ = true;
    } else if (fresh) timeout_reported_ = false;

    Rs03Can::Feedback fb{};
    if (can_->receive_feedback(fb)) {
      std_msgs::msg::Float32 msg;
      msg.data = fb.torque_nm;
      torque_pub_->publish(msg);
    }
  }

  void zero_and_stop() {
    if (!can_) return;
    if (mode_ == "current") can_->set_iq(0.0F); else can_->set_torque(0.0F);
    can_->stop();
    enabled_ = false;
  }

  std::unique_ptr<Rs03Can> can_;
  std::string mode_;
  double timeout_s_{0.1}, max_current_a_{1.0}, max_torque_nm_{2.0};
  float command_{0.0F};
  bool enabled_{false}, command_seen_{false}, timeout_reported_{false};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Rs03Node>()); }
  catch (const std::exception &e) {
    std::fprintf(stderr, "RS03 controller fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
