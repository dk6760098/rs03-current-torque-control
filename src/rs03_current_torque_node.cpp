#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kTypeMotion = 0x01;
constexpr uint8_t kTypeFeedback = 0x02;
constexpr uint8_t kTypeEnable = 0x03;
constexpr uint8_t kTypeStop = 0x04;
constexpr uint8_t kTypeReadParam = 0x11;
constexpr uint8_t kTypeWriteParam = 0x12;
constexpr uint16_t kRunMode = 0x7005;
constexpr uint16_t kIqRef = 0x7006;
constexpr uint16_t kSpeedRef = 0x700A;
constexpr uint16_t kPositionRef = 0x7016;
constexpr uint16_t kPositionSpeedLimit = 0x7024;
constexpr uint16_t kCurrentLimit = 0x7018;
constexpr uint16_t kMechanicalPosition = 0x7019;
constexpr uint16_t kVelocityAcceleration = 0x7022;
constexpr uint16_t kPositionAcceleration = 0x7025;
constexpr float kProtocolCurrentMaxA = 43.0F;
constexpr float kProtocolTorqueMaxNm = 60.0F;
constexpr float kPositionMaxRad = 4.0F * static_cast<float>(M_PI);
// RS03 protocol Type-1 command / Type-2 feedback velocity mapping is
// -20.0 .. +20.0 rad/s. Using the ±50 range from other motor variants makes
// decoded feedback 2.5x too large and can cause false overspeed trips.
constexpr float kVelocityMaxRadS = 20.0F;
constexpr float kKpMax = 5000.0F;
constexpr float kKdMax = 100.0F;
constexpr uint8_t kSerialExtendedFrameFlag = 0x04;

uint16_t encode_u16(float value, float low, float high) {
  value = std::clamp(value, low, high);
  return static_cast<uint16_t>(std::lround((value - low) * 65535.0F / (high - low)));
}

float decode_u16(uint16_t value, float low, float high) {
  return static_cast<float>(value) * (high - low) / 65535.0F + low;
}

float cyclic_position_error(float absolute_target, float cyclic_feedback) {
  // Type-2 position feedback cycles over -4pi..+4pi, whereas mechPos/loc_ref
  // are absolute multi-turn positions. Compare them modulo the 8pi period.
  return std::remainder(absolute_target - cyclic_feedback,
                        2.0F * kPositionMaxRad);
}
}  // namespace

class Rs03Can {
 public:
  Rs03Can(const std::string &transport, const std::string &iface,
          const std::string &serial_device, int serial_baud,
          bool serial_debug, uint8_t master_id, uint8_t motor_id,
          int receive_timeout_ms)
      : serial_mode_(transport == "serial"),
        serial_debug_(serial_debug), receive_timeout_ms_(receive_timeout_ms),
        master_id_(master_id), motor_id_(motor_id) {
    if (serial_mode_) {
      if (serial_baud != 921600)
        throw std::invalid_argument("the official CH340 transport currently requires serial_baud=921600");
      fd_ = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (fd_ < 0)
        throw std::runtime_error("cannot open serial device " + serial_device +
                                 ": " + std::strerror(errno));
      termios tty{};
      if (tcgetattr(fd_, &tty) != 0) {
        const std::string error = std::strerror(errno);
        close(fd_); fd_ = -1;
        throw std::runtime_error("cannot read serial settings: " + error);
      }
      cfmakeraw(&tty);
      cfsetispeed(&tty, B921600);
      cfsetospeed(&tty, B921600);
      tty.c_cflag |= CLOCAL | CREAD;
      tty.c_cflag &= ~CRTSCTS;
      tty.c_cc[VMIN] = 0;
      tty.c_cc[VTIME] = 0;
      if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        const std::string error = std::strerror(errno);
        close(fd_); fd_ = -1;
        throw std::runtime_error("cannot configure serial device: " + error);
      }
      tcflush(fd_, TCIOFLUSH);
      return;
    }

    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0)
      throw std::runtime_error(std::string("cannot create CAN socket: ") +
                               std::strerror(errno));

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      close(fd_); fd_ = -1;
      throw std::runtime_error("CAN interface not found: " + iface + ": " +
                               std::strerror(errno));
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      close(fd_); fd_ = -1;
      throw std::runtime_error("cannot bind CAN interface: " + iface + ": " +
                               std::strerror(errno));
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
    set_float_parameter(kIqRef, current_a);
  }

  void set_velocity(float velocity_rad_s) {
    set_float_parameter(kSpeedRef, velocity_rad_s);
  }

  void set_position(float position_rad) {
    set_float_parameter(kPositionRef, position_rad);
  }

  void configure_velocity(float current_limit_a, float acceleration_rad_s2) {
    set_float_parameter(kCurrentLimit, current_limit_a);
    set_float_parameter(kVelocityAcceleration, acceleration_rad_s2);
  }

  void configure_position_pp(float current_limit_a, float speed_limit_rad_s,
                             float acceleration_rad_s2) {
    set_float_parameter(kCurrentLimit, current_limit_a);
    set_float_parameter(kPositionSpeedLimit, speed_limit_rad_s);
    set_float_parameter(kPositionAcceleration, acceleration_rad_s2);
  }

  void set_float_parameter(uint16_t index, float value) {
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(index & 0xff);
    data[1] = static_cast<uint8_t>(index >> 8);
    static_assert(sizeof(float) == 4, "RS03 protocol requires 32-bit float");
    std::memcpy(data.data() + 4, &value, sizeof(value));
    send(kTypeWriteParam, master_id_, data);
  }

  bool read_float_parameter(uint16_t index, float &value) {
    std::array<uint8_t, 8> request{};
    request[0] = static_cast<uint8_t>(index & 0xff);
    request[1] = static_cast<uint8_t>(index >> 8);
    send(kTypeReadParam, master_id_, request);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(receive_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      can_frame frame{};
      if (!read_frame(frame, deadline)) return false;
      if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 8) continue;
      const uint32_t id = frame.can_id & CAN_EFF_MASK;
      if (((id >> 24) & 0x1f) != kTypeReadParam ||
          (id & 0xff) != master_id_ || ((id >> 8) & 0xff) != motor_id_)
        continue;
      const uint16_t response_index =
          static_cast<uint16_t>(frame.data[0]) |
          (static_cast<uint16_t>(frame.data[1]) << 8);
      if (response_index != index) continue;
      std::memcpy(&value, frame.data + 4, sizeof(value));
      return std::isfinite(value);
    }
    return false;
  }

  bool read_u8_parameter(uint16_t index, uint8_t &value) {
    std::array<uint8_t, 8> request{};
    request[0] = static_cast<uint8_t>(index & 0xff);
    request[1] = static_cast<uint8_t>(index >> 8);
    send(kTypeReadParam, master_id_, request);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(receive_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      can_frame frame{};
      if (!read_frame(frame, deadline)) return false;
      if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 8) continue;
      const uint32_t id = frame.can_id & CAN_EFF_MASK;
      if (((id >> 24) & 0x1f) != kTypeReadParam ||
          (id & 0xff) != master_id_ || ((id >> 8) & 0xff) != motor_id_)
        continue;
      const uint16_t response_index =
          static_cast<uint16_t>(frame.data[0]) |
          (static_cast<uint16_t>(frame.data[1]) << 8);
      if (response_index != index) continue;
      value = frame.data[4];
      return true;
    }
    return false;
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
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(receive_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      can_frame frame{};
      if (!read_frame(frame, deadline)) return false;
      if (!(frame.can_id & CAN_EFF_FLAG)) continue;
      const uint32_t id = frame.can_id & CAN_EFF_MASK;
      if (((id >> 24) & 0x1f) != kTypeFeedback || frame.can_dlc < 8 ||
          (id & 0xff) != master_id_ || ((id >> 8) & 0xff) != motor_id_)
        continue;
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
    return false;
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
    if (!serial_mode_) {
      if (write(fd_, &frame, sizeof(frame)) != sizeof(frame))
        throw std::runtime_error(std::string("CAN frame write failed: ") +
                                 std::strerror(errno));
      return;
    }
    const uint32_t can_id = frame.can_id & CAN_EFF_MASK;
    const uint32_t serial_id = (can_id << 3) | kSerialExtendedFrameFlag;
    std::array<uint8_t, 17> packet{};
    packet[0] = 'A'; packet[1] = 'T';
    packet[2] = static_cast<uint8_t>(serial_id >> 24);
    packet[3] = static_cast<uint8_t>(serial_id >> 16);
    packet[4] = static_cast<uint8_t>(serial_id >> 8);
    packet[5] = static_cast<uint8_t>(serial_id);
    packet[6] = frame.can_dlc;
    std::copy(frame.data, frame.data + frame.can_dlc, packet.begin() + 7);
    packet[7 + frame.can_dlc] = '\r';
    packet[8 + frame.can_dlc] = '\n';
    const size_t length = 9 + frame.can_dlc;
    if (serial_debug_) {
      std::fprintf(stderr, "RS03 serial TX (%zu):", length);
      for (size_t i = 0; i < length; ++i) std::fprintf(stderr, " %02X", packet[i]);
      std::fprintf(stderr, "\n");
    }
    size_t offset = 0;
    while (offset < length) {
      const ssize_t count = write(fd_, packet.data() + offset, length - offset);
      if (count > 0) { offset += static_cast<size_t>(count); continue; }
      if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
      throw std::runtime_error(std::string("CAN frame write failed: ") +
                               std::strerror(errno));
    }
  }

  using Deadline = std::chrono::steady_clock::time_point;

  bool read_exact(uint8_t *data, size_t length, const Deadline &deadline) {
    size_t offset = 0;
    while (offset < length && std::chrono::steady_clock::now() < deadline) {
      fd_set set;
      FD_ZERO(&set); FD_SET(fd_, &set);
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
          deadline - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) return false;
      timeval timeout{static_cast<time_t>(remaining.count() / 1000000),
                      static_cast<suseconds_t>(remaining.count() % 1000000)};
      const int ready = select(fd_ + 1, &set, nullptr, nullptr, &timeout);
      if (ready == 0) return false;
      if (ready < 0) { if (errno == EINTR) continue; return false; }
      const ssize_t count = read(fd_, data + offset, length - offset);
      if (count > 0) {
        if (serial_debug_) {
          std::fprintf(stderr, "RS03 serial RX (%zd):", count);
          for (ssize_t i = 0; i < count; ++i)
            std::fprintf(stderr, " %02X", data[offset + static_cast<size_t>(i)]);
          std::fprintf(stderr, "\n");
        }
        offset += static_cast<size_t>(count);
      }
      else if (count < 0 && errno != EINTR && errno != EAGAIN) return false;
    }
    return offset == length;
  }

  bool read_frame(can_frame &frame, const Deadline &deadline) {
    if (!serial_mode_) {
      const ssize_t count = recv(fd_, &frame, sizeof(frame), 0);
      return count == sizeof(frame);
    }
    bool found_a = false;
    while (std::chrono::steady_clock::now() < deadline) {
      uint8_t byte = 0;
      if (!read_exact(&byte, 1, deadline)) return false;
      if (!found_a) { found_a = byte == 'A'; continue; }
      if (byte != 'T') { found_a = byte == 'A'; continue; }
      uint8_t header[5]{};
      if (!read_exact(header, sizeof(header), deadline)) return false;
      const uint32_t serial_id = (static_cast<uint32_t>(header[0]) << 24) |
                                 (static_cast<uint32_t>(header[1]) << 16) |
                                 (static_cast<uint32_t>(header[2]) << 8) |
                                 header[3];
      const uint8_t dlc = header[4];
      if (dlc > 8) { found_a = false; continue; }
      uint8_t payload[10]{};
      if (!read_exact(payload, dlc + 2, deadline)) return false;
      if ((serial_id & 0x07) != kSerialExtendedFrameFlag ||
          payload[dlc] != '\r' || payload[dlc + 1] != '\n') {
        found_a = false; continue;
      }
      frame = {};
      frame.can_id = CAN_EFF_FLAG | (serial_id >> 3);
      frame.can_dlc = dlc;
      std::copy(payload, payload + dlc, frame.data);
      return true;
    }
    return false;
  }

  int fd_{-1};
  bool serial_mode_{false};
  bool serial_debug_{false};
  int receive_timeout_ms_{20};
  uint8_t master_id_;
  uint8_t motor_id_;
};

class Rs03Node final : public rclcpp::Node {
 public:
  Rs03Node() : Node("rs03_current_torque") {
    const auto transport = declare_parameter("transport", "serial");
    const auto iface = declare_parameter("can_interface", "can0");
    const auto serial_device = declare_parameter("serial_device", "/dev/ttyUSB0");
    const auto serial_baud = declare_parameter("serial_baud", 921600);
    const auto serial_debug = declare_parameter("serial_debug", false);
    const auto motor_id = declare_parameter("motor_id", 1);
    const auto master_id = declare_parameter("master_id", 255);
    mode_ = declare_parameter("control_mode", "current");
    const bool auto_enable = declare_parameter("auto_enable", false);
    timeout_s_ = declare_parameter("command_timeout_s", 0.30);
    max_current_a_ = std::clamp(declare_parameter("max_current_a", 1.0),
                                0.0, static_cast<double>(kProtocolCurrentMaxA));
    max_torque_nm_ = std::clamp(declare_parameter("max_torque_nm", 2.0),
                                0.0, static_cast<double>(kProtocolTorqueMaxNm));
    torque_demo_duration_s_ = declare_parameter("torque_demo_duration_s", 0.0);
    torque_breakaway_boost_nm_ = declare_parameter("torque_breakaway_boost_nm", 0.0);
    torque_breakaway_velocity_rad_s_ =
        declare_parameter("torque_breakaway_velocity_rad_s", 0.12);
    torque_breakaway_rearm_velocity_rad_s_ =
        declare_parameter("torque_breakaway_rearm_velocity_rad_s", 0.05);
    torque_breakaway_rearm_delay_s_ =
        declare_parameter("torque_breakaway_rearm_delay_s", 0.30);
    torque_breakaway_timeout_s_ =
        declare_parameter("torque_breakaway_timeout_s", 2.0);
    torque_breakaway_release_rate_nm_s_ =
        declare_parameter("torque_breakaway_release_rate_nm_s", 10.0);
    torque_soft_velocity_start_rad_s_ =
        declare_parameter("torque_soft_velocity_start_rad_s", 0.0);
    torque_soft_velocity_limit_rad_s_ =
        declare_parameter("torque_soft_velocity_limit_rad_s", 0.0);
    torque_soft_brake_gain_nm_per_rad_s_ =
        declare_parameter("torque_soft_brake_gain_nm_per_rad_s", 0.0);
    torque_soft_brake_max_nm_ =
        declare_parameter("torque_soft_brake_max_nm", 0.0);
    torque_speed_feedforward_nm_ =
        declare_parameter("torque_speed_feedforward_nm", 0.0);
    torque_velocity_filter_alpha_ =
        declare_parameter("torque_velocity_filter_alpha", 0.2);
    max_velocity_command_rad_s_ = declare_parameter("max_velocity_command_rad_s", 0.5);
    velocity_current_limit_a_ = declare_parameter("velocity_current_limit_a", 0.5);
    velocity_acceleration_rad_s2_ = declare_parameter("velocity_acceleration_rad_s2", 0.5);
    position_max_offset_rad_ = declare_parameter("position_max_offset_rad", 0.2);
    position_current_limit_a_ = declare_parameter("position_current_limit_a", 0.5);
    position_speed_limit_rad_s_ = declare_parameter("position_speed_limit_rad_s", 0.2);
    position_acceleration_rad_s2_ = declare_parameter("position_acceleration_rad_s2", 0.5);
    position_slew_rate_rad_s_ = declare_parameter("position_slew_rate_rad_s", 0.05);
    position_ramp_max_error_rad_ =
        declare_parameter("position_ramp_max_error_rad", 0.03);
    position_tracking_error_rad_ = declare_parameter("position_tracking_error_rad", 0.5);
    current_slew_rate_ = declare_parameter("current_slew_rate_a_s", 0.5);
    torque_slew_rate_ = declare_parameter("torque_slew_rate_nm_s", 1.0);
    velocity_slew_rate_ = declare_parameter("velocity_slew_rate_rad_s2", 0.5);
    max_velocity_rad_s_ = declare_parameter("max_velocity_rad_s", 2.0);
    velocity_trip_samples_ = declare_parameter("velocity_trip_samples", 5);
    max_temperature_c_ = declare_parameter("max_temperature_c", 60.0);
    const auto receive_timeout = declare_parameter("receive_timeout_ms", 20);
    if (motor_id < 0 || motor_id > 255 || master_id < 0 || master_id > 255)
      throw std::invalid_argument("motor_id and master_id must be in [0, 255]");
    if (timeout_s_ <= 0.0 || receive_timeout < 0 || torque_demo_duration_s_ < 0.0)
      throw std::invalid_argument("timeouts must be positive");
    if (current_slew_rate_ <= 0.0 || torque_slew_rate_ <= 0.0 ||
        velocity_slew_rate_ <= 0.0 || max_velocity_command_rad_s_ <= 0.0 ||
        velocity_current_limit_a_ <= 0.0 || velocity_current_limit_a_ > 43.0 ||
        velocity_acceleration_rad_s2_ <= 0.0 ||
        position_max_offset_rad_ <= 0.0 || position_current_limit_a_ <= 0.0 ||
        position_current_limit_a_ > 43.0 || position_speed_limit_rad_s_ <= 0.0 ||
        position_acceleration_rad_s2_ <= 0.0 || position_slew_rate_rad_s_ <= 0.0 ||
        position_ramp_max_error_rad_ <= 0.0 || position_tracking_error_rad_ <= 0.0 ||
        position_ramp_max_error_rad_ >= position_tracking_error_rad_ ||
        max_velocity_rad_s_ <= 0.0 || max_temperature_c_ <= 0.0)
      throw std::invalid_argument("safety limits and slew rates must be positive");
    if (velocity_trip_samples_ < 1)
      throw std::invalid_argument("velocity_trip_samples must be at least 1");
    if (torque_soft_velocity_start_rad_s_ < 0.0 ||
        torque_soft_velocity_limit_rad_s_ < 0.0 ||
        (torque_soft_velocity_limit_rad_s_ > 0.0 &&
         (torque_soft_velocity_start_rad_s_ >= torque_soft_velocity_limit_rad_s_ ||
          torque_soft_velocity_limit_rad_s_ >= max_velocity_rad_s_)))
      throw std::invalid_argument(
          "torque soft velocity limits must satisfy 0 <= start < limit < max_velocity");
    if (torque_breakaway_boost_nm_ < 0.0 ||
        torque_breakaway_boost_nm_ > max_torque_nm_ ||
        torque_breakaway_velocity_rad_s_ <= 0.0 ||
        torque_breakaway_rearm_velocity_rad_s_ < 0.0 ||
        torque_breakaway_rearm_velocity_rad_s_ >= torque_breakaway_velocity_rad_s_ ||
        torque_breakaway_rearm_delay_s_ <= 0.0 ||
        torque_breakaway_timeout_s_ <= 0.0 ||
        torque_breakaway_release_rate_nm_s_ <= 0.0 ||
        (torque_breakaway_boost_nm_ > 0.0 &&
         torque_soft_velocity_limit_rad_s_ > 0.0))
      throw std::invalid_argument(
          "breakaway compensation limits are invalid or conflict with the soft governor");
    if (torque_soft_brake_gain_nm_per_rad_s_ < 0.0 ||
        torque_soft_brake_max_nm_ < 0.0 ||
        torque_soft_brake_max_nm_ > max_torque_nm_ ||
        torque_speed_feedforward_nm_ < 0.0 ||
        torque_speed_feedforward_nm_ > max_torque_nm_ ||
        torque_velocity_filter_alpha_ <= 0.0 ||
        torque_velocity_filter_alpha_ > 1.0 ||
        ((torque_soft_brake_gain_nm_per_rad_s_ == 0.0) !=
         (torque_soft_brake_max_nm_ == 0.0)))
      throw std::invalid_argument(
          "torque brake gain/max must both be zero or positive; filter alpha must be in (0, 1]");
    if (mode_ != "current" && mode_ != "torque" && mode_ != "velocity" &&
        mode_ != "position_pp")
      throw std::invalid_argument(
          "control_mode must be current, torque, velocity, or position_pp");
    if (transport != "serial" && transport != "socketcan")
      throw std::invalid_argument("transport must be serial or socketcan");
    can_ = std::make_unique<Rs03Can>(
        transport, iface, serial_device, serial_baud, serial_debug,
        static_cast<uint8_t>(master_id), static_cast<uint8_t>(motor_id),
        receive_timeout);

    std::string command_topic = "~/torque_command_nm";
    if (mode_ == "current") command_topic = "~/current_command_a";
    else if (mode_ == "velocity") command_topic = "~/velocity_command_rad_s";
    else if (mode_ == "position_pp") command_topic = "~/position_offset_command_rad";
    command_sub_ = create_subscription<std_msgs::msg::Float32>(
        command_topic, 10,
        [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
          if (!std::isfinite(msg->data)) {
            RCLCPP_ERROR(get_logger(), "rejected non-finite command");
            return;
          }
          const bool first_command = !command_seen_;
          if (first_command)
            RCLCPP_INFO(get_logger(), "first %s command received: %.3f",
                        mode_.c_str(), msg->data);
          command_ = msg->data;
          last_command_ = now();
          if (first_command && mode_ == "torque" && torque_demo_duration_s_ > 0.0)
            torque_demo_start_ = last_command_;
          command_seen_ = true;
          if (position_waiting_for_command_) {
            const float offset = std::clamp(
                command_, -static_cast<float>(position_max_offset_rad_),
                static_cast<float>(position_max_offset_rad_));
            applied_command_ = 0.0F;
            // Clear any stale PP target before enabling. RS03 PP mode applies
            // vel_max/acc_set/loc_ref after enable, so write the real target
            // again only after the motor is enabled.
            can_->set_position(startup_position_rad_);
            can_->enable();
            can_->configure_position_pp(
                static_cast<float>(position_current_limit_a_),
                static_cast<float>(position_speed_limit_rad_s_),
                static_cast<float>(position_acceleration_rad_s2_));
            can_->set_position(startup_position_rad_);
            enabled_ = true;
            position_waiting_for_command_ = false;
            last_update_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(get_logger(),
                        "position mode enabled on first command: target=%.3f rad "
                        "(startup=%.3f, offset=%.3f)",
                        startup_position_rad_ + offset, startup_position_rad_, offset);
          }
        });
    torque_pub_ = create_publisher<std_msgs::msg::Float32>("~/estimated_torque_nm", 10);
    position_pub_ = create_publisher<std_msgs::msg::Float32>("~/position_rad", 10);
    velocity_pub_ = create_publisher<std_msgs::msg::Float32>("~/velocity_rad_s", 10);
    temperature_pub_ = create_publisher<std_msgs::msg::Float32>("~/temperature_c", 10);
    timer_ = create_wall_timer(10ms, [this] { update(); });

    can_->stop();
    Rs03Can::Feedback probe{};
    const bool motor_online = can_->receive_feedback(probe);
    if (motor_online) {
      startup_position_rad_ = probe.position_rad;
      last_position_feedback_rad_ = probe.position_rad;
      last_velocity_feedback_rad_s_ = probe.velocity_rad_s;
      filtered_velocity_rad_s_ = probe.velocity_rad_s;
      has_position_feedback_ = true;
      RCLCPP_INFO(get_logger(),
                  "RS03 feedback received: position=%.3f rad, velocity=%.3f rad/s, "
                  "estimated_torque=%.3f Nm, temperature=%.1f C",
                  probe.position_rad, probe.velocity_rad_s, probe.torque_nm,
                  probe.temperature_c);
    }
    else
      RCLCPP_WARN(get_logger(), "no RS03 feedback after safe stop probe");

    if (auto_enable) {
      if (!motor_online)
        throw std::runtime_error("refusing to enable: no valid RS03 feedback");
      uint8_t run_mode = 0;
      if (mode_ == "current") run_mode = 3;
      else if (mode_ == "velocity") run_mode = 2;
      else if (mode_ == "position_pp") run_mode = 1;
      can_->set_mode(run_mode);
      uint8_t confirmed_run_mode = 0xff;
      if (!can_->read_u8_parameter(kRunMode, confirmed_run_mode) ||
          confirmed_run_mode != run_mode) {
        throw std::runtime_error(
            "refusing to enable: RS03 run_mode write could not be verified");
      }
      RCLCPP_INFO(get_logger(), "RS03 run_mode confirmed: %u",
                  static_cast<unsigned>(confirmed_run_mode));
      if (mode_ == "current") {
        can_->set_iq(0.0F);
      } else if (mode_ == "velocity") {
        can_->configure_velocity(static_cast<float>(velocity_current_limit_a_),
                                 static_cast<float>(velocity_acceleration_rad_s2_));
        can_->set_velocity(0.0F);
      } else if (mode_ == "position_pp") {
        float mechanical_position = 0.0F;
        if (!can_->read_float_parameter(kMechanicalPosition, mechanical_position))
          throw std::runtime_error(
              "refusing PP mode: cannot read absolute mechanical position (0x7019)");
        startup_position_rad_ = mechanical_position;
        can_->configure_position_pp(static_cast<float>(position_current_limit_a_),
                                    static_cast<float>(position_speed_limit_rad_s_),
                                    static_cast<float>(position_acceleration_rad_s2_));
        position_waiting_for_command_ = true;
        RCLCPP_WARN(get_logger(),
                    "position mode armed at mechanical position %.3f rad; motor remains "
                    "disabled until the first valid position command",
                    startup_position_rad_);
      } else {
        can_->set_torque(0.0F);
      }
      if (mode_ != "position_pp") {
        can_->enable();
        enabled_ = true;
      }
      last_command_ = now();
      last_update_ = std::chrono::steady_clock::now();
    } else {
      RCLCPP_WARN(get_logger(), "auto_enable=false: motor remains stopped; set true only after bench checks");
    }
  }

  ~Rs03Node() override {
    try { zero_and_stop(); } catch (...) {}
  }

 private:
  void update() {
    if (!enabled_) return;
    const auto ros_now = now();
    if (mode_ == "torque" && command_seen_ && torque_demo_duration_s_ > 0.0 &&
        (ros_now - torque_demo_start_).seconds() >= torque_demo_duration_s_) {
      send_zero_command();
      can_->stop();
      enabled_ = false;
      applied_command_ = 0.0F;
      RCLCPP_FATAL(get_logger(),
                   "torque demo time limit reached: %.3f s; output forced to zero "
                   "and motor stopped; restart node to re-enable",
                   torque_demo_duration_s_);
      return;
    }
    const auto update_time = std::chrono::steady_clock::now();
    const double dt = std::clamp(
        std::chrono::duration<double>(update_time - last_update_).count(),
        0.0, 0.1);
    last_update_ = update_time;
    const bool fresh = command_seen_ && (ros_now - last_command_).seconds() <= timeout_s_;
    if (!fresh && command_seen_ && !timeout_reported_) {
      send_zero_command();
      can_->stop();
      enabled_ = false;
      applied_command_ = 0.0F;
      timeout_reported_ = true;
      RCLCPP_FATAL(get_logger(),
                   "command timeout: output forced to zero and motor stopped; "
                   "restart node to re-enable");
      return;
    }
    const float requested = fresh ? command_ : 0.0F;
    float limit = static_cast<float>(max_torque_nm_);
    if (mode_ == "current") limit = static_cast<float>(max_current_a_);
    else if (mode_ == "velocity") limit = static_cast<float>(max_velocity_command_rad_s_);
    else if (mode_ == "position_pp") limit = static_cast<float>(position_max_offset_rad_);
    const float desired = std::clamp(requested, -limit, limit);
    float control_desired = desired;
    if (fresh && mode_ == "torque" && torque_breakaway_boost_nm_ > 0.0 &&
        std::abs(desired) > 1e-4F) {
      const int direction_sign = desired > 0.0F ? 1 : -1;
      const float direction = static_cast<float>(direction_sign);
      if (direction_sign != torque_breakaway_direction_) {
        torque_breakaway_direction_ = direction_sign;
        torque_breakaway_active_ = true;
        torque_breakaway_elapsed_s_ = 0.0;
        torque_breakaway_low_speed_s_ = 0.0;
        RCLCPP_INFO(get_logger(), "breakaway boost armed: %.3f Nm",
                    torque_breakaway_boost_nm_);
      }
      const float signed_speed = direction * filtered_velocity_rad_s_;
      if (torque_breakaway_active_) {
        torque_breakaway_elapsed_s_ += dt;
        if (signed_speed >= static_cast<float>(torque_breakaway_velocity_rad_s_)) {
          torque_breakaway_active_ = false;
          torque_breakaway_elapsed_s_ = 0.0;
          torque_breakaway_low_speed_s_ = 0.0;
          RCLCPP_INFO(get_logger(),
                      "breakaway complete at %.3f rad/s; returning to %.3f Nm",
                      filtered_velocity_rad_s_, desired);
        } else if (torque_breakaway_elapsed_s_ >= torque_breakaway_timeout_s_) {
          send_zero_command();
          can_->stop();
          enabled_ = false;
          applied_command_ = 0.0F;
          RCLCPP_FATAL(get_logger(),
                       "breakaway timeout: no motion after %.3f s; output forced "
                       "to zero and motor stopped; restart node to re-enable",
                       torque_breakaway_timeout_s_);
          return;
        }
      } else {
        if (signed_speed <=
            static_cast<float>(torque_breakaway_rearm_velocity_rad_s_))
          torque_breakaway_low_speed_s_ += dt;
        else
          torque_breakaway_low_speed_s_ = 0.0;
        if (torque_breakaway_low_speed_s_ >= torque_breakaway_rearm_delay_s_) {
          torque_breakaway_active_ = true;
          torque_breakaway_elapsed_s_ = 0.0;
          torque_breakaway_low_speed_s_ = 0.0;
          RCLCPP_INFO(get_logger(), "breakaway boost re-armed after low speed");
        }
      }
      if (torque_breakaway_active_)
        control_desired = direction * std::max(
            std::abs(desired), static_cast<float>(torque_breakaway_boost_nm_));
    } else if (mode_ == "torque" && torque_breakaway_boost_nm_ > 0.0) {
      torque_breakaway_active_ = false;
      torque_breakaway_direction_ = 0;
      torque_breakaway_elapsed_s_ = 0.0;
      torque_breakaway_low_speed_s_ = 0.0;
    }
    if (fresh && mode_ != "position_pp") {
      float rate = static_cast<float>(torque_slew_rate_);
      if (mode_ == "current") rate = static_cast<float>(current_slew_rate_);
      else if (mode_ == "velocity") rate = static_cast<float>(velocity_slew_rate_);
      else if (mode_ == "torque" && torque_breakaway_boost_nm_ > 0.0 &&
               !torque_breakaway_active_ &&
               std::abs(control_desired) < std::abs(applied_command_)) {
        // Once static friction has been overcome, shed the temporary boost
        // quickly.  Reusing the slow startup slew here leaves excess torque on
        // the shaft and can cause an immediate overspeed trip.
        rate = static_cast<float>(torque_breakaway_release_rate_nm_s_);
      }
      const float max_step = rate * static_cast<float>(dt);
      applied_command_ +=
          std::clamp(control_desired - applied_command_, -max_step, max_step);
    } else if (fresh) {
      const float current_target = startup_position_rad_ + applied_command_;
      const float following_error = has_position_feedback_
          ? std::abs(cyclic_position_error(current_target,
                                           last_position_feedback_rad_))
          : 0.0F;
      // Do not let a slowly ramped command run far ahead of a stuck shaft.
      // Resume the ramp only after the motor catches up with the current target.
      if (!has_position_feedback_ ||
          following_error <= static_cast<float>(position_ramp_max_error_rad_)) {
        const float max_step = static_cast<float>(position_slew_rate_rad_s_ * dt);
        applied_command_ +=
            std::clamp(desired - applied_command_, -max_step, max_step);
      }
    } else {
      // The communication watchdog always wins over slew limiting.
      applied_command_ = 0.0F;
    }
    float sent_command = applied_command_;
    if (mode_ == "torque" && torque_soft_velocity_limit_rad_s_ > 0.0 &&
        std::abs(applied_command_) > 1e-4F) {
      const float direction = std::copysign(1.0F, applied_command_);
      const int command_direction = applied_command_ > 0.0F ? 1 : -1;
      if (command_direction != torque_demo_direction_) {
        torque_demo_direction_ = command_direction;
        torque_motion_started_ = false;
      }
      const float signed_speed = direction * filtered_velocity_rad_s_;
      if (!torque_motion_started_ &&
          signed_speed >= static_cast<float>(torque_soft_velocity_start_rad_s_)) {
        torque_motion_started_ = true;
        RCLCPP_INFO(get_logger(),
                    "torque demo entered governed phase at %.3f rad/s",
                    filtered_velocity_rad_s_);
      }
      if (torque_motion_started_ && torque_soft_brake_gain_nm_per_rad_s_ > 0.0) {
        // After breakaway, use one continuous proportional law instead of
        // switching between full positive torque and a fixed braking torque.
        // The latched phase prevents a temporary hand obstruction from
        // re-arming the full breakaway torque.
        const float speed_error =
            static_cast<float>(torque_soft_velocity_limit_rad_s_) - signed_speed;
        const float governed_torque =
            static_cast<float>(torque_speed_feedforward_nm_) +
            static_cast<float>(torque_soft_brake_gain_nm_per_rad_s_) * speed_error;
        sent_command = direction * std::clamp(
            governed_torque,
            -static_cast<float>(torque_soft_brake_max_nm_),
            std::abs(applied_command_));
      }
    } else if (mode_ == "torque") {
      torque_motion_started_ = false;
      torque_demo_direction_ = 0;
    }
    if (mode_ == "current") {
      can_->set_iq(applied_command_);
    } else if (mode_ == "torque") {
      can_->set_torque(sent_command);
    } else if (mode_ == "velocity") {
      can_->set_velocity(applied_command_);
    } else {
      can_->set_position(startup_position_rad_ + applied_command_);
    }
    if (fresh) timeout_reported_ = false;

    Rs03Can::Feedback fb{};
    if (can_->receive_feedback(fb)) {
      last_position_feedback_rad_ = fb.position_rad;
      last_velocity_feedback_rad_s_ = fb.velocity_rad_s;
      filtered_velocity_rad_s_ += static_cast<float>(torque_velocity_filter_alpha_) *
          (fb.velocity_rad_s - filtered_velocity_rad_s_);
      has_position_feedback_ = true;
      std_msgs::msg::Float32 msg;
      msg.data = fb.torque_nm;
      torque_pub_->publish(msg);
      msg.data = fb.position_rad;
      position_pub_->publish(msg);
      msg.data = fb.velocity_rad_s;
      velocity_pub_->publish(msg);
      msg.data = fb.temperature_c;
      temperature_pub_->publish(msg);

      if (mode_ == "torque" && fresh) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "torque command: requested=%.3f Nm, ramped=%.3f Nm, sent=%.3f Nm, "
            "feedback=%.3f Nm, velocity=%.3f rad/s, filtered_velocity=%.3f rad/s",
            desired, applied_command_, sent_command, fb.torque_nm,
            fb.velocity_rad_s, filtered_velocity_rad_s_);
      }

      if (std::abs(fb.velocity_rad_s) > max_velocity_rad_s_)
        ++velocity_trip_count_;
      else
        velocity_trip_count_ = 0;
      const bool velocity_trip = velocity_trip_count_ >= velocity_trip_samples_;
      const float position_target = startup_position_rad_ + applied_command_;
      const float position_error = mode_ == "position_pp"
          ? cyclic_position_error(position_target, fb.position_rad)
          : 0.0F;
      const bool position_trip = mode_ == "position_pp" && command_seen_ &&
          std::abs(position_error) > position_tracking_error_rad_;
      if (velocity_trip || position_trip || fb.temperature_c > max_temperature_c_) {
        send_zero_command();
        can_->stop();
        enabled_ = false;
        applied_command_ = 0.0F;
        RCLCPP_FATAL(get_logger(),
                     "safety stop: velocity=%.3f rad/s (limit %.3f), "
                     "position_error=%.3f rad (limit %.3f), temperature=%.1f C (limit %.1f)",
                     fb.velocity_rad_s, max_velocity_rad_s_,
                     position_error,
                     position_tracking_error_rad_,
                     fb.temperature_c, max_temperature_c_);
      }
    }
  }

  void zero_and_stop() {
    if (!can_) return;
    send_zero_command();
    can_->stop();
    enabled_ = false;
  }

  void send_zero_command() {
    if (mode_ == "current") can_->set_iq(0.0F);
    else if (mode_ == "torque") can_->set_torque(0.0F);
    else if (mode_ == "velocity") can_->set_velocity(0.0F);
    // PP position mode is stopped without issuing a new position target.
  }

  std::unique_ptr<Rs03Can> can_;
  std::string mode_;
  double timeout_s_{0.1}, max_current_a_{1.0}, max_torque_nm_{2.0};
  double torque_demo_duration_s_{0.0};
  double torque_breakaway_boost_nm_{0.0};
  double torque_breakaway_velocity_rad_s_{0.12};
  double torque_breakaway_rearm_velocity_rad_s_{0.05};
  double torque_breakaway_rearm_delay_s_{0.30}, torque_breakaway_timeout_s_{2.0};
  double torque_breakaway_release_rate_nm_s_{10.0};
  double torque_soft_velocity_start_rad_s_{0.0};
  double torque_soft_velocity_limit_rad_s_{0.0};
  double torque_soft_brake_gain_nm_per_rad_s_{0.0};
  double torque_soft_brake_max_nm_{0.0}, torque_speed_feedforward_nm_{0.0};
  double torque_velocity_filter_alpha_{0.2};
  bool position_waiting_for_command_{false};
  double max_velocity_command_rad_s_{0.5}, velocity_current_limit_a_{0.5};
  double velocity_acceleration_rad_s2_{0.5};
  double position_max_offset_rad_{0.2}, position_current_limit_a_{0.5};
  double position_speed_limit_rad_s_{0.2};
  double position_acceleration_rad_s2_{0.5}, position_slew_rate_rad_s_{0.05};
  double position_ramp_max_error_rad_{0.03}, position_tracking_error_rad_{0.5};
  double current_slew_rate_{0.5}, torque_slew_rate_{1.0}, velocity_slew_rate_{0.5};
  double max_velocity_rad_s_{2.0}, max_temperature_c_{60.0};
  int64_t velocity_trip_samples_{5};
  int64_t velocity_trip_count_{0};
  int torque_demo_direction_{0};
  int torque_breakaway_direction_{0};
  float command_{0.0F}, applied_command_{0.0F}, startup_position_rad_{0.0F};
  float last_position_feedback_rad_{0.0F}, last_velocity_feedback_rad_s_{0.0F};
  float filtered_velocity_rad_s_{0.0F};
  bool has_position_feedback_{false};
  bool torque_motion_started_{false};
  bool torque_breakaway_active_{false};
  double torque_breakaway_elapsed_s_{0.0}, torque_breakaway_low_speed_s_{0.0};
  bool enabled_{false}, command_seen_{false}, timeout_reported_{false};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Time torque_demo_start_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_update_{std::chrono::steady_clock::now()};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
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
