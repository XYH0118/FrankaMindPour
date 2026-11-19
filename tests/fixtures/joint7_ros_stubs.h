#pragma once

// Minimal ROS boundary for running the real controller against fake joints.
#include <array>
#include <cstdint>
#include <exception>
#include <functional>
#include <string>

namespace ros {
struct Duration {
  explicit Duration(double seconds = 0) : seconds(seconds) {}
  double toSec() const { return seconds; }
  double seconds;
};
struct Time {
  Time() = default;
  explicit Time(double seconds) : seconds(seconds) {}
  bool isZero() const { return seconds == 0; }
  static Time now() { return Time(); }
  double seconds = 0;
};
inline Duration operator-(Time lhs, Time rhs) { return Duration(lhs.seconds - rhs.seconds); }
inline Time operator+(Time time, Duration duration) {
  return Time(time.seconds + duration.seconds);
}
inline bool operator<(Time lhs, Time rhs) { return lhs.seconds < rhs.seconds; }
struct Subscriber {};
struct TransportHints {
  TransportHints& reliable() { return *this; }
  TransportHints& tcpNoDelay() { return *this; }
};
struct NodeHandle {
  template <typename Message, typename Owner>
  Subscriber subscribe(const std::string&, int, void (Owner::*callback)(const Message* const&),
                       Owner* owner, const TransportHints&) {
    deliver = [owner, callback](const void* msg) {
      (owner->*callback)(static_cast<const Message*>(msg));
    };
    return {};
  }
  template <typename T>
  void param(const std::string&, T& value, const T& fallback) { value = fallback; }
  template <typename T>
  bool getParam(const std::string&, T& names) {
    names.resize(7, "joint");
    return true;
  }
  std::function<void(const void*)> deliver;
};
}  // namespace ros

namespace hardware_interface {
struct JointHandle {
  double* measured;
  double* desired;
  double getPosition() const { return *measured; }
  void setCommand(double value) { *desired = value; }
};
struct HardwareInterfaceException : std::exception {};
struct PositionJointInterface {
  JointHandle getHandle(const std::string&) {
    const auto i = next++;
    return {&measured[i], &desired[i]};
  }
  std::array<double, 7> measured{};
  std::array<double, 7> desired{};
  std::size_t next = 0;
};
struct RobotHW {
  template <typename T>
  T* get() { return &joints; }
  PositionJointInterface joints;
};
}  // namespace hardware_interface

namespace controller_interface {
template <typename T>
struct MultiInterfaceController {
  virtual bool init(hardware_interface::RobotHW*, ros::NodeHandle&) = 0;
  virtual void starting(const ros::Time&) = 0;
  virtual void update(const ros::Time&, const ros::Duration&) = 0;
};
}  // namespace controller_interface

namespace realtime_tools {
template <typename T>
struct RealtimeBuffer {
  const T* readFromRT() { return &value; }
  void writeFromNonRT(const T& input) { value = input; }
  T value{};
};
template <typename T>
struct RealtimePublisher {
  RealtimePublisher(ros::NodeHandle&, const char*, int) {}
  bool trylock() { return true; }
  void unlockAndPublish() { last = msg_; }
  T msg_{};
  inline static T last{};
};
}  // namespace realtime_tools

namespace franka_mind_pour {
struct PourCommand {
  using ConstPtr = const PourCommand*;
  static constexpr int8_t NONE = 0;
  struct Header { ros::Time stamp; } header;
  uint32_t seq = 0;
  int8_t command = NONE;
};
struct PourState {
  static constexpr uint8_t STATE_WAITING = 0, STATE_ROTATING = 1, STATE_DONE = 2;
  PourCommand::Header header;
  uint8_t state = STATE_WAITING;
  int8_t command = 0;
  uint32_t command_seq = 0;
  std::array<double, 7> q{};
  double q7_start = 0, q7_desired = 0, dq7_desired = 0, angle = 0, elapsed = 0;
};
}  // namespace franka_mind_pour

#define ROS_ERROR_STREAM(message) do {} while (false)
#define ROS_WARN_STREAM(message) do {} while (false)
#define ROS_INFO_STREAM(message) do {} while (false)
#define ROS_INFO_STREAM_THROTTLE(rate, message) do {} while (false)
#define PLUGINLIB_EXPORT_CLASS(derived, base)
