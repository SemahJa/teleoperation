#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <quest_teleop_interfaces/msg/quest_controller_state.hpp>
#include <quest_teleop_interfaces/msg/quest_button_state.hpp>

namespace {
using json = nlohmann::json;
using Message = quest_teleop_interfaces::msg::QuestControllerState;
constexpr uint32_t kMaxPayload = 4096;

bool read_all(int fd, void* destination, size_t size) {
  auto* bytes = static_cast<unsigned char*>(destination);
  size_t received = 0;
  while (received < size) {
    const ssize_t count = recv(fd, bytes + received, size - received, 0);
    if (count == 0) return false;
    if (count < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    received += static_cast<size_t>(count);
  }
  return true;
}

uint64_t pc_monotonic_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) throw std::runtime_error("clock_gettime failed");
  return static_cast<uint64_t>(value.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(value.tv_nsec);
}

int64_t decimal_i64(const json& value, const char* field) {
  if (!value.is_string()) throw std::runtime_error(std::string(field) + " must be a decimal string");
  const std::string text = value.get<std::string>();
  if (text.empty() || text.size() > 20) throw std::runtime_error(std::string(field) + " has invalid length");
  const size_t first_digit = text.front() == '-' ? 1 : 0;
  if (first_digit == text.size() || !std::all_of(text.begin() + first_digit, text.end(),
      [](char digit) { return digit >= '0' && digit <= '9'; })) {
    throw std::runtime_error(std::string(field) + " is not a signed decimal");
  }
  size_t consumed = 0;
  const int64_t result = std::stoll(text, &consumed, 10);
  if (consumed != text.size()) throw std::runtime_error(std::string(field) + " is not decimal");
  return result;
}

uint64_t decimal_u64(const json& value, const char* field) {
  if (!value.is_string()) throw std::runtime_error(std::string(field) + " must be a decimal string");
  const std::string text = value.get<std::string>();
  if (text.empty() || text.size() > 20) throw std::runtime_error(std::string(field) + " has invalid length");
  if (!std::all_of(text.begin(), text.end(), [](char digit) { return digit >= '0' && digit <= '9'; })) {
    throw std::runtime_error(std::string(field) + " is not an unsigned decimal");
  }
  size_t consumed = 0;
  const uint64_t result = std::stoull(text, &consumed, 10);
  if (consumed != text.size()) throw std::runtime_error(std::string(field) + " is not decimal");
  return result;
}

template <size_t N>
std::array<double, N> finite_array(const json& value, const char* field) {
  if (!value.is_array() || value.size() != N) throw std::runtime_error(std::string(field) + " has wrong length");
  std::array<double, N> result{};
  for (size_t i = 0; i < N; ++i) {
    if (!value[i].is_number()) throw std::runtime_error(std::string(field) + " contains non-number");
    result[i] = value[i].get<double>();
    if (!std::isfinite(result[i])) throw std::runtime_error(std::string(field) + " contains non-finite value");
  }
  return result;
}

bool boolean(const json& value, const char* field) {
  if (!value.is_boolean()) throw std::runtime_error(std::string(field) + " must be boolean");
  return value.get<bool>();
}

float scalar(const json& value, const char* field) {
  if (!value.is_number()) throw std::runtime_error(std::string(field) + " must be numeric");
  const double result = value.get<double>();
  if (!std::isfinite(result) || result < 0.0 || result > 1.0) {
    throw std::runtime_error(std::string(field) + " must be finite and in [0,1]");
  }
  return static_cast<float>(result);
}

class QuestUsbBridge final : public rclcpp::Node {
 public:
  QuestUsbBridge() : Node("quest_usb_bridge") {
    const int port = declare_parameter<int>("listen_port", 45454);
    if (port < 1 || port > 65535) throw std::invalid_argument("listen_port must be in [1,65535]");
    const std::string bind_address = declare_parameter<std::string>("bind_address", "127.0.0.1");
    const int command_timeout_ms = declare_parameter<int>("command_timeout_ms", 100);
    const int max_uncertainty_ms = declare_parameter<int>("max_clock_uncertainty_ms", 10);
    const int max_sync_age_ms = declare_parameter<int>("max_clock_sync_age_ms", 3000);
    const int max_future_ms = declare_parameter<int>("max_source_future_ms", 10);
    const int client_receive_timeout_ms = declare_parameter<int>("client_receive_timeout_ms", 500);
    if (command_timeout_ms <= 0 || max_uncertainty_ms <= 0 || max_sync_age_ms <= 0 ||
        max_future_ms < 0 || client_receive_timeout_ms <= 0) {
      throw std::invalid_argument("clock/freshness limits must be positive (future allowance may be zero)");
    }
    command_timeout_ns_ = static_cast<uint64_t>(command_timeout_ms) * 1'000'000ULL;
    max_clock_uncertainty_ns_ = static_cast<uint64_t>(max_uncertainty_ms) * 1'000'000ULL;
    max_clock_sync_age_ns_ = static_cast<uint64_t>(max_sync_age_ms) * 1'000'000ULL;
    max_future_ns_ = static_cast<uint64_t>(max_future_ms) * 1'000'000ULL;
    client_receive_timeout_ms_ = client_receive_timeout_ms;
    publisher_ = create_publisher<Message>("~/controller_state", rclcpp::QoS(10));
    controller_publishers_[0] = create_publisher<Message>("~/left/controller_state", rclcpp::QoS(10));
    controller_publishers_[1] = create_publisher<Message>("~/right/controller_state", rclcpp::QoS(10));
    listener_fd_ = create_listener(bind_address, port);
    worker_ = std::thread([this] { accept_loop(); });
    RCLCPP_INFO(get_logger(), "Quest USB TCP bridge listening on %s:%d", bind_address.c_str(), port);
  }

  ~QuestUsbBridge() override {
    stopping_.store(true);
    if (listener_fd_ >= 0) {
      shutdown(listener_fd_, SHUT_RDWR);
      close(listener_fd_);
      listener_fd_ = -1;
    }
    if (client_fd_.load() >= 0) shutdown(client_fd_.load(), SHUT_RDWR);
    if (worker_.joinable()) worker_.join();
  }

 private:
  int create_listener(const std::string& address, int port) {
    const int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) throw std::runtime_error("socket() failed");
    int reuse = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
      close(fd);
      throw std::runtime_error("setsockopt(SO_REUSEADDR) failed");
    }
    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, address.c_str(), &endpoint.sin_addr) != 1) {
      close(fd);
      throw std::invalid_argument("bind_address must be an IPv4 address");
    }
    if (bind(fd, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0 || listen(fd, 1) != 0) {
      close(fd);
      throw std::runtime_error("bind()/listen() failed");
    }
    return fd;
  }

  void accept_loop() {
    while (!stopping_.load()) {
      pollfd listener{listener_fd_, POLLIN, 0};
      const int ready = poll(&listener, 1, 250);
      if (ready == 0) continue;
      if (ready < 0) {
        if (errno == EINTR) continue;
        if (!stopping_.load()) RCLCPP_ERROR(get_logger(), "poll() failed: %s", std::strerror(errno));
        break;
      }
      const int client = accept4(listener_fd_, nullptr, nullptr, SOCK_CLOEXEC);
      if (client < 0) {
        if (errno == EINTR) continue;
        if (!stopping_.load()) RCLCPP_ERROR(get_logger(), "accept() failed: %s", std::strerror(errno));
        break;
      }
      timeval receive_timeout{};
      receive_timeout.tv_sec = client_receive_timeout_ms_ / 1000;
      receive_timeout.tv_usec = (client_receive_timeout_ms_ % 1000) * 1000;
      setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));
      const int no_delay = 1;
      if (setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay)) != 0) {
        RCLCPP_WARN(get_logger(), "Could not disable TCP_NODELAY buffering: %s", std::strerror(errno));
      }
      client_fd_.store(client);
      RCLCPP_INFO(get_logger(), "Quest TCP client connected");
      consume_connection(client);
      shutdown(client, SHUT_RDWR);
      close(client);
      client_fd_.store(-1);
      publish_disconnect();
      RCLCPP_WARN(get_logger(), "Quest TCP client disconnected; motion input invalidated");
    }
  }

  static bool send_all(int fd, const void* data, size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    size_t sent = 0;
    while (sent < size) {
      const ssize_t count = send(fd, bytes + sent, size - sent, MSG_NOSIGNAL);
      if (count < 0) {
        if (errno == EINTR) continue;
        return false;
      }
      if (count == 0) return false;
      sent += static_cast<size_t>(count);
    }
    return true;
  }

  void reply_clock_sync(int fd, const json& request) {
    if (request.at("version").get<int>() != 1) throw std::runtime_error("unsupported sync version");
    const std::string sync_id = request.at("sync_id").get<std::string>();
    if (sync_id.empty() || sync_id.size() > 32) throw std::runtime_error("invalid sync_id");
    const uint64_t quest_send_ns = decimal_u64(request.at("quest_send_time_ns"), "quest_send_time_ns");
    const uint64_t pc_receive_ns = pc_monotonic_ns();
    json response = {
        {"version", 1}, {"type", "clock_sync_reply"}, {"sync_id", sync_id},
        {"quest_send_time_ns", std::to_string(quest_send_ns)},
        {"pc_receive_time_ns", std::to_string(pc_receive_ns)},
        {"pc_send_time_ns", "0"}};
    std::string payload = response.dump();
    const uint64_t pc_send_ns = pc_monotonic_ns();
    const std::string placeholder = "\"pc_send_time_ns\":\"0\"";
    const size_t send_time_field = payload.find(placeholder);
    if (send_time_field == std::string::npos) throw std::runtime_error("clock response serialization failed");
    payload.replace(send_time_field, placeholder.size(),
                    "\"pc_send_time_ns\":\"" + std::to_string(pc_send_ns) + "\"");
    if (payload.empty() || payload.size() > kMaxPayload) throw std::runtime_error("clock reply too large");
    const uint32_t length = htonl(static_cast<uint32_t>(payload.size()));
    std::string frame(sizeof(length), '\0');
    std::memcpy(frame.data(), &length, sizeof(length));
    frame += payload;
    if (!send_all(fd, frame.data(), frame.size())) throw std::runtime_error("clock reply send failed");
    RCLCPP_INFO(get_logger(), "Clock sync reply sent: sync_id=%s", sync_id.c_str());
  }

  void consume_connection(int fd) {
    std::string active_session;
    uint64_t last_sequence = 0;
    bool have_sequence = false;
    uint64_t reference_generation = 0;
    bool have_reference_generation = false;
    std::unordered_map<std::string, std::pair<uint64_t, uint64_t>> last_controller_times;
    while (!stopping_.load()) {
      uint32_t wire_length = 0;
      if (!read_all(fd, &wire_length, sizeof(wire_length))) return;
      const uint32_t length = ntohl(wire_length);
      if (length == 0 || length > kMaxPayload) {
        RCLCPP_ERROR(get_logger(), "Rejected invalid Quest frame length: %u", length);
        return;
      }
      std::string payload(length, '\0');
      if (!read_all(fd, payload.data(), length)) return;
      try {
        const json packet = json::parse(payload);
        if (packet.is_object() && packet.value("type", std::string()) == "clock_sync_request") {
          reply_clock_sync(fd, packet);
          continue;
        }
        auto message = decode(packet, active_session, last_sequence, have_sequence,
                              reference_generation, have_reference_generation,
                              last_controller_times);
        publish_state(message);
      } catch (const std::exception& error) {
        RCLCPP_WARN(get_logger(), "Rejected Quest frame; closing connection: %s", error.what());
        return;
      }
    }
  }

  Message decode(const json& packet, std::string& active_session, uint64_t& last_sequence,
                 bool& have_sequence, uint64_t& reference_generation,
                 bool& have_reference_generation,
                 std::unordered_map<std::string, std::pair<uint64_t, uint64_t>>& last_times) {
    if (!packet.is_object() || packet.at("version").get<int>() != 1 ||
        packet.at("type").get<std::string>() != "controller_input") {
      throw std::runtime_error("unsupported packet version/type");
    }
    const std::string session = packet.at("session_id").get<std::string>();
    if (session.empty() || session.size() > 128) throw std::runtime_error("invalid session_id");
    const uint64_t sequence = decimal_u64(packet.at("sequence"), "sequence");
    const uint64_t acquisition_ns = decimal_u64(packet.at("acquisition_time_ns"), "acquisition_time_ns");
    const uint64_t location_ns = decimal_u64(packet.at("openxr_location_time_ns"), "openxr_location_time_ns");
    const bool reported_clock_sync = boolean(packet.at("clock_sync_valid"), "clock_sync_valid");
    const int64_t clock_offset = decimal_i64(packet.at("clock_offset_pc_minus_quest_ns"), "clock_offset_pc_minus_quest_ns");
    const uint64_t clock_uncertainty = decimal_u64(packet.at("clock_uncertainty_ns"), "clock_uncertainty_ns");
    const uint64_t clock_sync_age = decimal_u64(packet.at("clock_sync_age_ns"), "clock_sync_age_ns");
    const uint64_t generation = decimal_u64(packet.at("reference_space_generation"), "reference_space_generation");
    const std::string controller_id = packet.at("controller_id").get<std::string>();
    if (controller_id != "left" && controller_id != "right") throw std::runtime_error("invalid controller_id");
    if (active_session.empty()) active_session = session;
    if (session != active_session) throw std::runtime_error("session changed without reconnect");
    if (have_sequence && sequence <= last_sequence) throw std::runtime_error("duplicate/out-of-order sequence");
    if (have_reference_generation && generation < reference_generation) {
      throw std::runtime_error("reference-space generation moved backwards");
    }
    const auto previous_time = last_times.find(controller_id);
    if (previous_time != last_times.end() &&
        (acquisition_ns <= previous_time->second.first || location_ns <= previous_time->second.second)) {
      throw std::runtime_error("controller timestamp duplicate/out-of-order");
    }

    Message out;
    out.header.stamp = now();
    out.header.frame_id = packet.at("reference_space").get<std::string>();
    if (out.header.frame_id != "LOCAL") throw std::runtime_error("unsupported reference space");
    out.session_id = session;
    out.sequence = sequence;
    out.acquisition_time_ns = acquisition_ns;
    out.openxr_location_time_ns = location_ns;
    out.clock_offset_pc_minus_quest_ns = clock_offset;
    out.clock_uncertainty_ns = clock_uncertainty;
    out.clock_sync_age_ns = clock_sync_age;
    out.clock_sync_valid = reported_clock_sync && clock_uncertainty <= max_clock_uncertainty_ns_ &&
                           clock_sync_age <= max_clock_sync_age_ns_;
    if (out.clock_sync_valid) {
      const long double converted = static_cast<long double>(acquisition_ns) +
                                    static_cast<long double>(clock_offset);
      if (converted >= 0.0L && converted <= static_cast<long double>(std::numeric_limits<uint64_t>::max())) {
        out.source_time_pc_monotonic_ns = static_cast<uint64_t>(converted);
        const uint64_t pc_now = pc_monotonic_ns();
        if (out.source_time_pc_monotonic_ns <= pc_now) {
          out.source_time_valid = pc_now - out.source_time_pc_monotonic_ns <= command_timeout_ns_;
        } else {
          out.source_time_valid = out.source_time_pc_monotonic_ns - pc_now <= max_future_ns_;
        }
      }
    }
    out.reference_space = out.header.frame_id;
    out.reference_space_generation = generation;
    out.controller_id = controller_id;
    out.connection_valid = true;
    out.session_focused = boolean(packet.at("session_focused"), "session_focused");

    const auto& pose = packet.at("pose");
    out.pose_action_active = boolean(pose.at("pose_action_active"), "pose_action_active");
    const auto position = finite_array<3>(pose.at("position"), "pose.position");
    const auto orientation = finite_array<4>(pose.at("orientation_xyzw"), "pose.orientation_xyzw");
    out.grip_pose.position.x = position[0];
    out.grip_pose.position.y = position[1];
    out.grip_pose.position.z = position[2];
    out.grip_pose.orientation.x = orientation[0];
    out.grip_pose.orientation.y = orientation[1];
    out.grip_pose.orientation.z = orientation[2];
    out.grip_pose.orientation.w = orientation[3];
    out.position_valid = boolean(pose.at("position_valid"), "position_valid");
    out.orientation_valid = boolean(pose.at("orientation_valid"), "orientation_valid");
    out.position_tracked = boolean(pose.at("position_tracked"), "position_tracked");
    out.orientation_tracked = boolean(pose.at("orientation_tracked"), "orientation_tracked");
    const double qnorm = std::sqrt(orientation[0] * orientation[0] + orientation[1] * orientation[1] +
                                   orientation[2] * orientation[2] + orientation[3] * orientation[3]);
    if (out.orientation_valid && (qnorm < 0.99 || qnorm > 1.01)) throw std::runtime_error("invalid orientation norm");

    const auto& twist = packet.at("native_twist");
    const auto linear = finite_array<3>(twist.at("linear"), "native_twist.linear");
    const auto angular = finite_array<3>(twist.at("angular"), "native_twist.angular");
    out.native_twist.linear.x = linear[0];
    out.native_twist.linear.y = linear[1];
    out.native_twist.linear.z = linear[2];
    out.native_twist.angular.x = angular[0];
    out.native_twist.angular.y = angular[1];
    out.native_twist.angular.z = angular[2];
    out.linear_velocity_valid = boolean(twist.at("linear_valid"), "linear_valid");
    out.angular_velocity_valid = boolean(twist.at("angular_valid"), "angular_valid");

    const auto& inputs = packet.at("inputs");
    if (!inputs.is_object()) throw std::runtime_error("inputs must be an object");
    const std::vector<std::string> button_names = controller_id == "right"
        ? std::vector<std::string>{"a", "b"} : std::vector<std::string>{"x", "y", "menu"};
    std::vector<std::string> input_fields{"trigger", "trigger_active", "grip", "grip_active"};
    for (const auto& name : button_names) {
      const std::string key = name == "menu" ? "menu_button" : "button_" + name;
      input_fields.push_back(key);
      input_fields.push_back(key + "_active");
      quest_teleop_interfaces::msg::QuestButtonState button;
      button.name = name;
      button.pressed = boolean(inputs.at(key), key.c_str());
      button.active = boolean(inputs.at(key + "_active"), (key + "_active").c_str());
      out.buttons.push_back(button);
    }
    for (auto field = inputs.begin(); field != inputs.end(); ++field) {
      if (std::find(input_fields.begin(), input_fields.end(), field.key()) == input_fields.end()) {
        throw std::runtime_error("unexpected input for " + controller_id + " controller: " + field.key());
      }
    }
    out.trigger_value = scalar(inputs.at("trigger"), "trigger");
    out.trigger_active = boolean(inputs.at("trigger_active"), "trigger_active");
    out.grip_value = scalar(inputs.at("grip"), "grip");
    out.grip_active = boolean(inputs.at("grip_active"), "grip_active");
    last_sequence = sequence;
    have_sequence = true;
    reference_generation = generation;
    have_reference_generation = true;
    last_times[controller_id] = {acquisition_ns, location_ns};
    return out;
  }

  void publish_state(const Message& message) {
    publisher_->publish(message);
    if (message.controller_id == "left") controller_publishers_[0]->publish(message);
    else if (message.controller_id == "right") controller_publishers_[1]->publish(message);
  }

  void publish_disconnect() {
    for (const char* controller : {"left", "right"}) {
      Message invalid;
      invalid.header.stamp = now();
      invalid.reference_space = "LOCAL";
      invalid.controller_id = controller;
      invalid.connection_valid = false;
      invalid.clock_sync_valid = false;
      invalid.source_time_valid = false;
      invalid.session_focused = false;
      invalid.pose_action_active = false;
      publish_state(invalid);
    }
  }

  rclcpp::Publisher<Message>::SharedPtr publisher_;
  std::array<rclcpp::Publisher<Message>::SharedPtr, 2> controller_publishers_;
  int listener_fd_{-1};
  std::atomic<int> client_fd_{-1};
  std::atomic<bool> stopping_{false};
  uint64_t command_timeout_ns_{100'000'000ULL};
  uint64_t max_clock_uncertainty_ns_{10'000'000ULL};
  uint64_t max_clock_sync_age_ns_{3'000'000'000ULL};
  uint64_t max_future_ns_{10'000'000ULL};
  int client_receive_timeout_ms_{500};
  std::thread worker_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<QuestUsbBridge>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("quest_usb_bridge"), "Bridge startup failed: %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
