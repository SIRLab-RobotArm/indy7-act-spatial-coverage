#include "pipet_hand_mark7_driver/mark7_system_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cmath>

namespace pipet_hand_mark7_driver
{

hardware_interface::CallbackReturn Mark7SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // URDF ros2_control 파라미터 읽기
  port_ = info_.hardware_parameters.count("port")
    ? info_.hardware_parameters.at("port")
    : "/dev/ttyACM0";

  use_mock_hardware_ = info_.hardware_parameters.count("use_mock_hardware")
    ? (info_.hardware_parameters.at("use_mock_hardware") == "true")
    : false;

  const std::size_t n = info_.joints.size();

  // 벡터 초기화
  hw_commands_.assign(n, 0.0);
  hw_commands_sent_.assign(n, -1.0);  // -1: 아직 전송 안 함
  hw_states_position_.assign(n, 0.0);
  hw_states_current_.assign(n, 0.0);
  hw_states_temperature_.assign(n, 0.0);

  // serial_ 은 on_activate() 에서 open()
  serial_ = SerialPort(port_);

  // /gripper/status 퍼블리셔 초기화
  status_node_ = std::make_shared<rclcpp::Node>("mark7_gripper_status_publisher");
  gripper_status_pub_ =
    status_node_->create_publisher<pipet_hand_mark7_msgs::msg::GripperStatus>(
      "/gripper/status", rclcpp::QoS(10).reliable());

  RCLCPP_INFO(
    rclcpp::get_logger("Mark7SystemHardware"),
    "on_init: port=%s, mock=%s, joints=%zu",
    port_.c_str(), use_mock_hardware_ ? "true" : "false", n);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Mark7SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;

    state_interfaces.emplace_back(
      joint_name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    state_interfaces.emplace_back(
      joint_name, "current", &hw_states_current_[i]);
    state_interfaces.emplace_back(
      joint_name, "temperature", &hw_states_temperature_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Mark7SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Mark7SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("Mark7SystemHardware"), "Mock 모드로 활성화");
    return CallbackReturn::SUCCESS;
  }

  // 시리얼 포트 열기
  if (!serial_.open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Mark7SystemHardware"),
      "시리얼 포트 열기 실패: %s", port_.c_str());
    return CallbackReturn::ERROR;
  }

  // Position Counter 리셋 명령 3회 전송 (RF 손실 대비)
  Mark7TxPacket reset_pkt;
  reset_pkt.finger_mask = 0x3F;  // 전체 손가락
  reset_pkt.direction   = Direction::RESET;
  const auto reset_buf = reset_pkt.encode();
  for (int i = 0; i < 3; ++i) {
    serial_.write(reset_buf.data(), reset_buf.size());
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Mark7SystemHardware"),
    "시리얼 포트 열기 완료, Reset 송신: %s", port_.c_str());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Mark7SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!use_mock_hardware_) {
    serial_.close();
    RCLCPP_INFO(rclcpp::get_logger("Mark7SystemHardware"), "시리얼 포트 닫음");
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Mark7SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_mock_hardware_) {
    // 이상적 모터 가정: command를 state로 복사
    for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
      hw_states_position_[i] = hw_commands_[i];
    }
    publish_gripper_status();
    return hardware_interface::return_type::OK;
  }

  const std::string raw_line = serial_.read_crlf_line();
  if (raw_line.empty()) {
    // 타임아웃: 이전 상태 유지
    return hardware_interface::return_type::OK;
  }

  // 로그 출력 시 줄바꿈 문자로 출력이 깨지지 않도록 CR/LF 제거
  std::string line = raw_line;
  while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
    line.pop_back();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Mark7SystemHardware"),
    "Rx raw: [%s]", line.c_str());

  Mark7RxPacket pkt;
  if (!Mark7RxPacket::parse(line, pkt)) {
    RCLCPP_WARN(
      rclcpp::get_logger("Mark7SystemHardware"),
      "RxPacket 파싱 실패: [%s]", line.c_str());
    return hardware_interface::return_type::OK;
  }

  const std::size_t n = std::min(hw_states_position_.size(),
    static_cast<std::size_t>(Mark7RxPacket::NUM_FINGERS));
  for (std::size_t i = 0; i < n; ++i) {
    hw_states_position_[i]    = static_cast<double>(pkt.fingers[i].position);
    hw_states_current_[i]     = static_cast<double>(pkt.fingers[i].current);
    hw_states_temperature_[i] = static_cast<double>(pkt.fingers[i].temperature);
  }

  publish_gripper_status();

  RCLCPP_INFO(
    rclcpp::get_logger("Mark7SystemHardware"),
    "Rx: target=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f] pos=[%d,%d,%d,%d,%d,%d]",
    hw_commands_[0], hw_commands_[1], hw_commands_[2],
    hw_commands_[3], hw_commands_[4], hw_commands_[5],
    pkt.fingers[0].position, pkt.fingers[1].position, pkt.fingers[2].position,
    pkt.fingers[3].position, pkt.fingers[4].position, pkt.fingers[5].position);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Mark7SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("Mark7SystemHardware"), steady_clock_, 1000,
      "Mock write: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
      hw_commands_[0], hw_commands_[1], hw_commands_[2],
      hw_commands_[3], hw_commands_[4], hw_commands_[5]);
    return hardware_interface::return_type::OK;
  }

  // command가 변경된 경우에만 전송
  if (hw_commands_ == hw_commands_sent_) {
    return hardware_interface::return_type::OK;
  }
  hw_commands_sent_ = hw_commands_;

  Mark7TxPacket pkt;
  pkt.finger_mask = 0x3F;   // 전체 6개 손가락
  pkt.speed       = 50;     // 실제 RPM = 50×200 = 10000 RPM
  pkt.current     = 250;    // 실제 mA = 600+250×3 = 1350 mA
  pkt.direction   = Direction::FORWARD;
  for (std::size_t i = 0; i < hw_commands_.size() && i < 6; ++i) {
    pkt.position[i] = Mark7TxPacket::steps_to_raw(
      static_cast<int>(hw_commands_[i]));
  }

  const auto buf = pkt.encode();
  serial_.write(buf.data(), buf.size());
  serial_.write(buf.data(), buf.size());

  RCLCPP_INFO(
    rclcpp::get_logger("Mark7SystemHardware"),
    "Tx: [%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]",
    hw_commands_[0], hw_commands_[1], hw_commands_[2],
    hw_commands_[3], hw_commands_[4], hw_commands_[5]);

  return hardware_interface::return_type::OK;
}

void Mark7SystemHardware::publish_gripper_status()
{
  pipet_hand_mark7_msgs::msg::GripperStatus msg;
  msg.header.stamp = status_node_->now();

  const std::size_t n = std::min(hw_states_position_.size(), static_cast<std::size_t>(6));
  for (std::size_t i = 0; i < n; ++i) {
    msg.fingers[i].position    = static_cast<float>(hw_states_position_[i]);
    msg.fingers[i].current     = static_cast<float>(hw_states_current_[i]);
    msg.fingers[i].temperature = static_cast<float>(hw_states_temperature_[i]);
  }

  gripper_status_pub_->publish(msg);
}

}  // namespace pipet_hand_mark7_driver

PLUGINLIB_EXPORT_CLASS(
  pipet_hand_mark7_driver::Mark7SystemHardware,
  hardware_interface::SystemInterface)
