#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pipet_hand_mark7_driver/serial_port.hpp"
#include "pipet_hand_mark7_driver/mark7_tx_packet.hpp"
#include "pipet_hand_mark7_driver/mark7_rx_packet.hpp"
#include "pipet_hand_mark7_msgs/msg/gripper_status.hpp"

namespace pipet_hand_mark7_driver
{

class Mark7SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Mark7SystemHardware)

  /// URDF ros2_control 블록에서 파라미터 읽기, 인터페이스 벡터 초기화
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /// State 인터페이스 등록 (position, current, temperature × 6관절)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// Command 인터페이스 등록 (position × 6관절)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// 시리얼 포트 열기, Reset 명령 전송 (stub → Task 2-2에서 구현)
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// 시리얼 포트 닫기 (stub → Task 2-2에서 구현)
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Rx ASCII 라인 수신 → hw_states_ 업데이트 (stub → Task 2-3에서 구현)
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// hw_commands_ → Tx 11바이트 패킷 전송
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void publish_gripper_status();

  SerialPort serial_;

  std::vector<double> hw_commands_;           ///< position commands [num_joints]
  std::vector<double> hw_commands_sent_;      ///< 마지막으로 전송한 commands (변경 감지용)
  std::vector<double> hw_states_position_;    ///< [num_joints]
  std::vector<double> hw_states_current_;     ///< [num_joints]
  std::vector<double> hw_states_temperature_; ///< [num_joints]

  std::string port_;
  bool use_mock_hardware_{false};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  /// 오차가 이 값(steps)을 초과할 때만 Tx 패킷 재전송
  static constexpr double POSITION_ERROR_THRESHOLD = 10.0;

  /// /gripper/status 퍼블리셔 (hardware interface 내부 전용 노드)
  rclcpp::Node::SharedPtr status_node_;
  rclcpp::Publisher<pipet_hand_mark7_msgs::msg::GripperStatus>::SharedPtr gripper_status_pub_;
};

}  // namespace pipet_hand_mark7_driver
