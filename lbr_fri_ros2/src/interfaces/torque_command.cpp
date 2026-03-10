#include "lbr_fri_ros2/interfaces/torque_command.hpp"

namespace lbr_fri_ros2 {
TorqueCommandInterface::TorqueCommandInterface(
    const double &joint_position_tau, const CommandGuardParameters &command_guard_parameters,
    const std::string &command_guard_variant)
    : BaseCommandInterface(joint_position_tau, command_guard_parameters, command_guard_variant), accum_(0.0) {}

void TorqueCommandInterface::buffered_command_to_fri(fri_command_t_ref command,
                                                     const_idl_state_t_ref state) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (state.client_command_mode != KUKA::FRI::EClientCommandMode::TORQUE) {
    std::string err =
        "Client side (configured via client_command_mode in lbr_system_config.yaml) "
        "expected robot in '" +
        EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::TORQUE) +
        "' command mode, but robot was in '" +
        EnumMaps::client_command_mode_map(state.client_command_mode) +
        "' command mode. Correct the configurations or run the robot in '" +
        EnumMaps::client_command_mode_map(KUKA::FRI::EClientCommandMode::TORQUE) +
        "' command mode.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  if (!joint_position_filter_.is_initialized()) {
    joint_position_filter_.initialize(state.sample_time);
  }

  if (!command_initialized_) {
    std::string err = "Uninitialized command.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  if (!std::any_of(command_target_.joint_position.cbegin(), command_target_.joint_position.cend(),
                   [](const double &v) { return std::isnan(v); }) &&
      !std::any_of(command_target_.torque.cbegin(), command_target_.torque.cend(),
                   [](const double &v) { return std::isnan(v); })) {
    // write command_target_ to command_ (with exponential smooth on joint positions), else use
    // internal command_
    joint_position_filter_.compute(command_target_.joint_position, command_.joint_position);
    command_.torque = command_target_.torque;
  }

  if (!command_guard_) {
    std::string err = "Uninitialized command guard.";
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
    throw std::runtime_error(err);
  }

  // write the measured joint position to joint position
  command_.joint_position = state.measured_joint_position;

  // Implements dithering to trigger the friction observer.
  // If the physical robot is at the commanded positions, KUKA turns off friction
  // compensation.
  accum_ += 0.01; // known issue: not sample frequency aware
  accum_ = fmod(accum_, M_PI*2.0); // keeps the value small

  double delta = 0.01*std::sin(accum_); // original implementation is 0.1

  for(int i = 0; i < 7; i++) {
    command_.joint_position[i] += delta;
    //RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()), ColorScheme::OKCYAN << "joint " << i << " = " << command_.joint_position[i] << ColorScheme::ENDC);
  }

  // validate
  if (!command_guard_->is_valid_command(command_, state)) {
    std::string warn = "Overriding invalid command to neutral command.";
    RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                       ColorScheme::WARNING << warn.c_str() << ColorScheme::ENDC);
    neutralize_command_(state, command_);
  }

  // write joint position and torque to output
  command.setJointPosition(command_.joint_position.data());
  command.setTorque(command_.torque.data());
}
} // namespace lbr_fri_ros2
