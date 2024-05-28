// Copyright (c) 2024, Jakub Delicat
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "microros_hardware_interfaces/microros_hardware_interfaces.hpp"

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace microros_hardware_interfaces
{
hardware_interface::CallbackReturn MicroROSHArdwareInterfaces::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MicroROSHArdwareInterfaces::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  encoders_cpr = hardware_interface::stod(info_.hardware_parameters.at("encoders_cpr"));
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MicroROSHArdwareInterfaces::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[2 * i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[2 * i + 1]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MicroROSHArdwareInterfaces::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
MicroROSHArdwareInterfaces::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  node_ = rclcpp::Node::make_shared("microros_hardware_interfaces_node");

  wheel_velocity_command_pub_ =
      node_->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_velocity_command", rclcpp::QoS(1));
  wheel_position_state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "wheel_position_state", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float32MultiArray wheel_position_state) {
        latest_wheel_position_state_.data.clear();
        for (auto const& state : wheel_position_state.data)
        {
          latest_wheel_position_state_.data.push_back(state);
        }
      });

  wheel_velocity_state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "wheel_velocity_state", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float32MultiArray wheel_velocity_state) {
        latest_wheel_velocity_state_.data.clear();
        for (auto const& state : wheel_velocity_state.data)
        {
          latest_wheel_velocity_state_.data.push_back(state);
        }
      });

  while (!wheel_velocity_command_pub_->get_subscription_count())
  {
    RCLCPP_WARN(rclcpp::get_logger("microros_hardware_interface"),
                "There is no microros subscription for wheel command.");

    std::chrono::seconds duration_in_seconds(1);
    std::chrono::nanoseconds duration_in_nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration_in_seconds);

    rclcpp::sleep_for(duration_in_nanoseconds);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MicroROSHArdwareInterfaces::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  wheel_velocity_command_pub_.reset();
  wheel_position_state_sub_.reset();
  wheel_velocity_state_sub_.reset();

  node_.reset();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MicroROSHArdwareInterfaces::read(const rclcpp::Time& /*time*/,
                                                                 const rclcpp::Duration& /*period*/)
{
  if (rclcpp::ok() and node_ != nullptr)
  {
    rclcpp::spin_some(node_);
  }

  if (latest_wheel_position_state_.data.size())
  {
    double left_rotations_rad = latest_wheel_position_state_.data[0] / encoders_cpr;
    double right_rotations_rad = latest_wheel_position_state_.data[1] / encoders_cpr;
    hw_states_[0] = left_rotations_rad;
    hw_states_[2] = right_rotations_rad;
  }
  if (latest_wheel_velocity_state_.data.size())
  {
    double left_rotations_rad_per_sec = latest_wheel_velocity_state_.data[0] / encoders_cpr;
    double right_rotations_rad_per_sec = latest_wheel_velocity_state_.data[1] / encoders_cpr;
    hw_states_[1] = left_rotations_rad_per_sec;
    hw_states_[3] = right_rotations_rad_per_sec;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MicroROSHArdwareInterfaces::write(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/)
{
  std_msgs::msg::Float32MultiArray command_msg;
  for (auto const& command : hw_commands_)
  {
    float tick_per_sec_command = command*encoders_cpr;
    command_msg.data.push_back(tick_per_sec_command);
  }

  if (rclcpp::ok() and node_ != nullptr)
  {
    wheel_velocity_command_pub_->publish(command_msg);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace microros_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(microros_hardware_interfaces::MicroROSHArdwareInterfaces, hardware_interface::SystemInterface)
