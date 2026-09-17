// Copyright 2025 Enactic, Inc.
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

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/dynamics.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace openarm_hardware {

/**
 * @brief Simplified OpenArm V10 Hardware Interface
 *
 * This is a simplified version that uses the OpenArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class OpenArmHW : public hardware_interface::SystemInterface {
 public:
  OpenArmHW();
  ~OpenArmHW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // V10 default configuration
  static constexpr size_t ARM_DOF = 7;
  static constexpr bool ENABLE_GRIPPER = true;

  // Default motor configuration for V10
  const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM8009,  // Joint 1
      openarm::damiao_motor::MotorType::DM8009,  // Joint 2
      openarm::damiao_motor::MotorType::DM4340,  // Joint 3
      openarm::damiao_motor::MotorType::DM4340,  // Joint 4
      openarm::damiao_motor::MotorType::DM4310,  // Joint 5
      openarm::damiao_motor::MotorType::DM4310,  // Joint 6
      openarm::damiao_motor::MotorType::DM4310   // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15, 0x16, 0x17};

  const openarm::damiao_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

  // Gains. These are the nominal set the description passes in; the modes
  // below are derived from them at startup.
  std::vector<double> kp_ = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
  std::vector<double> kd_ = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};

  // --- Gain modes ------------------------------------------------------
  // Three ways to drive the same arm, switched by service while it runs.
  //
  //   DEFAULT     the nominal gains: stiff enough to track a trajectory
  //   IMPEDANCE   a quarter of the stiffness, so the arm yields to a hand
  //               and the feed-forward carries its weight
  //   ZERO_G      no stiffness at all, only damping: hand guiding
  //
  // The two soft modes need the gravity feed-forward, because what the PD
  // term stops holding the model has to hold instead.
  enum GainMode : size_t {
    MODE_DEFAULT = 0,
    MODE_IMPEDANCE = 1,
    MODE_ZERO_G = 2,
    MODE_COUNT = 3,
  };
  // Scales on the nominal gains, from the operator console's soft recipe.
  static constexpr double IMPEDANCE_KP_SCALE = 0.25;
  static constexpr double IMPEDANCE_KD_SCALE = 0.6;
  // Damping that stays once kp is zero. Shoulder and elbow need more than
  // the wrist to stop the arm swinging when nothing holds it.
  static constexpr std::array<double, ARM_DOF> ZERO_G_KD = {1.0, 1.0, 0.8, 0.8,
                                                            0.2, 0.2, 0.2};
  std::array<std::vector<double>, MODE_COUNT> kp_modes_;
  std::array<std::vector<double>, MODE_COUNT> kd_modes_;
  // Read by write() every cycle and written by a service callback on another
  // thread, so the switch is one atomic store and never a half-applied set.
  std::atomic<size_t> mode_{MODE_DEFAULT};

  // Re-synchronisation after a soft mode. While kp was low the arm was moved
  // by hand, and the controller's command stayed where it was, so restoring
  // stiffness would snap the arm back to it. Instead the commanded position
  // restarts at the measured pose and walks to the controller's command at a
  // speed a person can step away from.
  static constexpr double RESYNC_SPEED = 0.4;      // rad/s
  static constexpr double DEFAULT_PERIOD = 1.0 / 750.0;  // s, if write() gets 0
  std::atomic<bool> resync_pending_{false};
  bool resync_active_ = false;
  std::vector<double> pos_targets_ = std::vector<double>(ARM_DOF, 0.0);

  // The services live on a node of our own: a hardware component is not one,
  // and the controller manager's node must not be blocked by our callbacks.
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::array<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr, MODE_COUNT>
      mode_services_;

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_KP = 5.0;
  const double GRIPPER_KD = 0.1;

  double gripper_kp_ = GRIPPER_KP;
  double gripper_kd_ = GRIPPER_KD;

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
  std::string ee_type_;
  bool hand_;
  bool can_fd_;

  // Gravity feed-forward (#77). Every value here is measured per robot, so the
  // description passes it in; the defaults below only mean "not calibrated".
  bool gravity_comp_ = false;
  std::string root_link_;
  std::string tip_link_;
  double payload_mass_ = 0.0;
  std::array<double, 3> payload_com_ = {0.0, 0.0, 0.0};
  // Per-joint constant torque offset (Nm), clamped to +-TAU_BIAS_LIMIT.
  std::vector<double> tau_bias_ = std::vector<double>(ARM_DOF, 0.0);
  // Absolute per-joint cap on the feed-forward term (Nm). Half the joint's
  // rated effort (40/40/27/27/7/7/7 Nm), so a bad model cannot spend more than
  // half the motor and the PD term keeps its headroom.
  std::vector<double> saturation_cap_ = {20.0, 20.0, 13.5, 13.5, 3.5, 3.5, 3.5};
  static constexpr double TAU_BIAS_LIMIT = 1.5;
  std::unique_ptr<Dynamics> dynamics_;
  // write() scratch, sized here so the RT path allocates nothing.
  std::vector<double> feedforward_ = std::vector<double>(ARM_DOF, 0.0);

  // OpenArm instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  static constexpr std::array<double, ARM_DOF> ZERO_POSITION = {
      0.0,  // joint1
      0.0,  // joint2
      0.0,  // joint3
      0.0,  // joint4
      0.0,  // joint5
      0.0,  // joint6
      0.0,  // joint7
  };

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();
  // Derives the mode tables from the nominal gains. Called once, so write()
  // only ever indexes them.
  void build_gain_modes();
  // Brings up the node the mode services sit on and spins it.
  void start_mode_services();
  void stop_mode_services();
  // Service body: switches the mode, or explains why it did not.
  void switch_mode(size_t mode,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  // Walks pos_targets_ to the controller's command after a soft mode.
  void update_targets(const rclcpp::Duration& period);
  // Fills feedforward_ with the capped model torque plus the measured offset
  // for the current pose, or zeros when compensation is off. Allocation-free.
  void update_feedforward();

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}  // namespace openarm_hardware
