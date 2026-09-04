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

#include <urdf/model.h>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <string>

namespace openarm_hardware {

/**
 * @brief KDL inverse-dynamics wrapper for gravity feed-forward.
 *
 * The MIT command is tau = kp*(q* - q) + kd*(qd* - qd) + tau_ff. With tau_ff
 * left at zero the position gains alone have to carry the arm's weight, so the
 * arm sags between trajectory points (openarm_ros2 issue #77). GetGravity
 * returns tau_g = G(q); adding it to tau_ff makes the motors hold the weight
 * and leaves the PD term to correct tracking error only.
 */
class Dynamics {
 public:
  // urdf_xml: the URDF XML string (ros2_control hands the plugin the exact
  // document the robot was built from as info.original_xml).
  Dynamics(const std::string& urdf_xml, const std::string& root_link,
           const std::string& tip_link);

  // Append a point mass rigidly attached to the tip segment (CoM offset in the
  // tip-link frame, meters). Call BEFORE Init(). Models the mass the root->tip
  // chain misses: KDL stops at the tip link, so the gripper body and its
  // prismatic finger children are outside the chain. Without it the arm sags
  // at large-moment poses even with compensation on.
  void SetPayload(double mass_kg, double x, double y, double z);

  // Parse URDF -> KDL tree -> root->tip chain (+ payload) -> solver.
  // False on parse failure, unknown root/tip link, or a solver that rejects
  // the chain. Allocates: call from on_init, never from write().
  bool Init();

  // Per-joint gravity-holding torque (Nm) at joint position q. Array length =
  // chain joint count (7 for one OpenArm arm).
  // RT-safe: allocation-free — the query arrays are members sized by Init().
  void GetGravity(const double* joint_position, double* gravity);

  // Joints in the solver's chain. Valid only after a successful Init().
  // (The payload segment is Joint::None, so a payload never changes it.)
  size_t joint_count() const { return kdl_chain_.getNrOfJoints(); }

 private:
  std::string urdf_xml_;
  std::string root_link_;
  std::string tip_link_;
  double payload_mass_ = 0.0;
  double payload_com_[3] = {0.0, 0.0, 0.0};

  KDL::Chain kdl_chain_;
  KDL::JntArray gravity_forces_;
  // Preallocated query input, reused by every GetGravity call so the RT path
  // never hits the heap.
  KDL::JntArray q_query_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
};

}  // namespace openarm_hardware
