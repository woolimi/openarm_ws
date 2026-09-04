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

#include "openarm_hardware/dynamics.hpp"

#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

namespace {
constexpr double GRAVITY_ACCEL = 9.81;
}  // namespace

Dynamics::Dynamics(const std::string& urdf_xml, const std::string& root_link,
                   const std::string& tip_link)
    : urdf_xml_(urdf_xml), root_link_(root_link), tip_link_(tip_link) {}

void Dynamics::SetPayload(double mass_kg, double x, double y, double z) {
  payload_mass_ = mass_kg;
  payload_com_[0] = x;
  payload_com_[1] = y;
  payload_com_[2] = z;
}

bool Dynamics::Init() {
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_xml_)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Dynamics: failed to parse URDF");
    return false;
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Dynamics: failed to extract KDL tree from URDF");
    return false;
  }

  if (!kdl_tree.getChain(root_link_, tip_link_, kdl_chain_)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Dynamics: no KDL chain from '%s' to '%s'", root_link_.c_str(),
                 tip_link_.c_str());
    return false;
  }

  // Unmodeled end mass (see SetPayload): a rigid massless joint carrying a
  // point mass at the given tip-frame offset. The joint count is unchanged;
  // the solver simply carries the extra weight in G(q).
  if (payload_mass_ > 0.0) {
    kdl_chain_.addSegment(
        KDL::Segment("payload", KDL::Joint(KDL::Joint::None),
                     KDL::Frame(KDL::Vector(payload_com_[0], payload_com_[1],
                                            payload_com_[2])),
                     KDL::RigidBodyInertia(payload_mass_)));
  }

  const unsigned int nj = kdl_chain_.getNrOfJoints();
  gravity_forces_.resize(nj);
  gravity_forces_.data.setZero();
  // The query input lives as a member so the RT read path allocates nothing.
  q_query_.resize(nj);
  q_query_.data.setZero();

  // Gravity is expressed in the root link frame, so the root must be
  // world-aligned for -Z to be straight down.
  auto solver = std::make_unique<KDL::ChainDynParam>(
      kdl_chain_, KDL::Vector(0.0, 0.0, -GRAVITY_ACCEL));
  if (solver->JntToGravity(q_query_, gravity_forces_) !=
      KDL::SolverI::E_NOERROR) {
    // Never install a solver that cannot answer — a model returning garbage
    // is worse than no compensation at all.
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Dynamics: solver rejected the chain %s -> %s",
                 root_link_.c_str(), tip_link_.c_str());
    return false;
  }
  solver_ = std::move(solver);

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Dynamics chain %s -> %s ready (%u joints, payload %.3f kg at "
              "(%.3f, %.3f, %.3f))",
              root_link_.c_str(), tip_link_.c_str(), nj, payload_mass_,
              payload_com_[0], payload_com_[1], payload_com_[2]);
  return true;
}

void Dynamics::GetGravity(const double* joint_position, double* gravity) {
  const unsigned int nj = kdl_chain_.getNrOfJoints();
  for (unsigned int i = 0; i < nj; ++i) {
    q_query_(i) = joint_position[i];
  }
  solver_->JntToGravity(q_query_, gravity_forces_);
  for (unsigned int i = 0; i < nj; ++i) {
    gravity[i] = gravity_forces_(i);
  }
}

}  // namespace openarm_hardware
