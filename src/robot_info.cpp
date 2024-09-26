#include "robot_info.hpp"

RobotModel::RobotModel(const char *filename, bool floating_base, bool verbose = false) {
  RigidBodyDynamics::Addons::URDFReadFromFile(filename, robot_.get(), floating_base, verbose);
  M_.setZero(robot_->qdot_size, robot_->qdot_size);
  G_.setZero(robot_->qdot_size);
  D_.setZero(robot_->qdot_size);
  C_.setZero(robot_->qdot_size);
  zeros_.setZero(robot_->qdot_size);
  RigidBodyDynamics::Math::Vector3d gravity;
  gravity << 0, 0, -9.81;
  robot_->gravity = gravity;
}

RobotModel::RobotModel(const char *filename, bool floating_base, bool verbose,
                       RigidBodyDynamics::Math::Vector3d gravity) {
  RigidBodyDynamics::Addons::URDFReadFromFile(filename, robot_.get(), floating_base, verbose);
  M_.setZero(robot_->qdot_size, robot_->qdot_size);
  G_.setZero(robot_->qdot_size);
  D_.setZero(robot_->qdot_size);
  C_.setZero(robot_->qdot_size);
  zeros_.setZero(robot_->qdot_size);
  robot_->gravity = gravity;
}

void RobotModel::UpdateRobotState(robot_state::Joint_State joint_state) {
  q_ = joint_state.q;
  qd_ = joint_state.qd;
  qdd_ = joint_state.qdd;
  tau_ = joint_state.tau;
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(*robot_, q_, M_, false);

  Eigen::VectorXd ZeroQdot;
  ZeroQdot.setZero(robot_->qdot_size);
  // Gravity
  Eigen::VectorXd grav_tmp;
  grav_tmp.setZero(robot_->qdot_size);
  RigidBodyDynamics::InverseDynamics(*robot_, q_, ZeroQdot, ZeroQdot, grav_tmp);
  G_ = grav_tmp;
  // Coriolis
  Eigen::VectorXd coriolis_tmp;
  coriolis_tmp.setZero(robot_->qdot_size);
  RigidBodyDynamics::InverseDynamics(*robot_, q_, qd_, ZeroQdot, coriolis_tmp);
  C_ = coriolis_tmp;
}

void RobotModel::PrintInfo() {
  std::cout << "get Mass Mat = " << std::endl;
  std::cout << "[ " << M_ << " ]" << std::endl;
  std::cout << "gravity = [ " << robot_->gravity.transpose() << " ] , "
            << "get G Mat" << std::endl;
  std::cout << "[ " << G_.transpose() << " ]" << std::endl;
  // std::cout << "get D = G + Cqdot Mat" << std::endl;
  // std::cout << "[ " << D_.transpose() << " ]" << std::endl;
  std::cout << "get Cqdot Mat = " << std::endl;
  std::cout << "[ " << C_.transpose() << " ]" << std::endl;
}
