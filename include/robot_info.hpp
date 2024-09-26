#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#include <Eigen/Dense>
#include <iostream>

#include "data_struct.hpp"

class RobotModel {
 public:
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd qdd_;
  Eigen::VectorXd tau_;
  Eigen::VectorXd zeros_;
  Eigen::MatrixXd M_;
  Eigen::VectorXd G_;
  Eigen::VectorXd D_;
  Eigen::VectorXd C_;
  std::unique_ptr<RigidBodyDynamics::Model> robot_ = std::make_unique<RigidBodyDynamics::Model>();

  RobotModel(const char* filename, bool floating_base, bool verbose);
  RobotModel(const char* filename, bool floating_base, bool verbose, RigidBodyDynamics::Math::Vector3d gravity);
  void UpdateRobotState(robot_state::Joint_State joint_state);
  void PrintInfo();
};