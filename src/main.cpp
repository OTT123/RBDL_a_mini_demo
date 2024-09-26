#include "robot_info.hpp"
void convert(double yaw, double pitch, double roll, Eigen::Quaternion<double> &to);
int main(int argc, char const *argv[]) {
  namespace rbdl = RigidBodyDynamics;
  // // robot 对象
  // std::unique_ptr<RigidBodyDynamics::Model> robot_ = std::make_unique<rbdl::Model>();
  // rbdl::Addons::URDFReadFromFile("/home/xuzhilin2004/RBDL_a_mini_demo/urdf/ur5_robot.urdf", robot_.get(), false,
  // false); std::cout << "Model Hierarchy:" << std::endl; std::cout << rbdl::Utils::GetModelHierarchy(*robot_) <<
  // std::endl; std::cout << "Degree of freedoms overview: " << robot_->dof_count << std::endl; std::cout << "Q Dofs: "
  // << robot_->q_size << ", QD Dofs: " << robot_->qdot_size << std::endl; std::cout <<
  // rbdl::Utils::GetModelDOFOverview(*robot_) << std::endl; std::cout << "Rbdl dofs: " << robot_->dof_count <<
  // std::endl;

  // RigidBodyDynamics::Math::Vector3d gravity;
  // gravity << 0, 0, -9.81;
  // robot_->gravity = gravity;

  // const int kNumDofs = 6;

  // Eigen::VectorXd q;
  // Eigen::VectorXd qd;
  // Eigen::VectorXd qdd;
  // Eigen::VectorXd tau;
  // Eigen::VectorXd zeros;
  // q.setZero(robot_->q_size);
  // qd.setZero(robot_->qdot_size);
  // qdd.setZero(robot_->qdot_size);
  // tau.setZero(robot_->qdot_size);
  // zeros.setZero(robot_->qdot_size);

  // Eigen::MatrixXd M;
  // Eigen::VectorXd G;
  // Eigen::VectorXd D;
  // Eigen::VectorXd C;
  // M.setZero(robot_->q_size, robot_->q_size);
  // G.setZero(robot_->q_size);
  // D.setZero(robot_->q_size);
  // C.setZero(robot_->q_size);
  // q << 1, 0, 1, 0, 1, 0;
  // qd << 0, 1, 0, 1, 0, 1;

  // // tau = Mqddot + Cqdot + G
  // rbdl::CompositeRigidBodyAlgorithm(*robot_, q, M, false);
  // std::cout << "get Mass Mat = " << std::endl;
  // std::cout << "[ " << M << " ]" << std::endl;

  // rbdl::NonlinearEffects(*robot_, q, zeros, G);
  // std::cout << "gravity = [ " << robot_->gravity.transpose() << " ] , "
  //           << "get G Mat" << std::endl;
  // std::cout << "[ " << G.transpose() << " ]" << std::endl;

  // rbdl::NonlinearEffects(*robot_, q, qd, D);
  // std::cout << "get D = G + Cqdot Mat" << std::endl;
  // std::cout << "[ " << D.transpose() << " ]" << std::endl;

  // C = D - G;
  // std::cout << "get Cqdot Mat = " << std::endl;
  // std::cout << "[ " << C.transpose() << " ]" << std::endl;

  // Eigen::Vector3d zeros_local_pos;
  // zeros_local_pos << 0.01, 0.01, 0.01;
  // std::cout << "test Jdot*Qdot" << std::endl;
  // std::cout << "[ "
  //           << rbdl::CalcPointAcceleration(*robot_, q, qd, zeros, robot_->GetBodyId("wrist_3_link"), zeros_local_pos)
  //                  .transpose()
  //           << " ]" << std::endl;

  // Eigen::MatrixXd point_jac;
  // point_jac.setZero(3, kNumDofs);
  // rbdl::CalcPointJacobian(*robot_, q, robot_->GetBodyId("wrist_3_link"), zeros_local_pos, point_jac, false);
  // std::cout << "test point Jac" << std::endl;
  // std::cout << "[ " << point_jac << " ]" << std::endl;

  // // Jacobian
  // Eigen::MatrixXd point_jac6D;
  // point_jac6D.setZero(6, kNumDofs);
  // rbdl::CalcPointJacobian6D(*robot_, q, robot_->GetBodyId("wrist_3_link"), zeros_local_pos, point_jac6D, false);
  // std::cout << "test point Jac6d" << std::endl;
  // std::cout << "[ " << point_jac6D << " ]" << std::endl;

  const char *filename = "/home/xuzhilin2004/RBDL_a_mini_demo/urdf/ur5_robot.urdf";
  std::unique_ptr<RobotModel> static_robot = std::make_unique<RobotModel>(filename, false, false);
  std::unique_ptr<RobotModel> floating_robot = std::make_unique<RobotModel>(filename, true, false);

  Eigen::VectorXd q;
  q.setZero(static_robot->robot_->q_size);
  Eigen::VectorXd qd;
  qd.setZero(static_robot->robot_->q_size);
  Eigen::VectorXd qdd;
  qdd.setZero(static_robot->robot_->q_size);
  Eigen::VectorXd base_zeros;
  base_zeros.setZero(6);
  q << 0, 0, 0, 0, 0, 0;
  qd << 0, 0, 0, 0, 0, 0;
  base_zeros << 0, 0, 0, 0, 0, 0;

  robot_state::Joint_State static_joint;
  static_joint.q = q;
  static_joint.qd = qd;

  robot_state::Joint_State floating_joint;
  floating_joint.q.setZero(floating_robot->robot_->q_size);
  floating_joint.qd.setZero(floating_robot->robot_->qdot_size);
  Eigen::Vector3d RPY;
  RPY << 0.4, 0.3, 0.2;
  Eigen::Quaternion<double> qua;
  convert(RPY[2], RPY[1], RPY[0], qua);
  floating_joint.q << base_zeros, q, 0;
  floating_joint.qd << base_zeros, qd;
  floating_joint.q.segment(3, 3) << qua.x(), qua.y(), qua.z();
  floating_joint.q.tail(1) << qua.w();
  
  static_robot->UpdateRobotState(static_joint);
  static_robot->PrintInfo();
  std::cout << "***************" << std::endl;
  floating_robot->UpdateRobotState(floating_joint);
  floating_robot->PrintInfo();

  return 0;
}

void convert(double yaw, double pitch, double roll, Eigen::Quaternion<double> &to) {
  // double c1 = cos(yaw);
  // double s1 = sin(yaw);
  // double c2 = cos(pitch);
  // double s2 = sin(pitch);
  // double c3 = cos(roll);
  // double s3 = sin(roll);
  // double w = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
  // double w4 = (4.0 * w);
  // double x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4 ;
  // double y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4 ;
  // double z = (-s1 * s3 + c1 * s2 * c3 +s2) / w4 ;
  // to = dynacore::Quaternion(w, x, y, z);
  Eigen::Quaternion<double> Qyaw(cos(0.5 * yaw), 0, 0, sin(0.5 * yaw));
  Eigen::Quaternion<double> Qpitch(cos(0.5 * pitch), 0, sin(0.5 * pitch), 0);
  Eigen::Quaternion<double> Qroll(cos(0.5 * roll), sin(0.5 * roll), 0, 0);

  to = Qyaw * Qpitch * Qroll;
}