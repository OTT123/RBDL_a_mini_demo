#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <iostream>

int main(int argc, char const *argv[])
{
  namespace rbdl = RigidBodyDynamics;
  // robot 对象
  std::unique_ptr<RigidBodyDynamics::Model> robot_ = std::make_unique<rbdl::Model>();
  rbdl::Addons::URDFReadFromFile("./urdf/ur5.urdf", robot_.get(), false, true);
  std::cout << "Model Hierarchy:" << std::endl;
  std::cout << rbdl::Utils::GetModelHierarchy(*robot_) << std::endl;
  std::cout << "Degree of freedoms overview: " << robot_->dof_count << std::endl;
  std::cout << "Q Dofs: " << robot_->q_size << ", QD Dofs: " << robot_->qdot_size << std::endl;
  std::cout << rbdl::Utils::GetModelDOFOverview(*robot_) << std::endl;
  std::cout << "Rbdl dofs: " << robot_->dof_count << std::endl;

  RigidBodyDynamics::Math::Vector3d gravity;
  gravity << 0, 0, -9.81;
  robot_->gravity = gravity;

  int kNumDofs = 22;

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;
  Eigen::VectorXd tau;
  Eigen::VectorXd zeros;
  q.setZero(robot_->q_size);
  qd.setZero(robot_->qdot_size);
  qdd.setZero(robot_->qdot_size);
  tau.setZero(robot_->qdot_size);
  zeros.setZero(robot_->qdot_size);

  Eigen::MatrixXd M;
  Eigen::VectorXd G;
  Eigen::VectorXd D;
  Eigen::VectorXd C;
  M.setZero(robot_->q_size, robot_->q_size);
  G.setZero(robot_->q_size);
  D.setZero(robot_->q_size);
  C.setZero(robot_->q_size);
  q << 1, 0, 1, 0, 1, 0;
  qd << 0, 1, 0, 1, 0, 1;

  rbdl::CompositeRigidBodyAlgorithm(*robot_, q, M, false);
  std::cout << "get Mass Mat = " << std::endl;
  std::cout << "[ " << M << " ]" << std::endl;

  rbdl::NonlinearEffects(*robot_, q, zeros, G);
  std::cout << "gravity = [ " << robot_->gravity.transpose() << " ] , " << "get G Mat" << std::endl;
  std::cout << "[ " << G.transpose() << " ]" << std::endl;

  rbdl::NonlinearEffects(*robot_, q, qd, D);
  std::cout << "get D = G + Cqdot Mat" << std::endl;
  std::cout << "[ " << D.transpose() << " ]" << std::endl;

  C = D - G;
  std::cout << "get Cqdot Mat = " << std::endl;
  std::cout << "[ " << C.transpose() << " ]" << std::endl;

  Eigen::Vector3d pos;
  pos << 0, 0, 0;
  std::cout << "test Jdot*Qdot" << std::endl;
  std::cout << "[ " << rbdl::CalcPointAcceleration(*robot_, q, qd, zeros, 6, pos).transpose() << " ]" << std::endl;

  return 0;
}
