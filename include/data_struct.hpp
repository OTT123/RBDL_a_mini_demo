#include <Eigen/Dense>

namespace robot_state {
struct Joint_State {
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;
  Eigen::VectorXd tau;
};
} // namespace robot_state
