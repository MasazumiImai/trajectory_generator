#include <traj_gen/traj_gen.hpp>

int main()
{
  return traj_gen::expMap(Eigen::Vector3d::Zero()).isApprox(Eigen::Quaterniond::Identity()) ? 0 : 1;
}
