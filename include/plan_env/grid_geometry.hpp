#pragma once

#include <Eigen/Core>

namespace plan_env {

struct GridGeometry {
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  float resolution = 0.0f;
  int size_x = 0;
  int size_y = 0;
  int size_z = 0;
};

}  // namespace plan_env
