#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace plan_env {

namespace registration {

struct Config {
  bool enabled = true;
  float max_correspondence_distance = 0.50f;
  int maximum_iterations = 40;
  float transformation_epsilon = 1e-5f;
  float euclidean_fitness_epsilon = 1e-3f;
  float max_fitness = 0.25f;
  float min_overlap = 0.20f;
  float max_translation_correction = 1.0f;
  float max_rotation_correction_rad = 0.50f;
  std::size_t min_target_points = 50;
};

struct Result {
  bool accepted = false;
  bool converged = false;
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  float fitness = 0.0f;
  float overlap = 0.0f;
  float translation_correction = 0.0f;
  float rotation_correction_rad = 0.0f;
};

class PointCloudRegistrationBackend {
public:
  PointCloudRegistrationBackend() = default;
  explicit PointCloudRegistrationBackend(const Config &config);

  bool configure(const Config &config);
  const Config &config() const;

  // Align source points in their sensor frame to a map-frame target. The
  // supplied pose is the navigation/TF prior and is used as ICP's initial
  // guess. No map state is retained by this class.
  Result align(const std::vector<Eigen::Vector3f> &source,
               const std::vector<Eigen::Vector3f> &target,
               const Eigen::Isometry3f &initial_pose) const;

private:
  Config config_;
};

}  // namespace registration

}  // namespace plan_env
