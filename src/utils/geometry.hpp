#pragma once

#include <Eigen/Core>
#include <vector>

namespace plan_env {
namespace detail {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

double wrapAngle(double angle);
bool pointInPolygon(const Eigen::Vector2d &point,
                    const std::vector<Eigen::Vector2f> &polygon);
double distanceToSegment(const Eigen::Vector2d &point, const Eigen::Vector2d &a,
                         const Eigen::Vector2d &b, Eigen::Vector2d *closest);
bool nearAny(const Eigen::Vector2d &point,
             const std::vector<Eigen::Vector2d> &points, double radius);
std::vector<Eigen::Vector2d> cleanPolygon(
    const std::vector<Eigen::Vector2d> &polygon);

}  // namespace detail
}  // namespace plan_env
