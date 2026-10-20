#include "utils/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace plan_env {
namespace detail {

double wrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool pointInPolygon(const Eigen::Vector2d &point,
                    const std::vector<Eigen::Vector2f> &polygon) {
  if (polygon.size() < 3) {
    return false;
  }
  bool inside = false;
  std::size_t j = polygon.size() - 1;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const double xi = polygon[i].x();
    const double yi = polygon[i].y();
    const double xj = polygon[j].x();
    const double yj = polygon[j].y();
    const bool crosses = ((yi > point.y()) != (yj > point.y()));
    if (crosses) {
      const double x_at_y = (xj - xi) * (point.y() - yi) / (yj - yi) + xi;
      if (point.x() < x_at_y) {
        inside = !inside;
      }
    }
    j = i;
  }
  return inside;
}

double distanceToSegment(const Eigen::Vector2d &point, const Eigen::Vector2d &a,
                         const Eigen::Vector2d &b, Eigen::Vector2d *closest) {
  const Eigen::Vector2d d = b - a;
  const double denom = d.squaredNorm();
  const double t =
      denom > kEps ? std::max(0.0, std::min(1.0, (point - a).dot(d) / denom))
                   : 0.0;
  if (closest) {
    *closest = a + t * d;
  }
  return (point - (a + t * d)).norm();
}

bool nearAny(const Eigen::Vector2d &point,
             const std::vector<Eigen::Vector2d> &points, const double radius) {
  const double radius_sq = radius * radius;
  for (const auto &other : points) {
    if ((point - other).squaredNorm() <= radius_sq) {
      return true;
    }
  }
  return false;
}

std::vector<Eigen::Vector2d> cleanPolygon(
    const std::vector<Eigen::Vector2d> &polygon) {
  std::vector<Eigen::Vector2d> result;
  result.reserve(polygon.size());
  for (const auto &point : polygon) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
      continue;
    }
    if (!result.empty() && (point - result.back()).squaredNorm() < 1e-12) {
      continue;
    }
    result.push_back(point);
  }
  if (result.size() > 1 &&
      (result.front() - result.back()).squaredNorm() < 1e-12) {
    result.pop_back();
  }
  return result;
}

}  // namespace detail
}  // namespace plan_env
