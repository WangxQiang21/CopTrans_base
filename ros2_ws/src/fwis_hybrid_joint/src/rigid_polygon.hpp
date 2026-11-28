#pragma once

#include "fwis_hybrid_joint/model.hpp"

namespace fwis_hybrid_joint {
// Keeps the existing rectangular checker for vehicles and handles a possibly
// concave cargo outline as the exact union of triangles in the cargo frame.
class FormationCollision {
public:
  FormationCollision(const Request&, const CargoPolygon& polygon);
  fwis_planning_core::CollisionResult check(
    Pose2d reference, const double* headings, std::size_t count, double padding = 0.) const;
  double polygon_radius() const noexcept { return radius_; }
  const char* reason(fwis_planning_core::CollisionReason collision) const noexcept;

private:
  using Point = fwis_planning_core::Point2d;
  using Triangle = std::array<Point, 3>;
  bool polygon_clear(Pose2d reference, double padding) const;

  fwis_planning_core::CollisionKernel bodies_;
  Point origin_;
  double resolution_, right_, top_, radius_{};
  std::vector<Triangle> triangles_;
  std::vector<std::vector<std::size_t>> occupied_rows_;
};
}
