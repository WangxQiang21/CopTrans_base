// S4 pure C++ geometry tests.  No ROS master or PlannerNode instance is used.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "fwis_v3_planner/formation_collision.hpp"
#include "fwis_v3_planner/formation_model.hpp"
#include "fwis_v3_planner/formation_search.hpp"

namespace {

namespace formation = fwis::formation;

constexpr double kPi = 3.14159265358979323846;

formation::FormationVehicle make_vehicle(
    uint32_t id, double offset_x, double offset_y,
    const formation::FormationPose& start,
    const formation::FormationPose& goal) {
  const double sc = std::cos(start.psi);
  const double ss = std::sin(start.psi);
  const double gc = std::cos(goal.psi);
  const double gs = std::sin(goal.psi);
  formation::FormationVehicle vehicle;
  vehicle.vehicle_id = id;
  vehicle.offset_x = offset_x;
  vehicle.offset_y = offset_y;
  vehicle.offset_z = 0.0;
  vehicle.start_x = start.x + sc * offset_x - ss * offset_y;
  vehicle.start_y = start.y + ss * offset_x + sc * offset_y;
  vehicle.start_theta = start.psi;
  vehicle.goal_x = goal.x + gc * offset_x - gs * offset_y;
  vehicle.goal_y = goal.y + gs * offset_x + gc * offset_y;
  vehicle.goal_theta = goal.psi;
  vehicle.goal_heading_free = false;
  vehicle.body_length_m = 0.738;
  vehicle.body_width_m = 0.50;
  return vehicle;
}

formation::FormationRequest make_request(
    const formation::FormationPose& start,
    const formation::FormationPose& goal,
    const std::vector<std::pair<double, double>>& offsets) {
  formation::FormationRequest request;
  request.frame_id = "map";
  request.stamp_secs = 100;
  request.stamp_nsecs = 5;
  request.plan_id = 17;
  request.formation_id = "test_formation";
  request.start_qf = start;
  request.goal_qf = goal;
  request.allow_reverse = false;
  for (size_t i = 0; i < offsets.size(); ++i) {
    request.vehicles.push_back(make_vehicle(
        static_cast<uint32_t>(i), offsets[i].first, offsets[i].second,
        start, goal));
  }
  return request;
}

struct WorldRect {
  double x0;
  double y0;
  double x1;
  double y1;
};

formation::FormationMap make_map(
    int width, int height, double resolution,
    const std::vector<WorldRect>& obstacles = {}) {
  std::vector<int8_t> data(static_cast<size_t>(width) * height, 0);
  for (const auto& obstacle : obstacles) {
    const int ix0 = std::max(
        0, static_cast<int>(std::floor(obstacle.x0 / resolution)));
    const int iy0 = std::max(
        0, static_cast<int>(std::floor(obstacle.y0 / resolution)));
    const int ix1 = std::min(
        width - 1,
        static_cast<int>(std::ceil(obstacle.x1 / resolution)) - 1);
    const int iy1 = std::min(
        height - 1,
        static_cast<int>(std::ceil(obstacle.y1 / resolution)) - 1);
    for (int y = iy0; y <= iy1; ++y) {
      for (int x = ix0; x <= ix1; ++x) {
        data[static_cast<size_t>(y) * width + x] = 100;
      }
    }
  }
  return formation::FormationMap(width, height, resolution, 0.0, 0.0,
                                 std::move(data));
}

formation::FormationSearchConfig test_config() {
  formation::FormationSearchConfig config;
  config.xy_res = 0.20;
  config.yaw_res = kPi / 18.0;
  config.translation_step_m = 0.50;
  config.rotation_step_rad = kPi / 18.0;
  config.goal_xy_tol_m = 0.16;
  config.goal_yaw_tol_rad = 0.16;
  config.max_corner_step_m = 0.05;
  config.max_expansions = 50000;
  config.max_candidates = 4;
  config.max_wall_time_s = 10.0;
  return config;
}

formation::FormationPose integrate_path(
    const formation::FormationPose& start,
    const std::vector<formation::FormationPrimitive>& primitives) {
  formation::FormationPose qf = start;
  for (const auto& primitive : primitives) {
    formation::FormationPose next;
    EXPECT_TRUE(formation::integrate_primitive(qf, primitive, &next));
    qf = next;
  }
  return qf;
}

}  // namespace

TEST(FormationModel, AcceptsArbitraryFixedOffsetsAndCanonicalisesIds) {
  const auto request = make_request({2.0, 2.0, 0.35}, {6.0, 3.0, -0.4},
                                    {{0.0, 0.0}, {1.1, -0.7}, {-0.9, 0.2}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid) << validation.field << " " << validation.detail;
  ASSERT_EQ(validation.model.vehicles.size(), 3u);
  EXPECT_EQ(validation.model.vehicle_ids(),
            (std::vector<uint32_t>{0, 1, 2}));
}

TEST(FormationModel, RejectsBrokenRigidCenterAndDuplicateIds) {
  auto request = make_request({2.0, 2.0, 0.35}, {6.0, 3.0, -0.4},
                              {{0.0, 0.0}, {1.1, 0.0}});
  request.vehicles[1].start_x += 0.01;
  auto validation = formation::validate_formation_request(request);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.field, "vehicles.start_center");

  request = make_request({2.0, 2.0, 0.35}, {6.0, 3.0, -0.4},
                         {{0.0, 0.0}, {1.1, 0.0}});
  request.vehicles[1].vehicle_id = 0;
  validation = formation::validate_formation_request(request);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.field, "vehicles[1].vehicle_id");
}

TEST(FormationModel, RejectsMalformedRequestFieldsFailClosed) {
  auto request = make_request({2.0, 2.0, 0.35}, {6.0, 3.0, -0.4},
                              {{0.0, 0.0}, {1.1, 0.0}});
  request.vehicles[0].offset_z = 1e-9;
  auto validation = formation::validate_formation_request(request);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.field, "vehicles[0].offset_in_formation.z");

  request = make_request({2.0, 2.0, 0.35}, {6.0, 3.0, -0.4},
                         {{0.0, 0.0}, {1.1, 0.0}});
  request.frame_id = "odom";
  validation = formation::validate_formation_request(request);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.field, "header.frame_id");

  request = make_request({2.0, 2.0, 0.35}, {6.0, 3.0, -0.4},
                         {{0.0, 0.0}, {1.1, 0.0}});
  request.stamp_secs = 0;
  request.stamp_nsecs = 0;
  validation = formation::validate_formation_request(request);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.field, "header.stamp");

  request = make_request({2.0, 2.0, 0.35}, {6.0, 3.0, -0.4},
                         {{0.0, 0.0}, {1.1, 0.0}});
  request.vehicles[0].body_width_m = 0.0;
  validation = formation::validate_formation_request(request);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.field, "vehicles[0].body_shape");
}

TEST(FormationModel, PrimitiveIntegrationUsesFormationFrameAndExplicitPivot) {
  formation::FormationPrimitive translate;
  translate.geometry_type = formation::FormationPrimitive::TRANSLATION;
  translate.dx = 1.0;
  translate.dy = 0.0;
  formation::FormationPose next;
  ASSERT_TRUE(formation::integrate_primitive({2.0, 3.0, kPi / 2.0},
                                             translate, &next));
  EXPECT_NEAR(next.x, 2.0, 1e-12);
  EXPECT_NEAR(next.y, 4.0, 1e-12);
  EXPECT_NEAR(next.psi, kPi / 2.0, 1e-12);

  formation::FormationPrimitive rotate;
  rotate.geometry_type = formation::FormationPrimitive::ROTATION;
  rotate.dpsi = kPi / 2.0;
  rotate.pivot_x = 1.0;
  rotate.pivot_y = 0.0;
  ASSERT_TRUE(formation::integrate_primitive({2.0, 3.0, 0.0}, rotate, &next));
  EXPECT_NEAR(next.x, 3.0, 1e-12);
  EXPECT_NEAR(next.y, 2.0, 1e-12);
  EXPECT_NEAR(next.psi, kPi / 2.0, 1e-12);
}

TEST(FormationCollision, SatTreatsTouchingBodiesAsCollision) {
  formation::Obb first{0.0, 0.0, 0.0, 0.5, 0.25};
  formation::Obb separated{1.25, 0.0, 0.0, 0.5, 0.25};
  formation::Obb touching{1.0, 0.0, 0.0, 0.5, 0.25};
  EXPECT_TRUE(formation::collision_detail::separated(first, separated));
  EXPECT_FALSE(formation::collision_detail::separated(first, touching));
}

TEST(FormationCollision, DetectsVehicleOverlapAndMapObstacle) {
  const auto request = make_request({2.0, 2.0, 0.0}, {3.0, 2.0, 0.0},
                                    {{0.0, 0.0}, {0.2, 0.0}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid);
  const auto map = make_map(80, 80, 0.10);
  formation::FormationCollisionChecker checker(map, validation.model,
                                               validation.request);
  EXPECT_FALSE(checker.start_clear());

  const auto clean_request =
      make_request({2.0, 2.0, 0.0}, {3.0, 2.0, 0.0}, {{0.0, 0.0}, {1.2, 0.0}});
  const auto clean_validation =
      formation::validate_formation_request(clean_request);
  ASSERT_TRUE(clean_validation.valid);
  const auto blocked_map = make_map(80, 80, 0.10, {{2.35, 1.9, 2.65, 2.1}});
  formation::FormationCollisionChecker blocked(blocked_map,
                                                clean_validation.model,
                                                clean_validation.request);
  EXPECT_FALSE(blocked.start_clear());
}

TEST(FormationCollision, StartGoalAndRigidHeadingsRemainExplicit) {
  auto request = make_request({2.0, 2.0, 0.35}, {6.0, 3.0, -0.4},
                              {{0.0, 0.0}, {1.2, 0.0}});
  request.vehicles[0].start_theta = 0.55;
  request.vehicles[1].start_theta = 0.10;
  request.vehicles[0].goal_theta = -0.15;
  request.vehicles[1].goal_theta = -0.65;
  request.vehicles[1].goal_heading_free = true;
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid) << validation.field << " " << validation.detail;
  const auto map = make_map(120, 120, 0.10);
  formation::FormationCollisionChecker checker(map, validation.model,
                                               validation.request);

  const auto start_bodies = checker.start_bodies();
  EXPECT_DOUBLE_EQ(start_bodies[0].theta, 0.55);
  EXPECT_DOUBLE_EQ(start_bodies[1].theta, 0.10);
  const auto goal_bodies = checker.goal_bodies();
  EXPECT_DOUBLE_EQ(goal_bodies[0].theta, -0.15);
  EXPECT_DOUBLE_EQ(goal_bodies[1].theta, -0.65);

  formation::FormationPose rotated_qf = request.start_qf;
  rotated_qf.psi += 0.25;
  const auto rigid_bodies = checker.rigid_bodies(rotated_qf);
  EXPECT_NEAR(rigid_bodies[0].theta, 0.80, 1e-12);
  EXPECT_NEAR(rigid_bodies[1].theta, 0.35, 1e-12);
  EXPECT_NE(rigid_bodies[0].theta, rotated_qf.psi);
  EXPECT_NE(rigid_bodies[1].theta, rotated_qf.psi);
}

TEST(FormationCollision, RotationSweepDistinguishesMidArcObstacle) {
  const formation::FormationPose start{6.0, 6.0, 0.0};
  const formation::FormationPose goal{6.0, 6.0, kPi / 2.0};
  const auto request =
      make_request(start, goal, {{0.0, 0.0}, {0.0, 1.2}, {0.0, -1.2}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid);

  formation::FormationPrimitive rotate;
  rotate.geometry_type = formation::FormationPrimitive::ROTATION;
  rotate.dpsi = kPi / 2.0;
  rotate.pivot_x = 0.0;
  rotate.pivot_y = 0.0;

  const auto empty = make_map(120, 120, 0.10);
  formation::FormationCollisionChecker empty_checker(
      empty, validation.model, validation.request);
  ASSERT_TRUE(empty_checker.start_clear());
  ASSERT_TRUE(empty_checker.goal_clear());
  EXPECT_TRUE(empty_checker.sweep_clear(start, rotate, 0.05));

  // At +45 deg the top vehicle center is (5.1515, 6.8485).  The start and
  // goal formations remain clear, so only the swept path can reject this.
  const auto blocked = make_map(120, 120, 0.10,
                                {{5.00, 6.70, 5.30, 7.00}});
  formation::FormationCollisionChecker blocked_checker(
      blocked, validation.model, validation.request);
  ASSERT_TRUE(blocked_checker.start_clear());
  ASSERT_TRUE(blocked_checker.goal_clear());
  EXPECT_FALSE(blocked_checker.sweep_clear(start, rotate, 0.05));
}

TEST(FormationSearch, EmptyMapStraightLineSucceedsAsGeometryOnly) {
  const auto request =
      make_request({2.0, 6.0, 0.0}, {7.0, 6.0, 0.0},
                   {{0.0, 0.0}, {0.0, 1.2}, {0.0, -1.2}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid) << validation.field << " " << validation.detail;
  const auto map = make_map(120, 120, 0.10);
  const auto result = formation::search_formation_geometry(
      validation.request, validation.model, map, test_config());
  ASSERT_TRUE(result.success) << result.reason << " " << result.detail;
  EXPECT_TRUE(result.geometry_only);
  EXPECT_FALSE(result.primitives.empty());
  EXPECT_EQ(result.detail, "GEOMETRY_ONLY_UNCERTIFIED");
  for (const auto& primitive : result.primitives) {
    EXPECT_NE(primitive.geometry_type, formation::FormationPrimitive::COMPOSITE);
  }
  const auto end = integrate_path(validation.request.start_qf,
                                  result.primitives);
  EXPECT_LT(std::hypot(end.x - request.goal_qf.x,
                       end.y - request.goal_qf.y), 0.17);
  EXPECT_LT(std::abs(formation::wrap_angle(end.psi - request.goal_qf.psi)), 0.17);
}

TEST(FormationSearch, ObstacleDetourGeometricallySucceeds) {
  const auto request =
      make_request({2.0, 2.0, 0.0}, {8.0, 2.0, 0.0},
                   {{0.0, 0.0}, {0.0, 0.6}, {0.0, -0.6}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid) << validation.field << " " << validation.detail;
  const auto map = make_map(
      120, 120, 0.10,
      {{4.8, 0.0, 5.2, 2.8}, {4.8, 5.2, 5.2, 12.0}});
  const auto result = formation::search_formation_geometry(
      validation.request, validation.model, map, test_config());
  ASSERT_TRUE(result.success) << result.reason << " " << result.detail;
  EXPECT_GT(result.primitives.size(), 2u);
  const auto end = integrate_path(validation.request.start_qf,
                                  result.primitives);
  EXPECT_LT(std::hypot(end.x - request.goal_qf.x,
                       end.y - request.goal_qf.y), 0.17);
}

TEST(FormationSearch, GoalInsideObstacleFailsStatically) {
  const auto request = make_request({2.0, 2.0, 0.0}, {4.0, 4.0, 0.0},
                                    {{0.0, 0.0}, {1.2, 0.0}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid);
  const auto map = make_map(80, 80, 0.10, {{3.7, 3.7, 4.3, 4.3}});
  const auto result = formation::search_formation_geometry(
      validation.request, validation.model, map, test_config());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "GOAL_FORMATION_COLLISION");
  EXPECT_TRUE(result.geometry_only);
  EXPECT_TRUE(result.primitives.empty());
}

TEST(FormationSearch, ExplicitPivotRotationReachesRotatedGoal) {
  const auto request =
      make_request({6.0, 6.0, 0.0}, {6.0, 6.0, kPi / 2.0},
                   {{0.0, 0.0}, {0.0, 1.2}, {0.0, -1.2}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid) << validation.field << " " << validation.detail;
  const auto map = make_map(120, 120, 0.10);
  const auto result = formation::search_formation_geometry(
      validation.request, validation.model, map, test_config());
  ASSERT_TRUE(result.success) << result.reason << " " << result.detail;
  bool has_rotation = false;
  for (const auto& primitive : result.primitives) {
    if (primitive.geometry_type ==
        formation::FormationPrimitive::ROTATION) {
      has_rotation = true;
      EXPECT_DOUBLE_EQ(primitive.pivot_x, 0.0);
      EXPECT_DOUBLE_EQ(primitive.pivot_y, 0.0);
    }
  }
  EXPECT_TRUE(has_rotation);
}

TEST(FormationSearch, StartInsideObstacleFailsStatically) {
  const auto request = make_request({2.0, 2.0, 0.0}, {4.0, 2.0, 0.0},
                                    {{0.0, 0.0}, {1.2, 0.0}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid);
  const auto map = make_map(80, 80, 0.10, {{1.7, 1.7, 2.3, 2.3}});
  const auto result = formation::search_formation_geometry(
      validation.request, validation.model, map, test_config());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "START_FORMATION_COLLISION");
}

TEST(FormationSearch, NearGoalMustConnectBothPositionAndHeadingExactly) {
  const auto request = make_request({6.0, 6.0, 0.27}, {6.11, 6.09, 0.34},
                                    {{0.0, 1.0}, {-1.0, 0.0}, {1.0, 0.0}});
  const auto validation = formation::validate_formation_request(request);
  ASSERT_TRUE(validation.valid);
  const auto result = formation::search_formation_geometry(
      validation.request, validation.model, make_map(120, 120, 0.10), test_config());
  ASSERT_TRUE(result.success) << result.reason;
  const auto end = integrate_path(request.start_qf, result.primitives);
  EXPECT_NEAR(end.x, request.goal_qf.x, 1e-7);
  EXPECT_NEAR(end.y, request.goal_qf.y, 1e-7);
  EXPECT_NEAR(formation::wrap_angle(end.psi - request.goal_qf.psi), 0.0, 1e-7);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
