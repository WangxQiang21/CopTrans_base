#pragma once
// This POD boundary isolates the binary distribution of CasADi (libstdc++ ABI 0)
// from ROS 2 / ament releases compiled with libstdc++ ABI 1.
#define HJ_MAX_PHASES 16
#define HJ_MAX_SPANS 12
struct HJPhase {
  int mode, spans, gear[3];
  double cruise_ramp_fraction, tau, h0[3], h1[3], coefficient[HJ_MAX_SPANS][3][6];
};
struct HJSeparation {
  int phase, span; double station; int vehicle; // vehicle -1 indicates cargo
  double ox, oy, half_length, half_width, nx, ny, obstacle_support;
  int other{-1};
  double other_x{},other_y{},other_half_length{},other_half_width{};
};
struct HJProblem {
  const HJSeparation *separations;
  int sep_count, route_yaw_chart, general_chart;
  double terminal_heading_window;
  double nlp_build_s, nlp_solve_s;
  char primal_status[80];
  double min_motion_duration, prepare_time, lateral_acceleration, steer_acceleration, yaw_acceleration, yaw_jerk;
  int count, bodies, movable;
  HJPhase phase[HJ_MAX_PHASES];
  double offset[3][2], start[6], goal[6];
  int fixed_goal[4]; // reference yaw, body headings
  double radius, speed, acceleration, jerk, yaw_rate, spin_rate, spin_acc, spin_jerk;
  double wheelbase, track, steer_angle, steer_rate, wheel_speed, duration_cap;
  double wall_seconds;
  int (*stop)(void*);
  void *opaque;
};
extern "C" int hj_optimize(HJProblem*,char *reason,int capacity);
// Offline numerical tests run the exact assembled objective and constraint
// graph. No finite differences or random sampling enter production planning.
struct HJDerivativeAudit {
  int seed, cases, rows, failed, nonfinite;
  double max_relative_error, max_near_zero_absolute_error;
};
extern "C" int hj_audit_derivatives(HJProblem*, HJDerivativeAudit*, char*, int);
