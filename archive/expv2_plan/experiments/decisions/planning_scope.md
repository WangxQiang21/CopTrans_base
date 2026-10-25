# Current study scope: geometric planning with perfect tracking

Status: user-confirmed scope.

## In scope now

- Plan collision-free geometric pose paths and mode sequences for Ranger Mini 2.0 using documented chassis geometry and supported motion modes.
- Use ideal kinematic mode models and assume a perfect low-level controller tracks every accepted pose/mode trajectory exactly.
- Enforce path geometry limits that the manual supports: body footprint, Ackermann minimum turning radius (with reference-point caveat), oblique steering-direction range up to ±90°, and ideal in-place rotation.
- Treat time, acceleration, steering actuator dynamics, wheel-by-wheel inverse kinematics, steering-rate limits, transient mode transitions, localization error, and closed-loop tracking as deferred to the subsequent control stage.

## Modeling convention

- The planner may switch modes instantaneously at a path sample under the perfect-control abstraction. Switching cost may discourage excessive switching, but no transition time or steering transient is modeled now.
- A geometric path is accepted only if its ideal kinematics and swept vehicle footprint are collision-free. Perfect tracking does not relax geometric nonholonomic constraints or footprint collision checks.
- Use explicit `planning_assumption` labels for unsupported idealizations. Do not claim that geometric feasibility proves low-level executability.
- Keep paper reproduction (Chang's own simulated vehicle and equations) separate from Ranger adaptation. For Ranger, implement the documented high-level motion modes, not Chang's specific wheel actuator allocation.

## Deferred to controller stage

Wheel-level command mapping and feedback, mode transition trajectory, steering rate/acceleration, speed scheduling, actuator saturation beyond documented high-level mode limits, tracking-error margins, and physical closed-loop tests. Revisit these only after the planning geometry/mode-selection experiments are complete.

## Manual discrepancies / unresolved geometry

- The manual has two specification tables listing different maximum speeds (2.6 m/s and 5.4 km/h); speed does not affect the current spatial planner.
- The manual gives overall body dimensions and wheelbase but not CoG/rotation-center offset or front/rear overhang. Use the documented rectangular footprint centered on the planner reference point as an explicit assumption until a dimensioned drawing is recovered. Do not derive `Lf=Lr=wheelbase/2`.
- The stated 0.810 m Ackermann minimum turning radius has no clearly stated reference point. Use as a provisional curvature bound with a source caveat; verify which point the radius describes if geometry becomes sensitive to it.
