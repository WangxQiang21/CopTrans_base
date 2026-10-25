# Ranger ideal spatial model (prototype v0)

This is the planning abstraction approved for the current stage. It is separate from Chang's literal equations in `paper_equations.md` and assumes perfect tracking.

## State and footprint

Pose is `q=(x,y,θ)` at the vehicle rectangle center. The manual gives `length=0.738 m`, `width=0.500 m`, `wheelbase=0.494 m`, `track=0.364 m`. The planner uses a centered rectangle; CoG/axle offset is not given, so the chosen reference is an explicit idealization. The minimum Ackermann radius 0.810 m is provisionally applied to this reference point although the manual does not specify which point it describes.

## Ideal spatial propagation

Let `s` be signed travel distance.

- **A (Ackermann)**: constant signed curvature `κ`, `|κ|≤1/0.810 m⁻¹`; `θ⁺=θ+κs`. For `κ≈0`, `(x⁺,y⁺)=(x+s cosθ,y+s sinθ)`. Otherwise `x⁺=x+[sin(θ+κs)-sinθ]/κ`, `y⁺=y-[cos(θ+κs)-cosθ]/κ`. Forward/reverse are represented by the sign of `s`.
- **P (oblique/diagonal)**: relative steering direction `φ∈[-π/2,π/2]`; `(x⁺,y⁺)=(x+s cos(θ+φ),y+s sin(θ+φ))`, `θ⁺=θ`. The prototype action set samples `φ∈{-π/4,0,+π/4}`.
- **L (lateral)**: a DMM subcase with `φ=±π/2`; fixed heading, pure lateral translation. This is represented separately for comparison with the existing planner mode label.
- **S (spin)**: `(x⁺,y⁺)=(x,y)`, `θ⁺=θ+Δθ`.

Mode switches are instantaneous and have no transient because of the perfect-controller assumption. The selected path is therefore geometrically feasible in this ideal model only.

## Search and cost

Prototype is discrete-control A* with 0.25 m translational primitives, 15° spin primitives, XY key bins of 0.20 m, yaw key bins of 15°, and last-mode in the search key. Current action costs are translation length, `0.45 m × |Δθ|` for spin, and 0.18 m per mode switch. The weighted heuristic is Euclidean position lower bound combined with a yaw lower bound by `max(distance, 0.45×|wrapped yaw error|)`. `weight=1` is A*; `weight>1` is weighted A* and does not guarantee optimality. Terminal acceptance uses 0.23 m position tolerance and 12° yaw tolerance.

These values are pilot defaults, not tuned or claimed optimal. A refinement stage should sweep spatial/yaw resolution, primitive set, switch cost, and heuristic weight.

## Collision handling

The S09 map has 0.02 m cells. Occupied cells are inflated by a 3×3 cell neighborhood (up to 2.8 cm diagonally). The 0.738×0.500 m rectangle is sampled at roughly 3 cm spacing and expanded 1.5 cm to cover pose/yaw cache quantization. Search-time primitive sweeps and final candidate replay both sample at ≤1 cm translation / ≤1° rotation. Map boundaries count as occupied.

The inflated raster and expanded body are intentionally conservative; successful plans should be rechecked with the project's independent OBB swept validator before ROS integration. The prototype does not yet calculate a continuous minimum-clearance value.
