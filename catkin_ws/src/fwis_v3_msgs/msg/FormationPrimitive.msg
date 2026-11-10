# Formation-reference primitive, NOT a per-vehicle A/O/S mode primitive.
# All coordinates and angles are expressed in the current formation frame
# (x along psi_F, y left of psi_F) before the primitive is integrated.
uint8 TRANSLATION=0
uint8 ROTATION=1
uint8 COMPOSITE=2
uint8 geometry_type

# TRANSLATION: formation-reference displacement (dx, dy) in metres.
# psi_F is unchanged; every vehicle center translates by the same world vector.
float64 dx
float64 dy

# ROTATION: formation-reference angle dpsi in radians about the explicit
# pivot (pivot_x, pivot_y). The pivot is part of the contract, never hidden in
# dx/dy. A rigid formation rotation is only a geometry candidate, not per-vehicle
# Spin feasibility.
float64 dpsi
float64 pivot_x
float64 pivot_y
