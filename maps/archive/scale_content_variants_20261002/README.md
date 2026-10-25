# Archived content-edited map variants

These three generated maps were superseded after identifying that the actual mismatch was the global map-to-vehicle scale, not the obstacle layout:

- `P16_complex_parking_tight`: added a local constriction.
- `P11_case_a_tight`: inflated obstacles by 0.50 m.
- `P11_case_c_tight`: inflated obstacles by 1.00 m.

They are retained here for recovery/history, but are intentionally outside `maps/paper_digitized/` so `run_demo.sh --list-maps` does not offer them as current maps. Current maps preserve the paper obstacle layout and apply a uniform metric scale in `map.yaml`.
