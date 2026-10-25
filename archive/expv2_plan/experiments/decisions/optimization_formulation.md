# Spatial optimization formulation decision record

Status: **open; Phase 3 blocked pending equation verification and solver/formulation choice.**

## Evidence so far

- Chang et al. (2026) describes spatial pose/mode co-optimization and calls it MIP; the text names Gurobi for the spatial problem.
- The 4WID-4WIS spatial dynamics include sine/cosine of heading and steering, direction `p`, and one-hot mode indicators. Big-M conditional activation couples mode binaries with dynamics. The exact resulting class is generally MINLP unless the paper uses a linearization, lookup table, or other relaxation.
- Current environment inspection found CasADi 3.7.2 and SciPy 1.10.1 importable, with no Gurobi/Pyomo/CVXPY binding discovered. This is only a module/binary discovery, not a solver benchmark. A PATH entry named `cbc` was a CodeBuddy CLI symlink, not the COIN-OR CBC solver.

## Decision path

1. Finish visual equation transcription and search paper for the exact relaxation/discretization and Gurobi model class/settings.
2. Check intended experiment machine for solver binary, Python binding, license, and supported model classes.
3. If exact formulation is supported, build a tiny two-mode model and compare residuals against direct propagation.
4. If not, select and quantify a bounded approximation (e.g. piecewise affine trig envelopes) or use finite-topology candidate enumeration with continuous per-mode optimization. Report the latter as an adaptation, not exact Chang MIP reproduction.

No Phase 3 implementation or solver choice is authorized by the current evidence; Phase 0 can continue with specification and compatibility work.
