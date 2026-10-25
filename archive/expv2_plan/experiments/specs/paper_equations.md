# Chang et al. (2026): spatial model transcription ledger

Source: `papers/Generic motion planning for multi-locomotion autonomous vehicles_ A spatio-temporal decoupling framework to accommodate multiple maneuvering modes.pdf` (Information Sciences 754, 2026, article 123675). This is a transcription ledger, not yet a code-ready specification. Equation images must be visually checked before implementation; `pdftotext` loses several fraction bars, roots, subscripts, and vector layout.

## Framework and variables

- Spatial layer jointly chooses pose samples and mode sequence; temporal layer subsequently assigns timing/speed and locally optimizes mode transitions (Sections 3.2–3.3).
- Switched model: `ξ̇ = Σ_i ξ̇_Mi`; each mode contribution equals its continuous dynamics when binary `α_i=1`, otherwise zero (Eq. 1). `α_i` is binary and one-hot (Eq. 2); at each sample `Σ_i α_i(k)=1` (Eq. 15).
- For the running 4WID-4WIS example, CoG pose is `(x,y,φ)`, augmented continuous-time state is `ξ=[x,y,φ,v_r]^T`, and controls are `u=[δ_r,a_r]^T` (Section 2.1). `L_f,L_r` are CoG-to-axle distances, `L=L_f+L_r`, `B` is track. Paper's `p(k)∈{-1,+1}` is travel direction and `Δd` is nominal spatial step; Eq. (10) substitutes `v_u(k)ΔT'(k)=p(k)Δd`, so spatial discretization is not uniform in time.
- The paper explicitly distinguishes a virtual/master steering input from individual wheel steering and speed; deterministic bijective mappings are required for a faithful reproduction (Eqs. 3–4). For the current Ranger planning adaptation, per user scope, treat control as perfect and constrain the ideal geometric modes; wheel-level actuation is deferred.

## Mode semantics from the paper

- ASM: pose position and heading change; 4WID-4WIS example constrains ICR to the line through CoG perpendicular to the longitudinal axis. Virtual rear wheel steering/speed is used to parameterize the mode.
- DMM: heading stays fixed while position changes; in the example ICR is at CoG and all four wheel steering angles/speeds are equal.
- ZRM: position stays fixed while heading changes; the example uses a wheel-level steering/speed mapping.
- These are paper's abstract modes. Mapping them to Ranger's A/L/P/S enums remains a separate hardware claim (see `mode_mapping.csv`).

## Published simulation geometry (Table 3)

The paper's 4WID-4WIS simulation gives wheelbase `L=2.91 m`, track `B=1.675 m`, CoG-to-front axle `Lf=1.25 m`, CoG-to-rear axle `Lr=1.66 m`, front overhang `0.5 m`, rear overhang `0.5 m`. Thus derived overall length is 3.91 m. Table 3 does not list overall width; do not infer it from track. These values are for reproducing the paper's simulated vehicle, not Ranger.

## Equations requiring visual verification

Visual inspection of PDF printed page 5 confirms Eq. (5) as printed: ASM has `x_dot = v_r cos(δ_r) cos(φ)`, `y_dot = v_r sin(δ_r) sin(φ)`, `φ_dot = -v_r sin(δ_r)/L_r`, `v_dot_r=a_r`; DMM has `x_dot=v_r cos(δ_r+φ)`, `y_dot=v_r sin(δ_r+φ)`, `φ_dot=0`, `v_dot_r=a_r`; ZRM has `[0,0,-v_r/L_r,a_r]^T`. Visual inspection of printed page 9 confirms Eq. (11) exactly as printed: `x[k+1]=x[k]+pΔd cos(δ_r[k])cos(φ[k])`, `y[k+1]=y[k]+pΔd sin(δ_r[k])sin(φ[k])`, `φ[k+1]=φ[k]-pΔd sin(δ_r[k])/L_r`. Eq. (12) is `x,y` fixed and `φ[k+1]=φ[k]+pΔd/sqrt(L_r^2+B^2/4)`.

These printed ASM translational equations appear inconsistent with the usual planar rotation of a velocity vector at steering angle `δ_r` (and do not obviously reduce to ordinary forward motion at `δ_r=0, φ≠0`). Treat this as a source-model ambiguity/possible publication error, not a transcription/OCR error. Before implementing, check the article's definitions, original vector convention, and author supplementary material if available; record both literal reproduction and any corrected interpretation as separate model variants. Do not silently “fix” the equations.

The discrete mode model is reported in Eqs. (13)–(17): Euler discretization, spatial step substitution, one-hot activation and Big-M conditional dynamics. Big-M pose bounds are derived from coarse path samples using trust-region widths `β_x, β_y, β_φ`; they are local bounds, not arbitrary constants.

Constraints/objective reported in Eqs. (18)–(30): mapped individual steering limits; feature-point STC membership; fixed start pose and terminal body containment in parking region with heading tolerance and terminal steering condition; length, mode-specific steering change, mode switching and direction reversal costs, combined as a weighted sum. Eq. (21) was visually checked on printed page 10: it sums squared x and y increments (no square root in the displayed objective). Preserve the printed objective literally for reproduction, but note its dimensional mismatch with path length and compare against a distance objective only as a declared variant.

## Mathematical type audit (implementation gate)

The article calls the spatial formulation a MIP and reports Gurobi. The extracted model contains binary mode/direction decisions and nonlinear kinematics with trigonometric functions; mode activation can couple binaries to nonlinear expressions. Unless the authors' approximation/discretization reformulates these terms, the exact model is a mixed-integer nonlinear program, not a linear MIP. Determine the actual solved formulation from the complete Eq. (11)–(30), implementation details, and paper's solver settings before selecting a solver. A Gurobi mention alone does not prove an exact nonlinear model was solved as stated.

## Freeze checklist

1. Visually transcribe Eqs. (4)–(7), (11)–(12), (13)–(30), and switching equations from the PDF pages; retain page/equation references.
2. Numerically check straight motion, zero steering, sign of yaw under forward/reverse, and each mode's defining invariant.
3. Record all approximation tables/relaxations and their error bounds; do not label approximate results exact reproduction.
4. Only then implement the independent kinematics specification.
