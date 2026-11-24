"""A-only virtual rigid formation backend: G1 analytic geometry only."""
from .contracts import (
    CurvatureJet,
    FormationSpatialState,
    GoalPolicy,
    InitialAssumption,
    ValidationResult,
    VehicleSpatialJet,
    VirtualAContractError,
    coerce_goal_policy,
    validate_formation_model,
)
from .geometry import (
    C2CurvatureProfile,
    VirtualAPath,
    check_vehicle_curvatures,
    map_spatial_state,
    validate_initial_state,
)

__all__ = [
    "C2CurvatureProfile",
    "CurvatureJet",
    "FormationSpatialState",
    "GoalPolicy",
    "InitialAssumption",
    "ValidationResult",
    "VehicleSpatialJet",
    "VirtualAContractError",
    "VirtualAPath",
    "check_vehicle_curvatures",
    "coerce_goal_policy",
    "map_spatial_state",
    "validate_formation_model",
    "validate_initial_state",
]