"""Single interpreter for the cargo-transport formation contract.

The authoritative values live in
``catkin_ws/src/fwis_v3_demo/config/transport_formation.yaml``.  The demo
launch file loads that file into the goal bridge and playback nodes as
private parameters, so both nodes and the footprint tool read the same ids,
offsets, body dimensions and heading policies instead of keeping separate
hard-coded copies.

The transport contract fixes the geometry only.  Body headings ``theta_i``
are independent variables: the configured ``initial_heading_policy`` is used
when an experiment starts (a manual ``2D Pose Estimate`` or the configured
fallback), while later goals keep whatever heading the playback actually
shows.  ``goal_heading_free`` controls terminal ``theta_i``. The selected
planning backend determines the terminal policy for cargo ``psi_F``.
"""
import math
from dataclasses import dataclass

POLICY_CARGO_HEADING = "cargo_heading"
POLICY_TANGENT = "tangent"
SUPPORTED_POLICIES = (POLICY_CARGO_HEADING, POLICY_TANGENT)

REQUIRED_FIELDS = (
    "formation_id",
    "vehicle_ids",
    "offsets",
    "body_length_m",
    "body_width_m",
    "allow_reverse",
    "goal_heading_free",
    "initial_heading_policy",
)


class TransportContractError(ValueError):
    """Raised when the transport contract is missing or inconsistent."""


def _finite(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TransportContractError(
            "{} must be a finite number".format(name))
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        raise TransportContractError(
            "{} must be a finite {} number".format(
                name, "positive" if positive else ""))
    return result


def _boolean(value, name):
    if not isinstance(value, bool):
        raise TransportContractError("{} must be a bool".format(name))
    return value


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _vehicle_id(value, name):
    if isinstance(value, bool) or not isinstance(value, int):
        if isinstance(value, float) and value.is_integer():
            value = int(value)
        else:
            raise TransportContractError(
                "{} must contain integer ids".format(name))
    if value < 0:
        raise TransportContractError(
            "{} must contain non-negative ids".format(name))
    return int(value)


def contract_from_mapping(values):
    """Validate a mapping with exactly the contract fields."""
    if not isinstance(values, dict):
        raise TransportContractError("transport contract must be a mapping")
    missing = [name for name in REQUIRED_FIELDS if name not in values]
    if missing:
        raise TransportContractError(
            "transport contract is missing {}".format(", ".join(missing)))
    formation_id = values["formation_id"]
    if not isinstance(formation_id, str) or not formation_id.strip():
        raise TransportContractError(
            "formation_id must be a non-empty string")
    raw_ids = values["vehicle_ids"]
    if not isinstance(raw_ids, (list, tuple)) or not raw_ids:
        raise TransportContractError(
            "vehicle_ids must be a non-empty list")
    vehicle_ids = tuple(
        _vehicle_id(value, "vehicle_ids") for value in raw_ids)
    if len(set(vehicle_ids)) != len(vehicle_ids):
        raise TransportContractError(
            "vehicle_ids must be unique; ids must not be swapped")
    raw_offsets = values["offsets"]
    if (not isinstance(raw_offsets, (list, tuple))
            or len(raw_offsets) != len(vehicle_ids)):
        raise TransportContractError(
            "offsets must contain one [x, y] pair per vehicle")
    offsets = []
    for index, pair in enumerate(raw_offsets):
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise TransportContractError(
                "offsets[{}] must be [x, y]".format(index))
        offsets.append((_finite(pair[0], "offsets[{}][0]".format(index)),
                        _finite(pair[1], "offsets[{}][1]".format(index))))
    policy = values["initial_heading_policy"]
    if policy not in SUPPORTED_POLICIES:
        raise TransportContractError(
            "initial_heading_policy must be one of {}; got {!r}".format(
                SUPPORTED_POLICIES, policy))
    return TransportContract(
        formation_id=formation_id.strip(),
        vehicle_ids=vehicle_ids,
        offsets=tuple(offsets),
        body_length_m=_finite(
            values["body_length_m"], "body_length_m", positive=True),
        body_width_m=_finite(
            values["body_width_m"], "body_width_m", positive=True),
        allow_reverse=_boolean(values["allow_reverse"], "allow_reverse"),
        goal_heading_free=_boolean(
            values["goal_heading_free"], "goal_heading_free"),
        initial_heading_policy=policy)


def load_contract(get_param):
    """Read the contract from private parameters (roslaunch rosparam load).

    ``get_param`` is ``rospy.get_param``-like; a missing key raises KeyError
    so a node started without the authoritative file fails closed instead of
    silently using a stale hard-coded formation.
    """
    values = {}
    for name in REQUIRED_FIELDS:
        values[name] = get_param(name)
    return contract_from_mapping(values)


@dataclass(frozen=True)
class TransportContract:
    formation_id: str
    vehicle_ids: tuple
    offsets: tuple
    body_length_m: float
    body_width_m: float
    allow_reverse: bool
    goal_heading_free: bool
    initial_heading_policy: str

    def offset_map(self):
        return dict(zip(self.vehicle_ids, self.offsets))

    def headings_for_policy(self, cargo_theta):
        """Initial body headings for a new experiment at ``cargo_theta``.

        ``cargo_heading`` uses the documented example policy
        ``theta_i(0) = psi_F(0)``.  ``tangent`` points every body along the
        counter-clockwise tangent of its centre about the cargo reference,
        which is the analytic E05 rotation baseline.
        """
        theta = _finite(cargo_theta, "cargo_theta")
        if self.initial_heading_policy == POLICY_CARGO_HEADING:
            return {vehicle_id: theta for vehicle_id in self.vehicle_ids}
        headings = {}
        for vehicle_id, (offset_x, offset_y) in zip(
                self.vehicle_ids, self.offsets):
            if math.hypot(offset_x, offset_y) <= 1e-12:
                raise TransportContractError(
                    "tangent policy needs every vehicle offset to be away "
                    "from the cargo reference; vehicle_id={}".format(
                        vehicle_id))
            headings[vehicle_id] = _wrap_angle(
                math.atan2(offset_x, -offset_y) + theta)
        return headings
