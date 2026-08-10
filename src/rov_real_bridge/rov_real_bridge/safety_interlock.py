"""Software interlock for REAL BlueROV2 actuation.

Design rule (project safety contract): launching any normal ROS 2 stack must
NEVER be able to move the real vehicle. Real actuation requires BOTH explicit
flags AND live mode:

    vehicle_in_water:=true  AND  allow_real_actuation:=true
    AND real_control_mode:=live

Everything else — including any parsing error, missing parameter, or unknown
mode string — fails CLOSED (no actuation).

This module is pure Python (no rclpy import) so the interlock logic is fully
unit-testable and reusable from any node.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

SHADOW = "shadow"
LIVE = "live"
VALID_MODES = (SHADOW, LIVE)


def _strict_bool(value: Any) -> bool:
    """Interpret a parameter as boolean, failing CLOSED on anything unclear."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() == "true"
    # Ints, None, floats, lists... all refuse.
    return False


@dataclass
class InterlockDecision:
    may_actuate: bool
    reason: str


@dataclass
class SafetyInterlock:
    """Fail-closed actuation gate.

    All fields default to the SAFE state. `evaluate()` is the single
    authority; callers must not re-derive permission themselves.
    """

    vehicle_in_water: bool = False
    allow_real_actuation: bool = False
    real_control_mode: str = SHADOW
    # Monotonic counters for auditing.
    blocked_count: int = 0
    permitted_count: int = 0
    _audit: List[str] = field(default_factory=list)

    @classmethod
    def from_params(cls, params: Dict[str, Any]) -> "SafetyInterlock":
        mode = params.get("real_control_mode", SHADOW)
        mode = mode.strip().lower() if isinstance(mode, str) else SHADOW
        if mode not in VALID_MODES:
            mode = SHADOW  # unknown mode -> safest mode
        return cls(
            vehicle_in_water=_strict_bool(params.get("vehicle_in_water", False)),
            allow_real_actuation=_strict_bool(
                params.get("allow_real_actuation", False)),
            real_control_mode=mode,
        )

    def evaluate(self) -> InterlockDecision:
        if self.real_control_mode != LIVE:
            return InterlockDecision(False, f"mode={self.real_control_mode} (shadow: compute+log only)")
        if not self.vehicle_in_water:
            return InterlockDecision(False, "vehicle_in_water is false")
        if not self.allow_real_actuation:
            return InterlockDecision(False, "allow_real_actuation is false")
        return InterlockDecision(True, "all interlocks satisfied (LIVE)")

    def gate(self, command_description: str) -> InterlockDecision:
        """Gate one outgoing command; count and audit the decision."""
        decision = self.evaluate()
        if decision.may_actuate:
            self.permitted_count += 1
        else:
            self.blocked_count += 1
        self._audit.append(
            f"{'PERMIT' if decision.may_actuate else 'BLOCK'}: "
            f"{command_description} ({decision.reason})"
        )
        if len(self._audit) > 1000:
            del self._audit[: len(self._audit) - 1000]
        return decision

    @property
    def audit_tail(self) -> List[str]:
        return list(self._audit[-20:])


def forbid_ground_truth_topics(topics: List[str]) -> Optional[str]:
    """Return an error string if any topic is a ground-truth topic.

    Planner/control-path nodes must call this on their subscription list;
    `/ground_truth/*` is reserved for logger/validator/analyzer only.
    """
    for t in topics:
        if t.startswith("/ground_truth"):
            return (
                f"forbidden subscription {t!r}: ground-truth topics must never "
                "feed the control path"
            )
    return None
