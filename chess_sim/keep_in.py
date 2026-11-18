"""Working envelope for Cartesian targets, in board-frame millimetres.

Learned on the real arm (2026-11-17): the controller should not wander off the
playing surface. Leaving it keeps the board in view, and a mis-signed Cartesian
target is otherwise capable of driving the jaws into the table beside it.

The same bounds are enforced by the live app, so a violation means the same
thing on the bench and in simulation.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class KeepIn:
    """Axis-aligned working envelope centred on the board origin."""

    x_mm: float = 118.0
    y_mm: float = 118.0
    z_min_mm: float = -5.0
    z_max_mm: float = 320.0
    enforce: bool = True

    def problems(self, point_mm) -> list[str]:
        """Reasons a board-frame point is outside, empty when it is inside."""
        x, y, z = (float(v) for v in point_mm)
        out: list[str] = []
        if abs(x) > self.x_mm:
            out.append(f"x {x:.0f} outside +/-{self.x_mm:.0f}")
        if abs(y) > self.y_mm:
            out.append(f"y {y:.0f} outside +/-{self.y_mm:.0f}")
        if not self.z_min_mm <= z <= self.z_max_mm:
            out.append(f"z {z:.0f} outside {self.z_min_mm:.0f}..{self.z_max_mm:.0f}")
        return out

    def contains(self, point_mm) -> bool:
        return not self.problems(point_mm)

    def require(self, point_mm) -> None:
        """Raise ``ValueError`` when the envelope is enforced and violated."""
        if not self.enforce:
            return
        problems = self.problems(point_mm)
        if problems:
            raise ValueError("keep-in zone: " + "; ".join(problems))
