#!/usr/bin/env python3
"""Isaac bridge: /base/hold_state -> wheel joint position hold (NO FixedJoint).

Paste BaseHoldBridge into your Isaac demo loop. Requires:
  - ros2 run base_hold hold_node
  - same ROS_DOMAIN_ID
  - articulation + dof_names + wheel joint name list

Safe vs previous parking FixedJoint (base pinned to world) which could crash Isaac.
"""

from __future__ import annotations

import threading

from std_msgs.msg import Bool

# Prefer after colcon install / PYTHONPATH to workspace:
#   from base_hold.isaac_wheel_hold import IsaacWheelHold


class BaseHoldBridge:
    """Subscribe to /base/hold_state and toggle IsaacWheelHold."""

    def __init__(
        self,
        node,
        stage,
        articulation,
        dof_names,
        wheel_joint_names,
        *,
        state_topic: str = "/base/hold_state",
        release_damping: float = 140.0,
        release_max_force: float = 350.0,
    ):
        from base_hold.isaac_wheel_hold import IsaacWheelHold

        self._hold = IsaacWheelHold(
            stage,
            wheel_joint_names,
            release_damping=release_damping,
            release_max_force=release_max_force,
        )
        self._articulation = articulation
        self._dof_names = list(dof_names)
        self._lock = threading.Lock()
        self._want = False
        node.create_subscription(Bool, state_topic, self._on_state, 10)

    def _on_state(self, msg: Bool) -> None:
        with self._lock:
            self._want = bool(msg.data)

    @property
    def held(self) -> bool:
        return self._hold.held

    def spin_once(self) -> None:
        """Call every sim step (after articulation is valid)."""
        with self._lock:
            want = self._want
        if want and not self._hold.held:
            self._hold.engage(
                articulation=self._articulation, dof_names=self._dof_names
            )
            print("[base_hold] wheel position HOLD on", flush=True)
        elif not want and self._hold.held:
            self._hold.release()
            print("[base_hold] wheel position HOLD off (nav)", flush=True)
        elif self._hold.held:
            self._hold.tick()


# --- usage sketch -----------------------------------------------------------
#
# WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]  # or 4 mecanum
# bridge = BaseHoldBridge(ros_node, stage, articulation, dof_names, WHEEL_JOINTS)
# while simulation_app.is_running():
#     world.step(render=True)
#     if bridge.held:          # when held, do not apply cmd_vel wheel velocities
#         pass
#     else:
#         apply_cmd_vel(...)
#     bridge.spin_once()
#
# After dock:
#   ros2 service call /base/hold std_srvs/srv/SetBool "{data: true}"
#   # move arm — wheels resist reaction torque via PD position hold
#   ros2 service call /base/hold std_srvs/srv/SetBool "{data: false}"

if __name__ == "__main__":
    print(__doc__)
