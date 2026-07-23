#!/usr/bin/env python3
"""Minimal Isaac bridge: /base/hold_state -> FixedJoint parking brake.

Paste / adapt into an Isaac demo loop (e.g. mobile_manipulator_demo.py).
Requires:
  - ROS hold_node running (publishes /base/hold_state)
  - This process shares ROS_DOMAIN_ID with hold_node
  - pxr UsdPhysics available (Isaac python)

Not a full demo — copy the BaseHoldBridge class into your Isaac script.
"""

from __future__ import annotations

# --- copy from here ---------------------------------------------------------

import threading

from std_msgs.msg import Bool

# Inside Isaac, prefer:
#   from base_hold.isaac_parking_brake import engage, release, is_engaged
# Or vendor the helpers next to your demo.


class BaseHoldBridge:
    """Subscribe to latched /base/hold_state and toggle FixedJoint brake."""

    def __init__(
        self,
        node,
        stage,
        articulation_path: str,
        *,
        get_world_pose,
        state_topic: str = "/base/hold_state",
    ):
        """
        get_world_pose: callable () -> (xyz: tuple[float,float,float],
                                        quat_wxyz: tuple[float,float,float,float])
        """
        from base_hold import isaac_parking_brake as brake

        self._brake = brake
        self._stage = stage
        self._articulation_path = articulation_path
        self._get_world_pose = get_world_pose
        self._lock = threading.Lock()
        self._want_hold = False
        self._applied = False

        node.create_subscription(Bool, state_topic, self._on_state, 10)

    def _on_state(self, msg: Bool) -> None:
        with self._lock:
            self._want_hold = bool(msg.data)

    def spin_once(self) -> None:
        """Call every sim step after physics update / pose is valid."""
        with self._lock:
            want = self._want_hold
        engaged = self._brake.is_engaged(self._stage)
        if want and not engaged:
            xyz, quat = self._get_world_pose()
            path = self._brake.engage(
                self._stage,
                self._articulation_path,
                xyz,
                world_quat_wxyz=quat,
            )
            print(f"[base_hold] parking brake ON {path} @ {xyz}", flush=True)
            self._applied = True
        elif not want and engaged:
            self._brake.release(self._stage)
            print("[base_hold] parking brake OFF", flush=True)
            self._applied = False


# --- usage sketch -----------------------------------------------------------
#
# bridge = BaseHoldBridge(
#     ros_node,
#     stage,
#     articulation_path="/World/.../robot"),
#     get_world_pose=lambda: (
#         tuple(articulation.get_world_pose()[0]),
#         # convert articulation quat to (w,x,y,z)
#         quat_wxyz,
#     ),
# )
# while simulation_app.is_running():
#     ...
#     world.step(render=True)
#     bridge.spin_once()
#
# Mission after dock:
#   ros2 service call /base/hold std_srvs/srv/SetBool "{data: true}"
#   # run arm
#   ros2 service call /base/hold std_srvs/srv/SetBool "{data: false}"

if __name__ == "__main__":
    print(__doc__)
