import os
import sys
import json
import time
import asyncio
import threading
from typing import Set

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String, Bool, Int32, Empty
from std_srvs.srv import Trigger, SetBool

try:
    from serving_robot_interfaces.srv import OrderRequest
    ORDER_REQUEST_SRV_AVAILABLE = True
except ImportError as exc:
    OrderRequest = None
    ORDER_REQUEST_SRV_AVAILABLE = False
    ORDER_REQUEST_IMPORT_ERROR = str(exc)
else:
    ORDER_REQUEST_IMPORT_ERROR = ""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn

from serving_hmi.order_manager import OrderManager, OrderStatus

app = FastAPI(title="Pizza Serving Robot HMI Backend")

# WebSocket connections storage
connected_clients: Set[WebSocket] = set()

# Global state manager
order_manager = OrderManager()

class HMIBridgeNode(Node):
    def __init__(self):
        super().__init__('serving_hmi_bridge_node')

        self.declare_parameter('robot_namespace', 'robot1')
        robot_namespace = str(
            self.get_parameter('robot_namespace').value
        ).strip('/')
        topic_prefix = f'/{robot_namespace}' if robot_namespace else ''
        self.camera_topic = f'{topic_prefix}/camera/color/image_raw'
        self.odom_topic = f'{topic_prefix}/nav_robot/odom'
        
        # System status state
        self.last_clock_time = 0.0
        self.last_clock_recv_wall_time = 0.0
        self.clock_hz = 0.0
        self.isaac_sim_connected = False
        
        self.last_robot_status_recv_time = 0.0
        self.robot_connected = False
        self._robot_status_codes = {"robot1": None, "robot2": None}
        # Orders may arrive while the sequential multi-robot startup is still
        # bringing up robot2 and the fleet manager.  Keep them pending instead
        # of turning a temporary service outage / busy fleet into CANCELLED.
        self._pending_manager_orders = {}
        self._manager_order_inflight = set()
        self._manager_order_retry_after = {}
        self._robot_status_labels = {
            0: "IDLE",
            1: "RETURNING TO KITCHEN",
            2: "PREPARING FOOD (SPAWNING)",
            3: "NAVIGATING TO TABLE",
            4: "ARM SERVING FOOD",
            5: "PAUSED (SAFETY HAND INTRUSION)",
            6: "COMPLETED",
            7: "SYSTEM FAILED (RESET REQUIRED)",
        }
        
        # Drive mode: 'MOCK' (standalone virtual simulator) vs 'LIVE' (Isaac Sim / Real Robot ROS 2)
        self.drive_mode = "MOCK"
        
        self.robot_pose = {"x": -1.82, "y": -2.20, "yaw": 0.0}
        self.robot_state = "IDLE"  # IDLE, NAVIGATING, SERVING, ERROR
        self.parking_brake = True
        self.battery_level = 98.0
        self.arm_state = "HOME"
        self.obstacle_info = {"active": False, "x": 0.0, "y": 2.8, "stop": False}
        
        # Publishers
        self.order_pub = self.create_publisher(String, '/serving_robot/order', 10)
        self.estop_pub = self.create_publisher(Bool, '/serving_robot/emergency_stop', 10)

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        # ROS 2 Subscribers
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            qos_profile_sensor_data
        )
        
        # ROS 2 Status Subscribers (Dual QoS for maximum compatibility with Isaac Sim & ROS nodes)
        self.robot_status_sub = self.create_subscription(
            String,
            '/serving_robot/status',
            self.robot_status_callback,
            10
        )
        self.robot_status_sensor_sub = self.create_subscription(
            String,
            '/serving_robot/status',
            self.robot_status_callback,
            qos_profile_sensor_data
        )

        # Direct Event Topic Subscriber (/serving_robot/event)
        self.event_sub = self.create_subscription(
            String,
            '/serving_robot/event',
            self.robot_status_callback,
            10
        )

        # Obstacle Event Subscriber (/serving_robot/obstacle_event)
        self.obstacle_event_sub = self.create_subscription(
            String,
            '/serving_robot/obstacle_event',
            self.obstacle_event_callback,
            10
        )

        # Table Camera Subscription
        from sensor_msgs.msg import Image
        self.last_camera_base64 = ""
        self.last_camera_recv_time = 0.0
        self.last_camera_error_time = 0.0
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.table_camera_callback,
            qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        # Serving Robot Manager Integration Clients & Subscribers
        self.system_status_sub = self.create_subscription(
            Int32,
            '/system/status',
            self.system_status_callback,
            status_qos
        )
        for robot_name in ("robot1", "robot2"):
            self.create_subscription(
                Int32,
                f'/{robot_name}/system/status',
                lambda msg, name=robot_name: self._on_robot_fleet_status(name, msg),
                status_qos,
            )
        self.order_cancelled_sub = self.create_subscription(
            Int32,
            '/manager/order_cancelled',
            self.order_cancelled_callback,
            10,
        )
        self.manager_reset_client = self.create_client(Trigger, '/manager/reset_fault')
        self.hand_test_client = self.create_client(SetBool, '/hand_test/set_visible')
        self.obstacle_test_client = self.create_client(SetBool, '/obstacle_test/set_visible')
        self.typing_trigger_pub = self.create_publisher(
            Empty,
            '/hand_test/type_keyboard',
            10,
        )
        if ORDER_REQUEST_SRV_AVAILABLE:
            self.manager_order_client = self.create_client(OrderRequest, '/manager/order')
            self.get_logger().info("Manager /manager/order Service Client created.")
        else:
            self.get_logger().error(
                f"OrderRequest interface import failed: {ORDER_REQUEST_IMPORT_ERROR}")

        # Periodic timer for checking liveness and driving mock navigation (20 Hz)
        self.create_timer(0.05, self.check_liveness_timer)
        self.get_logger().info(
            "HMI ROS 2 Bridge Node initialized: "
            f"camera={self.camera_topic}, odom={self.odom_topic}"
        )

    def set_hand_test_visible(self, visible: bool):
        if not hasattr(self, 'hand_test_client'):
            return False, "hand_test_client not initialized"
        if not self.hand_test_client.service_is_ready():
            return False, "/hand_test/set_visible service is not ready"
        req = SetBool.Request()
        req.data = visible
        self.hand_test_client.call_async(req)
        return True, f"hand spawn visible={visible} request queued"

    def set_obstacle_test_visible(self, visible: bool):
        if not hasattr(self, 'obstacle_test_client'):
            return False, "obstacle_test_client not initialized"
        if not self.obstacle_test_client.service_is_ready():
            return False, "/obstacle_test/set_visible service is not ready"
        req = SetBool.Request()
        req.data = visible
        self.obstacle_test_client.call_async(req)
        return True, f"corridor person spawn visible={visible} request queued"

    def trigger_typing_animation(self):
        if not hasattr(self, 'typing_trigger_pub'):
            return False, "typing_trigger_pub not initialized"
        self.typing_trigger_pub.publish(Empty())
        return True, "typing animation trigger published"

    def table_camera_callback(self, msg):
        try:
            import cv2
            import numpy as np
            import base64

            height = msg.height
            width = msg.width
            encoding = msg.encoding.lower()
            channels = {
                'mono8': 1,
                'rgb8': 3,
                'bgr8': 3,
                'rgba8': 4,
                'bgra8': 4,
            }.get(encoding)
            if channels is None:
                raise ValueError(f"unsupported image encoding: {msg.encoding}")

            row_bytes = int(msg.step) if msg.step else width * channels
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, row_bytes)
            pixels = rows[:, :width * channels]
            if channels == 1:
                img = pixels.reshape(height, width)
            else:
                img = pixels.reshape(height, width, channels)
            if encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif encoding == 'rgba8':
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            elif encoding == 'bgra8':
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

            ok, buffer = cv2.imencode(
                '.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            b64_str = base64.b64encode(buffer).decode('utf-8')
            self.last_camera_base64 = f"data:image/jpeg;base64,{b64_str}"
            self.last_camera_recv_time = time.time()
        except Exception as e:
            now = time.time()
            if now - self.last_camera_error_time >= 5.0:
                self.last_camera_error_time = now
                self.get_logger().warning(f"Camera frame conversion failed: {e}")

    def clock_callback(self, msg: Clock):
        now_wall = time.time()
        sim_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        
        if self.last_clock_recv_wall_time > 0:
            dt_wall = now_wall - self.last_clock_recv_wall_time
            if dt_wall > 0:
                self.clock_hz = round(1.0 / dt_wall, 1)
        
        self.last_clock_time = sim_sec
        self.last_clock_recv_wall_time = now_wall
        self.isaac_sim_connected = True

    def odom_callback(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z)
        import math
        self.robot_pose = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": math.atan2(siny_cosp, cosy_cosp),
        }
        self.last_robot_status_recv_time = time.time()
        self.robot_connected = True
    def obstacle_event_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            active = bool(data.get("active", data.get("detected", False)))
            x_val = data.get("x")
            y_val = data.get("y")
            self.obstacle_info = {
                "active": active,
                "x": float(x_val) if (x_val is not None and active) else 0.0,
                "y": float(y_val) if (y_val is not None and active) else 2.8,
                "stop": active,
            }
            if active:
                self.robot_state = "🛑 OBSTACLE DETECTED (STOPPED)"
        except Exception:
            pass

    def robot_status_callback(self, msg: String):
        self.last_robot_status_recv_time = time.time()
        self.robot_connected = True
        try:
            raw_text = msg.data.strip()
            print(f"\n📥 [HMI RECEIVED ROS 2 EVENT] -> {raw_text}", flush=True)

            # Support plain text events as well as JSON
            event_signal = None
            data = {}
            if raw_text.startswith('{'):
                data = json.loads(raw_text)
                event_signal = data.get("event") or data.get("action_completed")
            else:
                event_signal = raw_text

            # Automatically update robot_pose and force LIVE mode when real/Isaac Sim pose is received
            if "pose" in data:
                self.robot_pose = data["pose"]
                if self.drive_mode != "LIVE":
                    self.drive_mode = "LIVE"
                    print("📡 [HMI Backend] Isaac Sim Robot Pose Received -> DRIVE MODE AUTO SWITCHED TO LIVE!", flush=True)

            if "state" in data:
                self.robot_state = data["state"]
            if "parking_brake" in data:
                self.parking_brake = data["parking_brake"]
            if "battery" in data:
                self.battery_level = data["battery"]

            active_id = order_manager.active_order_id
            if event_signal and active_id:
                active_order = order_manager.orders.get(active_id)
                if active_order:
                    if event_signal in ["ORDER_ACCEPTED", "START_KITCHEN_PICKUP"]:
                        print(f"🚀 [EVENT 1 ACCEPTED] Driving to Kitchen for Order {active_id}...", flush=True)
                        order_manager.update_status(active_id, OrderStatus.PICKING_UP)
                        self.robot_state = "NAVIGATING TO KITCHEN"
                        self.parking_brake = False

                    elif event_signal in ["PICKUP_COMPLETED", "START_TABLE_NAV"]:
                        print(f"🍕 [EVENT 2 PICKUP COMPLETED] Driving to Table {active_order.table_number}...", flush=True)
                        order_manager.update_status(active_id, OrderStatus.NAVIGATING)
                        self.robot_state = f"NAVIGATING TO T{active_order.table_number}"
                        self.parking_brake = False

                    elif event_signal in ["SERVING_COMPLETED", "COMPLETED"]:
                        print(f"🎉 [EVENT 3 SERVING COMPLETED] Mission Done.", flush=True)
                        order_manager.update_status(active_id, OrderStatus.COMPLETED)
                        self.robot_state = "IDLE (MISSION COMPLETED)"
                        self.parking_brake = True
        except Exception as e:
            self.get_logger().warn(f"Failed to parse robot status msg: {e}")

    def _on_robot_fleet_status(self, robot_name: str, msg: Int32):
        code = int(msg.data)
        self._robot_status_codes[robot_name] = code
        self.robot_connected = True
        self.last_robot_status_recv_time = time.time()
        if self.drive_mode != "LIVE":
            self.drive_mode = "LIVE"
            print(
                f"📡 [HMI Backend] {robot_name} status={code} -> DRIVE MODE LIVE",
                flush=True,
            )
        order = order_manager.active_order_for_robot(robot_name)
        if order is not None:
            self._apply_order_status_code(order.order_id, code)

    def _apply_order_status_code(self, order_id: str, status_code: int) -> None:
        """Apply one worker's status only to the order assigned to it."""
        if status_code == 1:
            # Returning is still part of the current delivery lifecycle.
            order_manager.update_status(order_id, OrderStatus.SERVING)
            self.parking_brake = False
        elif status_code == 2:
            order_manager.update_status(order_id, OrderStatus.PICKING_UP)
            self.parking_brake = True
        elif status_code == 3:
            order_manager.update_status(order_id, OrderStatus.NAVIGATING)
            self.parking_brake = False
        elif status_code in (4, 5):
            order_manager.update_status(order_id, OrderStatus.SERVING)
            self.parking_brake = True
        elif status_code == 6:
            order_manager.update_status(order_id, OrderStatus.COMPLETED)
            self.parking_brake = True
        elif status_code == 7:
            order_manager.update_status(order_id, OrderStatus.CANCELLED)
            self.parking_brake = True

    def fleet_robot_snapshot(self) -> dict:
        snapshot = {}
        for name in ("robot1", "robot2"):
            code = self._robot_status_codes.get(name)
            if code is None:
                snapshot[name] = {
                    "state": "UNKNOWN",
                    "state_code": None,
                    "available": False,
                }
                continue
            available = code in (0, 6)  # IDLE, COMPLETED
            snapshot[name] = {
                "state": self._robot_status_labels.get(code, f"STATE_{code}"),
                "state_code": code,
                "available": available,
            }
        return snapshot

    def system_status_callback(self, msg: Int32):
        status_code = msg.data
        status_map = self._robot_status_labels
        self.robot_state = status_map.get(status_code, f"STATE_{status_code}")
        self.robot_connected = True
        self.last_robot_status_recv_time = time.time()

        if self.drive_mode != "LIVE":
            self.drive_mode = "LIVE"
            print(f"📡 [HMI Backend] Manager Status Received ({self.robot_state}) -> DRIVE MODE AUTO SWITCHED TO LIVE!", flush=True)

        # /system/status is an aggregate fleet status and cannot identify the
        # owning order. Order progression is handled by the namespaced worker
        # callbacks above, otherwise robot1 can overwrite robot2's order.

    def order_cancelled_callback(self, msg: Int32):
        hmi_table_number = int(msg.data) + 1
        for order in order_manager.orders.values():
            if (order.table_number == hmi_table_number
                    and order.status not in (OrderStatus.COMPLETED,
                                             OrderStatus.CANCELLED)):
                order_manager.update_status(order.order_id, OrderStatus.CANCELLED)
                self.get_logger().warning(
                    f"Manager cancelled queued HMI order {order.order_id}")
                return

    def send_order_to_manager(
        self, order_id: str, table_num: int, items: list, preferred_robot: str = ""
    ):
        if not ORDER_REQUEST_SRV_AVAILABLE or not hasattr(self, 'manager_order_client'):
            return False, "OrderRequest srv not loaded"

        pizza1, pizza2, pizza3, drink, cutlery, plate = 0, 0, 0, 0, 0, 0
        for item in items:
            menu_id = str(item.get("menu_id", "")).strip().lower()
            name = item.get("name", "").lower()
            qty = int(item.get("quantity", 1))
            if qty <= 0:
                continue
            if menu_id == "m1" or "pizza 1" in name or "supreme" in name or name == "pizza1":
                pizza1 += qty
            elif menu_id == "m2" or "pizza 2" in name or "pepperoni" in name or name == "pizza2":
                pizza2 += qty
            elif menu_id == "m3" or "pizza 3" in name or "cheese" in name or name == "pizza3" or "pizza" in name:
                pizza3 += qty
            elif menu_id == "m4" or "soda" in name or "drink" in name or "beverage" in name:
                drink += qty
            elif menu_id == "m5" or "cutlery" in name or "fork" in name or "spoon" in name:
                cutlery += qty
            elif (menu_id == "m6" or "plate rack" in name
                  or "접시 트레이" in name or name == "접시"):
                plate += qty

        if not any((pizza1, pizza2, pizza3, drink, cutlery, plate)):
            return False, "No supported menu items in order"

        hmi_table_number = int(table_num)
        if not 1 <= hmi_table_number <= 4:
            return False, f"Invalid HMI table number: {hmi_table_number}"

        preferred = str(preferred_robot or "").strip().lower()
        if preferred in ("", "auto", "any"):
            preferred = ""
        elif preferred not in ("robot1", "robot2"):
            return False, f"Invalid preferred_robot: {preferred_robot}"

        # The web UI labels tables 1..4, while manager/axis routes use the
        # zero-based IDs 0..3. Keep the user-facing number unchanged and only
        # translate at the ROS service boundary.
        manager_table_id = hmi_table_number - 1
        req = OrderRequest.Request()
        req.table_id = manager_table_id
        req.pizza1_count = pizza1
        req.pizza2_count = pizza2
        req.pizza3_count = pizza3
        req.drink_count = drink
        req.cutlery_count = cutlery
        req.plate_count = plate
        req.preferred_robot = preferred

        self._pending_manager_orders[order_id] = req
        self._manager_order_retry_after[order_id] = 0.0
        if not self.manager_order_client.service_is_ready():
            self.get_logger().warning(
                f"HMI order {order_id} queued: /manager/order is not ready yet"
            )
            return True, "Order queued until fleet manager is ready"

        self._dispatch_pending_manager_order(order_id)
        return True, "Order dispatched to Manager via /manager/order"

    def _dispatch_pending_manager_order(self, order_id: str) -> None:
        req = self._pending_manager_orders.get(order_id)
        if req is None or order_id in self._manager_order_inflight:
            return
        if not self.manager_order_client.service_is_ready():
            return

        print(
            "🚀 [HMI Backend] Sending OrderRequest to Manager: "
            f"order={order_id}, route_id={req.table_id}, "
            f"preferred={req.preferred_robot or 'auto'}, "
            f"P1={req.pizza1_count}, P2={req.pizza2_count}, "
            f"P3={req.pizza3_count}, Drink={req.drink_count}, "
            f"Cutlery={req.cutlery_count}, Plate={req.plate_count}",
            flush=True,
        )
        self._manager_order_inflight.add(order_id)
        future = self.manager_order_client.call_async(req)
        future.add_done_callback(
            lambda completed, oid=order_id: self._on_manager_order_response(oid, completed)
        )

    def _on_manager_order_response(self, order_id, future):
        self._manager_order_inflight.discard(order_id)
        try:
            response = future.result()
            if response is not None and response.success:
                assigned = getattr(response, "assigned_robot", "") or ""
                self._pending_manager_orders.pop(order_id, None)
                self._manager_order_retry_after.pop(order_id, None)
                order_manager.assign_robot(order_id, assigned)
                # Acceptance itself means the order is no longer pending. Use
                # the latest latched worker state to catch status messages that
                # arrived just before the service response.
                order_manager.update_status(order_id, OrderStatus.PICKING_UP)
                worker_code = self._robot_status_codes.get(assigned)
                if worker_code is not None:
                    self._apply_order_status_code(order_id, int(worker_code))
                self.get_logger().info(
                    f"Manager accepted HMI order {order_id}"
                    + (f" -> {assigned}" if assigned else "")
                )
                return
            reason = "manager rejected the order"
        except Exception as exc:
            reason = f"service call failed: {exc}"

        # Rejection commonly means both workers are temporarily occupied.
        # Preserve the order and retry instead of reporting a false cancel.
        self._manager_order_retry_after[order_id] = time.monotonic() + 1.0
        self.get_logger().warning(
            f"HMI order {order_id} remains pending: {reason}; retrying"
        )

    def reset_manager_fault(self):
        if hasattr(self, 'manager_reset_client') and self.manager_reset_client.service_is_ready():
            req = Trigger.Request()
            self.manager_reset_client.call_async(req)
            print("🚀 [HMI Backend] Triggered /manager/reset_fault service call", flush=True)

    def check_liveness_timer(self):
        now = time.time()
        # Isaac Sim clock liveness check
        if now - self.last_clock_recv_wall_time > 2.0:
            self.isaac_sim_connected = False
            self.clock_hz = 0.0
            
        # Robot status liveness check
        if now - self.last_robot_status_recv_time > 3.0:
            self.robot_connected = False

        if (hasattr(self, 'manager_order_client')
                and self.manager_order_client.service_is_ready()):
            monotonic_now = time.monotonic()
            for order_id in list(self._pending_manager_orders):
                retry_after = self._manager_order_retry_after.get(order_id, 0.0)
                if monotonic_now >= retry_after:
                    self._dispatch_pending_manager_order(order_id)
            
        # Run Mock Navigation Engine unconditionally when drive_mode == 'MOCK'
        if self.drive_mode == "MOCK" and order_manager.active_order_id:
            self.update_mock_navigation()

    def update_mock_navigation(self):
        active_id = order_manager.active_order_id
        if not active_id:
            return
            
        active_order = order_manager.orders.get(active_id)
        if not active_order or active_order.status in [OrderStatus.COMPLETED, OrderStatus.CANCELLED]:
            return

        table_coords = {
            1: (-3.2, -2.2),
            2: (3.2, -2.2),
            3: (-3.2, 0.7),
            4: (3.2, 0.7)
        }

        target_table_xy = table_coords.get(active_order.table_number, (-3.2, -2.2))
        kitchen_xy = (0.0, 4.0)

        # Helper to step robot pose towards target coordinate (20Hz)
        def step_towards(tx, ty, speed=0.12):
            cx, cy = self.robot_pose["x"], self.robot_pose["y"]
            dx, dy = tx - cx, ty - cy
            dist = (dx**2 + dy**2)**0.5
            if dist < 0.25:
                return True
            import math
            yaw = math.atan2(dy, dx)
            self.robot_pose["x"] += (dx / dist) * speed
            self.robot_pose["y"] += (dy / dist) * speed
            self.robot_pose["yaw"] = yaw
            return False

        # Phase 0: PENDING -> Instantly start drive to Kitchen
        if active_order.status == OrderStatus.PENDING:
            self._waiting_pickup = False
            self._waiting_serve = False
            order_manager.update_status(active_id, OrderStatus.PICKING_UP)
            self.robot_state = "NAVIGATING TO KITCHEN"
            self.parking_brake = False
            print(f"🤖 [Auto Mock Nav] Order {active_id} received! Driving to Kitchen...", flush=True)

        # Phase 1: PICKING_UP -> Drive to Kitchen then auto-wait 2s & switch to NAVIGATING
        elif active_order.status == OrderStatus.PICKING_UP:
            arrived = step_towards(kitchen_xy[0], kitchen_xy[1])
            if arrived:
                self.robot_state = "AT KITCHEN (PICKING UP FOOD)"
                self.parking_brake = True
                if not getattr(self, "_waiting_pickup", False):
                    self._waiting_pickup = True
                    print(f"🤖 [Auto Mock Nav] Arrived at Kitchen. Loading food (2s)...", flush=True)
                    threading.Thread(target=self._finish_pickup_and_navigate, args=(active_id,), daemon=True).start()
            else:
                self.robot_state = "NAVIGATING TO KITCHEN"
                self.parking_brake = False

        # Phase 2: NAVIGATING -> Drive to Target Table then auto-wait 3s & COMPLETED
        elif active_order.status == OrderStatus.NAVIGATING:
            arrived = step_towards(target_table_xy[0], target_table_xy[1])
            if arrived:
                self.robot_state = f"AT TABLE {active_order.table_number} (SERVING FOOD)"
                self.parking_brake = True
                order_manager.update_status(active_id, OrderStatus.SERVING)
                if not getattr(self, "_waiting_serve", False):
                    self._waiting_serve = True
                    print(f"🤖 [Auto Mock Nav] Arrived at Table {active_order.table_number}. Serving (3s)...", flush=True)
                    threading.Thread(target=self._auto_complete_order, args=(active_id,), daemon=True).start()
            else:
                self.robot_state = f"NAVIGATING TO T{active_order.table_number}"
                self.parking_brake = False

    def _finish_pickup_and_navigate(self, order_id):
        time.sleep(2.0)
        order = order_manager.orders.get(order_id)
        if order and order.status == OrderStatus.PICKING_UP:
            order_manager.update_status(order_id, OrderStatus.NAVIGATING)
            self.parking_brake = False
            print(f"🤖 [Auto Mock Nav] Pickup Done! Driving to Table...", flush=True)

    def _auto_complete_order(self, order_id):
        time.sleep(3.0)
        order = order_manager.orders.get(order_id)
        if order and order.status in [OrderStatus.NAVIGATING, OrderStatus.SERVING]:
            order_manager.update_status(order_id, OrderStatus.COMPLETED)
            self.robot_state = "IDLE (MISSION COMPLETED)"
            self.parking_brake = True
            print(f"🎉 [Auto Mock Nav] Mission {order_id} Completed!", flush=True)
        if order and order.status == OrderStatus.SERVING:
            order_manager.update_status(order_id, OrderStatus.COMPLETED)
            self.robot_state = "IDLE (SERVING DONE)"
            self.parking_brake = True
            self.get_logger().info(f"[Mock Nav] Order {order_id} completed!")

    def publish_order(self, order_dict: dict):
        msg = String()
        msg.data = json.dumps(order_dict)
        self.order_pub.publish(msg)
        self.get_logger().info(f"Published Order to ROS 2: {order_dict['order_id']} for Table {order_dict['table_number']}")

    def publish_estop(self, stop: bool):
        msg = Bool()
        msg.data = stop
        self.estop_pub.publish(msg)
        self.get_logger().info(f"Published Emergency Stop: {stop}")

ros_node: HMIBridgeNode = None

def get_system_status_payload():
    is_mock = (ros_node.drive_mode == "MOCK") if ros_node else True
    camera_connected = bool(
        ros_node
        and ros_node.last_camera_recv_time > 0.0
        and time.time() - ros_node.last_camera_recv_time <= 2.0
    )
    robots = (
        ros_node.fleet_robot_snapshot()
        if ros_node
        else {
            "robot1": {"state": "UNKNOWN", "state_code": None, "available": False},
            "robot2": {"state": "UNKNOWN", "state_code": None, "available": False},
        }
    )
    return {
        "type": "SYSTEM_STATUS",
        "timestamp": time.time(),
        "drive_mode": ros_node.drive_mode if ros_node else "MOCK",
        "isaac_sim": {
            "connected": ros_node.isaac_sim_connected if ros_node else False,
            "hz": ros_node.clock_hz if ros_node else 0.0,
            "clock": ros_node.last_clock_time if ros_node else 0.0
        },
        "robot": {
            "connected": True if is_mock else (ros_node.robot_connected if ros_node else False),
            "state": ros_node.robot_state if ros_node else "OFFLINE",
            "pose": ros_node.robot_pose if ros_node else {"x": -1.82, "y": -2.20, "yaw": 0.0},
            "parking_brake": ros_node.parking_brake if ros_node else True,
            "battery": ros_node.battery_level if ros_node else 100.0,
            "domain_id": os.environ.get("ROS_DOMAIN_ID", "0")
        },
        "robots": robots,
        "camera_connected": camera_connected,
        "obstacle": (ros_node.obstacle_info if ros_node else {"active": False, "x": 0.0, "y": 2.8, "stop": False}),
        "camera_image": (
            ros_node.last_camera_base64 if camera_connected else ""),
        "orders": order_manager.get_all_orders_dict(),
        "active_order_id": order_manager.active_order_id
    }

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(broadcast_loop())
    print("[HMI Backend] WebSocket Broadcast Loop successfully started on Uvicorn Loop.", flush=True)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    try:
        # Send initial full state immediately
        await websocket.send_json(get_system_status_payload())
        
        while True:
            data_text = await websocket.receive_text()
            try:
                data = json.loads(data_text)
                msg_type = data.get("type")
                
                if msg_type == "SET_DRIVE_MODE":
                    mode = data.get("mode", "MOCK")
                    if ros_node:
                        ros_node.drive_mode = mode
                        print(f"[HMI Backend] DRIVE MODE SWITCHED TO: {mode}", flush=True)
                        if mode == "MOCK":
                            ros_node.update_mock_navigation()
                    payload = get_system_status_payload()
                    for client in list(connected_clients):
                        try:
                            await client.send_json(payload)
                        except Exception:
                            pass

                elif msg_type == "CREATE_ORDER":
                    table_num = int(data.get("table_number", 1))
                    items = data.get("items", [])
                    preferred_robot = str(data.get("preferred_robot", "auto") or "auto")
                    new_order = order_manager.create_order(table_num, items)
                    print(
                        f"[HMI Backend] NEW ORDER CREATED: {new_order.order_id} "
                        f"for Table {table_num} robot={preferred_robot}. "
                        f"Total items: {len(items)}"
                    )
                    
                    # Publish order to ROS 2 topic with quantities and total price
                    order_payload = {
                        "action": "NEW_ORDER",
                        "order_id": new_order.order_id,
                        "table_number": new_order.table_number,
                        "preferred_robot": preferred_robot,
                        "items": [f"{item.name} x{item.quantity}" for item in new_order.items],
                        "total_price": new_order.total_price,
                        "status": new_order.status
                    }
                    if ros_node:
                        if ros_node.drive_mode == "LIVE":
                            sent, msg = ros_node.send_order_to_manager(
                                new_order.order_id, table_num, items, preferred_robot)
                            if not sent:
                                order_manager.update_status(
                                    new_order.order_id, OrderStatus.CANCELLED)
                                ros_node.get_logger().error(
                                    f"HMI order {new_order.order_id} cancelled: {msg}")
                        else:
                            ros_node.publish_order(order_payload)
                            ros_node.update_mock_navigation()

                    # Immediate broadcast to all clients
                    payload = get_system_status_payload()
                    for client in list(connected_clients):
                        try:
                            await client.send_json(payload)
                        except Exception:
                            pass
                        
                elif msg_type == "UPDATE_ORDER_STATUS":
                    order_id = data.get("order_id")
                    status = data.get("status")
                    if order_id and status:
                        order_manager.update_status(order_id, status)
                        if ros_node:
                            ros_node.publish_order({
                                "action": "UPDATE_STATUS",
                                "order_id": order_id,
                                "status": status
                            })
                        payload = get_system_status_payload()
                        for client in list(connected_clients):
                            try:
                                await client.send_json(payload)
                            except Exception:
                                pass
                            
                elif msg_type == "EMERGENCY_STOP":
                    stop_flag = bool(data.get("stop", True))
                    if ros_node:
                        ros_node.publish_estop(stop_flag)

                elif msg_type == "TRIGGER_TYPING":
                    if ros_node:
                        sent, msg = ros_node.trigger_typing_animation()
                        print(f"[HMI Backend] TRIGGER TYPING: {msg}", flush=True)

                elif msg_type == "SET_HAND_TEST_VISIBLE":
                    # Legacy compat: hand-only USD test is retired. visible=True
                    # now triggers the typing animation; visible=False is a
                    # no-op since there is no hand-only prim left to remove.
                    visible = bool(data.get("visible", False))
                    if ros_node:
                        if visible:
                            sent, msg = ros_node.trigger_typing_animation()
                        else:
                            sent, msg = True, "hand remove is obsolete; no action required"
                        print(f"[HMI Backend] SET HAND TEST VISIBLE ({visible}): {msg}", flush=True)

                elif msg_type == "SET_OBSTACLE_TEST_VISIBLE":
                    visible = bool(data.get("visible", False))
                    if ros_node:
                        sent, msg = ros_node.set_obstacle_test_visible(visible)
                        print(f"[HMI Backend] SET OBSTACLE TEST VISIBLE ({visible}): {msg}", flush=True)

                elif msg_type == "RESET_FAULT":
                    if ros_node:
                        ros_node.reset_manager_fault()
                        
            except Exception as ex:
                print(f"Error handling WS message: {ex}")
                
    except WebSocketDisconnect:
        connected_clients.remove(websocket)
    except Exception as e:
        if websocket in connected_clients:
            connected_clients.remove(websocket)

async def broadcast_loop():
    while True:
        if connected_clients:
            payload = get_system_status_payload()
            disconnected = set()
            for client in list(connected_clients):
                try:
                    await client.send_json(payload)
                except Exception:
                    disconnected.add(client)
            for d in disconnected:
                connected_clients.discard(d)
        await asyncio.sleep(0.15)

# Setup static files directory
script_dir = os.path.dirname(os.path.abspath(__file__))
try:
    from ament_index_python.packages import get_package_share_directory

    package_share_web_ui = os.path.join(
        get_package_share_directory("serving_hmi"), "web_ui"
    )
except Exception:
    package_share_web_ui = ""

candidate_paths = [
    package_share_web_ui,
    os.path.abspath(os.path.join(script_dir, "..", "web_ui")),
    os.path.abspath(os.path.join(
        script_dir, "..", "..", "..", "..", "..", "..", "serving_robot",
        "src", "serving_hmi", "web_ui"
    )),
]
candidate_paths = [path for path in candidate_paths if path]

web_ui_dir = None
for candidate in candidate_paths:
    if os.path.exists(os.path.join(candidate, "index.html")):
        web_ui_dir = candidate
        break

if not web_ui_dir:
    web_ui_dir = candidate_paths[0]

if os.path.exists(web_ui_dir):
    app.mount("/static", StaticFiles(directory=web_ui_dir), name="static")

@app.get("/")
async def get_index():
    if web_ui_dir:
        index_file = os.path.join(web_ui_dir, "index.html")
        if os.path.exists(index_file):
            return FileResponse(
                index_file,
                headers={"Cache-Control": "no-cache, no-store, must-revalidate"},
            )
    return {"error": "HMI Frontend index.html not found", "searched_paths": candidate_paths}

@app.get("/css/{file_name}")
async def get_css(file_name: str):
    if web_ui_dir:
        file_path = os.path.join(web_ui_dir, "css", file_name)
        if os.path.exists(file_path):
            return FileResponse(file_path, media_type="text/css", headers={"Cache-Control": "no-cache, no-store, must-revalidate"})
    return {"error": f"css {file_name} not found"}

@app.get("/js/{file_name}")
async def get_js(file_name: str):
    if web_ui_dir:
        file_path = os.path.join(web_ui_dir, "js", file_name)
        if os.path.exists(file_path):
            return FileResponse(file_path, media_type="application/javascript", headers={"Cache-Control": "no-cache, no-store, must-revalidate"})
    return {"error": f"js {file_name} not found"}

def ros_spin_thread():
    rclpy.spin(ros_node)

def main():
    global ros_node
    rclpy.init()
    ros_node = HMIBridgeNode()
    
    # Run ROS spin in background thread
    spin_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    spin_thread.start()
    
    port = int(os.environ.get("HMI_PORT", 8000))
    print(f"Starting Serving HMI Web Dashboard on http://0.0.0.0:{port} (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')})")
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")

if __name__ == '__main__':
    main()
