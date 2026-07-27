import time
from typing import Dict, List, Optional
from dataclasses import dataclass, field, asdict

ROBOT_NAMES = ("robot1", "robot2")

class OrderStatus:
    PENDING = "PENDING"          # 주문 접수 / 주방 수거 대기
    PICKING_UP = "PICKING_UP"    # 주방 음식 수거 중
    NAVIGATING = "NAVIGATING"    # 해당 테이블로 자율 이동 중
    SERVING = "SERVING"          # 테이블 도킹 & 음식 세팅/서빙 중
    COMPLETED = "COMPLETED"      # 서빙 완료 후 복귀
    CANCELLED = "CANCELLED"      # 주문 취소

ACTIVE_STATUSES = (
    OrderStatus.PENDING,
    OrderStatus.PICKING_UP,
    OrderStatus.NAVIGATING,
    OrderStatus.SERVING,
)

@dataclass
class OrderItem:
    menu_id: str
    name: str
    quantity: int
    price: int

@dataclass
class Order:
    order_id: str
    table_number: int
    items: List[OrderItem]
    total_price: int
    assigned_robot: str = "robot1"
    status: str = OrderStatus.PENDING
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)

class OrderManager:
    def __init__(self):
        self.orders: Dict[str, Order] = {}
        # One active order per robot -- two robots can each be mid-mission
        # at the same time, unlike the old single-robot active_order_id.
        self.active_order_ids: Dict[str, Optional[str]] = {name: None for name in ROBOT_NAMES}
        self._counter = 100
        self._assign_counter = 0

    def create_order(self, table_number: int, items_data: List[dict], robot: Optional[str] = None) -> Order:
        self._counter += 1
        order_id = f"ORD-{self._counter}"

        items = []
        total = 0
        for item in items_data:
            order_item = OrderItem(
                menu_id=item.get("menu_id", "m1"),
                name=item.get("name", "피자"),
                quantity=int(item.get("quantity", 1)),
                price=int(item.get("price", 15000))
            )
            items.append(order_item)
            total += order_item.price * order_item.quantity

        if robot not in ROBOT_NAMES:
            # No robot picked the order explicitly (customer never chooses
            # one) -- round-robin between the two serving robots.
            robot = ROBOT_NAMES[self._assign_counter % len(ROBOT_NAMES)]
            self._assign_counter += 1

        order = Order(
            order_id=order_id,
            table_number=table_number,
            items=items,
            total_price=total,
            assigned_robot=robot,
        )
        self.orders[order_id] = order

        current_active = self.active_order_ids.get(robot)
        current_active_order = self.orders.get(current_active) if current_active else None
        if not current_active_order or current_active_order.status in (OrderStatus.COMPLETED, OrderStatus.CANCELLED):
            self.active_order_ids[robot] = order_id
        return order

    def update_status(self, order_id: str, new_status: str) -> bool:
        if order_id in self.orders:
            order = self.orders[order_id]
            order.status = new_status
            order.updated_at = time.time()
            if new_status in (OrderStatus.COMPLETED, OrderStatus.CANCELLED):
                robot = order.assigned_robot
                if self.active_order_ids.get(robot) == order_id:
                    self.active_order_ids[robot] = self._get_next_pending_id(robot)
            return True
        return False

    def _get_next_pending_id(self, robot: str) -> Optional[str]:
        for oid, order in self.orders.items():
            if order.assigned_robot == robot and order.status in ACTIVE_STATUSES:
                return oid
        return None

    def get_all_orders_dict(self) -> List[dict]:
        res = []
        for oid, order in self.orders.items():
            res.append({
                "order_id": order.order_id,
                "table_number": order.table_number,
                "assigned_robot": order.assigned_robot,
                "items": [{"name": item.name, "quantity": item.quantity, "price": item.price} for item in order.items],
                "total_price": order.total_price,
                "status": order.status,
                "created_at": order.created_at,
                "updated_at": order.updated_at
            })
        return res
