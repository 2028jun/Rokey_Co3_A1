import time
from typing import Dict, List, Optional
from dataclasses import dataclass, field, asdict

class OrderStatus:
    PENDING = "PENDING"          # 주문 접수 / 주방 수거 대기
    PICKING_UP = "PICKING_UP"    # 주방 음식 수거 중
    NAVIGATING = "NAVIGATING"    # 해당 테이블로 자율 이동 중
    SERVING = "SERVING"          # 테이블 도킹 & 음식 세팅/서빙 중
    COMPLETED = "COMPLETED"      # 서빙 완료 후 복귀
    CANCELLED = "CANCELLED"      # 주문 취소

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
    status: str = OrderStatus.PENDING
    assigned_robot: str = ""
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)

class OrderManager:
    def __init__(self):
        self.orders: Dict[str, Order] = {}
        self.active_order_id: Optional[str] = None
        self._counter = 100

    def create_order(self, table_number: int, items_data: List[dict]) -> Order:
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

        order = Order(
            order_id=order_id,
            table_number=table_number,
            items=items,
            total_price=total
        )
        self.orders[order_id] = order
        # Update active_order_id if empty or previous active order is finished
        if not self.active_order_id or self.orders.get(
            self.active_order_id, Order("", 0, [], 0)
        ).status in [OrderStatus.COMPLETED, OrderStatus.CANCELLED]:
            self.active_order_id = order_id
        return order

    def update_status(self, order_id: str, new_status: str) -> bool:
        if order_id in self.orders:
            self.orders[order_id].status = new_status
            self.orders[order_id].updated_at = time.time()
            if new_status in [OrderStatus.COMPLETED, OrderStatus.CANCELLED]:
                if self.active_order_id == order_id:
                    self.active_order_id = self._get_next_pending_id()
            return True
        return False

    def assign_robot(self, order_id: str, robot_name: str) -> bool:
        if order_id not in self.orders:
            return False
        self.orders[order_id].assigned_robot = str(robot_name or "")
        self.orders[order_id].updated_at = time.time()
        return True

    def active_order_for_robot(self, robot_name: str) -> Optional[Order]:
        for order in self.orders.values():
            if (order.assigned_robot == robot_name
                    and order.status not in (OrderStatus.COMPLETED,
                                             OrderStatus.CANCELLED)):
                return order
        return None

    def _get_next_pending_id(self) -> Optional[str]:
        for oid, order in self.orders.items():
            if order.status in [OrderStatus.PENDING, OrderStatus.PICKING_UP, OrderStatus.NAVIGATING, OrderStatus.SERVING]:
                return oid
        return None

    def get_all_orders_dict(self) -> List[dict]:
        res = []
        for oid, order in self.orders.items():
            res.append({
                "order_id": order.order_id,
                "table_number": order.table_number,
                "items": [{"name": item.name, "quantity": item.quantity, "price": item.price} for item in order.items],
                "total_price": order.total_price,
                "status": order.status,
                "assigned_robot": order.assigned_robot,
                "created_at": order.created_at,
                "updated_at": order.updated_at
            })
        return res
