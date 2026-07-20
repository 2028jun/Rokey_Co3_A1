/**
 * Main HMI Application JavaScript
 */
let socket = null;
let mapRenderer = null;

let selectedMenu = { id: 'm1', name: '페퍼로니', price: 18000 };
let selectedTable = 1;
let isEmergencyStop = false;
let currentDriveMode = 'MOCK';

function toggleDriveMode() {
    currentDriveMode = (currentDriveMode === 'MOCK') ? 'LIVE' : 'MOCK';
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({
            type: "SET_DRIVE_MODE",
            mode: currentDriveMode
        }));
    }
}

document.addEventListener('DOMContentLoaded', () => {
    mapRenderer = new RestaurantMap('restaurant-map-canvas');
    connectWebSocket();
});

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    socket = new WebSocket(wsUrl);

    socket.onopen = () => {
        console.log("Connected to HMI Backend WebSocket");
    };

    socket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'SYSTEM_STATUS') {
                updateSystemTelemetry(data);
            }
        } catch (e) {
            console.error("Error parsing WS message:", e);
        }
    };

    socket.onclose = () => {
        console.warn("WebSocket disconnected. Retrying in 2 seconds...");
        setTimeout(connectWebSocket, 2000);
    };

    socket.onerror = (err) => {
        console.error("WebSocket error:", err);
    };
}

function updateSystemTelemetry(data) {
    // 0. Drive Mode Button Sync
    if (data.drive_mode) {
        currentDriveMode = data.drive_mode;
        const modeBtn = document.getElementById('mode-toggle-btn');
        const modeText = document.getElementById('mode-text');
        if (modeBtn && modeText) {
            if (currentDriveMode === 'MOCK') {
                modeBtn.className = "mode-toggle-btn mock";
                modeText.innerText = "가상 모의 주행 (Mock)";
            } else {
                modeBtn.className = "mode-toggle-btn live";
                modeText.innerText = "Isaac Sim / 실제 로봇 (Live)";
            }
        }
    }

    // 1. Isaac Sim Status
    const isaac = data.isaac_sim || {};
    const isaacDot = document.getElementById('isaac-dot');
    const isaacStateText = document.getElementById('isaac-state-text');
    const isaacHz = document.getElementById('isaac-hz');

    if (isaac.connected) {
        isaacDot.className = "status-dot online";
        isaacStateText.innerText = "ONLINE";
        isaacHz.innerText = `${isaac.hz.toFixed(1)} Hz`;
    } else {
        isaacDot.className = "status-dot offline";
        isaacStateText.innerText = "OFFLINE";
        isaacHz.innerText = "0.0 Hz";
    }

    // 2. Robot Status
    const robot = data.robot || {};
    const robotDot = document.getElementById('robot-dot');
    const robotStateText = document.getElementById('robot-state-text');

    if (robot.connected) {
        robotDot.className = "status-dot online";
        robotStateText.innerText = robot.state || "READY";
    } else {
        robotDot.className = "status-dot offline";
        robotStateText.innerText = "OFFLINE";
    }

    const domainTag = document.getElementById('domain-id-tag');
    if (domainTag && robot.domain_id) {
        domainTag.innerText = `Domain ${robot.domain_id}`;
    }

    // 3. Brake Status
    const brakeText = document.getElementById('brake-state-text');
    const brakeIcon = document.getElementById('brake-icon');
    if (robot.parking_brake) {
        brakeText.innerText = "LOCKED";
        brakeText.style.color = "#f59e0b";
        brakeIcon.className = "fa-solid fa-parking-brake text-warning";
    } else {
        brakeText.innerText = "RELEASED";
        brakeText.style.color = "#10b981";
        brakeIcon.className = "fa-solid fa-parking-brake text-success";
    }

    // 4. Metrics Grid
    const pose = robot.pose || { x: -1.82, y: -2.20 };
    document.getElementById('val-pose').innerText = `(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}) m`;
    document.getElementById('val-battery').innerText = `${(robot.battery || 100).toFixed(1)} %`;
    document.getElementById('val-sim-hz').innerText = `${(isaac.hz || 0).toFixed(1)} Hz`;
    document.getElementById('val-arm-mode').innerText = robot.state || "READY";

    // Update 2D Canvas Robot Pose
    if (mapRenderer) {
        mapRenderer.updateRobotPose(pose.x, pose.y, pose.yaw || 0);
    }

    // 5. Orders & Active Stepper Progress
    const orders = data.orders || [];
    const activeOrderId = data.active_order_id;
    updateOrderTable(orders);
    updateMissionStepper(orders, activeOrderId);

    // 6. Live Camera Stream Frame Update
    const camImgData = data.camera_image;
    const camImgEl = document.getElementById('camera-stream-img');
    const camPlaceholder = document.getElementById('camera-placeholder');
    const camStatus = document.getElementById('camera-status-text');

    if (camImgData && camImgData.length > 0) {
        if (camImgEl) {
            camImgEl.src = camImgData;
            camImgEl.style.display = 'block';
        }
        if (camPlaceholder) camPlaceholder.style.display = 'none';
        if (camStatus) {
            camStatus.innerText = "LIVE (Receiving Topic)";
            camStatus.style.color = "#10b981";
        }
    } else {
        if (camImgEl) camImgEl.style.display = 'none';
        if (camPlaceholder) camPlaceholder.style.display = 'flex';
        if (camStatus) {
            camStatus.innerText = "STANDBY (Waiting Topic)";
            camStatus.style.color = "#94a3b8";
        }
    }
}

function updateMissionStepper(orders, activeOrderId) {
    const activeDisplay = document.getElementById('active-order-id-display');
    const steps = [
        document.getElementById('step-1'),
        document.getElementById('step-2'),
        document.getElementById('step-3'),
        document.getElementById('step-4'),
        document.getElementById('step-5')
    ];

    steps.forEach(s => s.classList.remove('active'));

    // Find active order or fall back to the latest pending/in-progress order
    let activeOrder = orders.find(o => o.order_id === activeOrderId);
    if (!activeOrder && orders.length > 0) {
        activeOrder = orders.find(o => o.status !== 'COMPLETED' && o.status !== 'CANCELLED') || orders[orders.length - 1];
    }

    if (!activeOrder) {
        activeDisplay.innerText = "없음 (대기 중)";
        steps[0].classList.add('active');
        return;
    }

    activeDisplay.innerText = `${activeOrder.order_id} (T${activeOrder.table_number} - ${activeOrder.status})`;
    if (mapRenderer) {
        mapRenderer.setActiveTargetTable(activeOrder.table_number);
    }

    const status = activeOrder.status;
    let activeIndex = 0;
    if (status === 'PENDING') activeIndex = 0;
    else if (status === 'PICKING_UP') activeIndex = 1;
    else if (status === 'NAVIGATING') activeIndex = 2;
    else if (status === 'SERVING') activeIndex = 3;
    else if (status === 'COMPLETED') activeIndex = 4;

    for (let i = 0; i <= activeIndex; i++) {
        if (steps[i]) steps[i].classList.add('active');
    }
}

function updateOrderTable(orders) {
    const tbody = document.getElementById('order-table-body');
    if (!orders || orders.length === 0) {
        tbody.innerHTML = `<tr><td colspan="5" class="empty-msg">주문 내역이 없습니다.</td></tr>`;
        return;
    }

    tbody.innerHTML = orders.map(order => {
        const menuStr = order.items ? order.items.map(i => i.name).join(', ') : '피자';
        return `
            <tr>
                <td><strong>${order.order_id}</strong></td>
                <td>Table ${order.table_number}</td>
                <td>${menuStr}</td>
                <td><span class="status-badge-tag status-${order.status}">${order.status}</span></td>
                <td>
                    ${order.status !== 'COMPLETED' && order.status !== 'CANCELLED' ? 
                        `<button class="table-btn" style="padding:4px 8px; font-size:0.75rem;" onclick="cancelOrder('${order.order_id}')">취소</button>` : '-'}
                </td>
            </tr>
        `;
    }).join('');
}

// Cart Management
const cart = {};

function updateCart(id, name, price, delta) {
    if (!cart[id]) {
        cart[id] = { id, name, price, quantity: 0 };
    }
    cart[id].quantity += delta;
    if (cart[id].quantity <= 0) {
        delete cart[id];
    }

    // Update UI counters
    const countSpan = document.getElementById(`qty-${id}`);
    const menuCard = document.getElementById(`menu-card-${id}`);
    const count = cart[id] ? cart[id].quantity : 0;
    
    if (countSpan) countSpan.innerText = count;
    if (menuCard) {
        if (count > 0) menuCard.classList.add('has-items');
        else menuCard.classList.remove('has-items');
    }

    renderCartSummary();
}

function renderCartSummary() {
    const items = Object.values(cart);
    const summaryName = document.getElementById('summary-menu-name');
    const summaryTotal = document.getElementById('summary-total-price');

    if (items.length === 0) {
        summaryName.innerText = "메뉴를 선택해 주세요";
        summaryTotal.innerText = "0원";
        return;
    }

    const itemStrList = items.map(item => `${item.name} x${item.quantity}`);
    const totalPrice = items.reduce((sum, item) => sum + (item.price * item.quantity), 0);

    summaryName.innerText = itemStrList.join(', ');
    summaryTotal.innerText = `${totalPrice.toLocaleString()}원`;
}

function selectTable(tableNum) {
    selectedTable = tableNum;
    document.querySelectorAll('.table-btn').forEach(btn => btn.classList.remove('selected'));
    event.currentTarget.classList.add('selected');
    
    document.getElementById('summary-table-num').innerText = `Table ${tableNum}`;
    if (mapRenderer) {
        mapRenderer.setActiveTargetTable(tableNum);
    }
}

function submitOrder() {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
        alert("백엔드 서버와 연결되어 있지 않습니다.");
        return;
    }

    const orderItems = Object.values(cart);
    if (orderItems.length === 0) {
        alert("최소 1개 이상의 메뉴를 수량 선택해 주세요!");
        return;
    }

    const payload = {
        type: "CREATE_ORDER",
        table_number: selectedTable,
        items: orderItems.map(item => ({
            menu_id: item.id,
            name: item.name,
            quantity: item.quantity,
            price: item.price
        }))
    };

    socket.send(JSON.stringify(payload));
    const summaryText = orderItems.map(i => `${i.name}(${i.quantity}개)`).join(', ');
    alert(`Table ${selectedTable}번으로 [${summaryText}] 다중 로봇 서빙 주문이 전송되었습니다!`);

    // Reset cart after submission
    for (const key in cart) {
        delete cart[key];
        const countSpan = document.getElementById(`qty-${key}`);
        const menuCard = document.getElementById(`menu-card-${key}`);
        if (countSpan) countSpan.innerText = 0;
        if (menuCard) menuCard.classList.remove('has-items');
    }
    renderCartSummary();
}

function cancelOrder(orderId) {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    socket.send(JSON.stringify({
        type: "UPDATE_ORDER_STATUS",
        order_id: orderId,
        status: "CANCELLED"
    }));
}

function toggleEmergencyStop() {
    isEmergencyStop = !isEmergencyStop;
    const btn = document.getElementById('estop-btn');
    if (isEmergencyStop) {
        btn.style.background = "#7f1d1d";
        btn.innerHTML = `<i class="fa-solid fa-ban"></i> E-STOP ACTIVE`;
    } else {
        btn.style.background = "linear-gradient(135deg, #dc2626, #ef4444)";
        btn.innerHTML = `<i class="fa-solid fa-triangle-exclamation"></i> EMERGENCY STOP`;
    }

    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({
            type: "EMERGENCY_STOP",
            stop: isEmergencyStop
        }));
    }
}
