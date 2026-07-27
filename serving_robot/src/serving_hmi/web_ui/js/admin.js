/**
 * Admin Monitoring Dashboard JavaScript
 * Telemetry, map, dual camera feeds, and operational controls for both
 * serving robots. Order creation lives entirely on the customer kiosk page
 * (kiosk.js) -- this page is read/operate only.
 */
let socket = null;
let mapRenderer = null;
let currentDriveMode = 'MOCK';
let isEmergencyStop = false;
let lastTelemetryData = null;
let openModalRobot = null;

const ROBOT_LABELS = { robot1: 'Robot 1', robot2: 'Robot 2' };
const MISSION_STEPS = [
    { key: 'PENDING', icon: 'fa-clipboard-check', name: '주문 접수' },
    { key: 'PICKING_UP', icon: 'fa-kitchen-set', name: '주방 수거' },
    { key: 'NAVIGATING', icon: 'fa-route', name: '테이블 이동' },
    { key: 'SERVING', icon: 'fa-hand-holding-hand', name: '서빙 중' },
    { key: 'COMPLETED', icon: 'fa-house', name: '복귀 완료' },
];

document.addEventListener('DOMContentLoaded', () => {
    mapRenderer = new RestaurantMap('restaurant-map-canvas');
    connectWebSocket();
});

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    socket = new WebSocket(wsUrl);

    socket.onopen = () => {
        console.log("Admin dashboard connected to HMI Backend WebSocket");
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

function toggleDriveMode() {
    currentDriveMode = (currentDriveMode === 'MOCK') ? 'LIVE' : 'MOCK';
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({
            type: "SET_DRIVE_MODE",
            mode: currentDriveMode
        }));
    }
}

function updateSystemTelemetry(data) {
    lastTelemetryData = data;

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

    const domainTag = document.getElementById('domain-id-tag');
    if (domainTag && data.domain_id) {
        domainTag.innerText = `Domain ${data.domain_id}`;
    }

    const robots = data.robots || {};

    // 2. Overall Robot Status badge (top navbar) mirrors robot1 for a quick
    // at-a-glance read; per-robot detail lives in the select buttons/modal.
    const primary = robots.robot1 || {};
    const robotDot = document.getElementById('robot-dot');
    const robotStateText = document.getElementById('robot-state-text');
    if (primary.connected) {
        robotDot.className = "status-dot online";
        robotStateText.innerText = primary.state || "READY";
    } else {
        robotDot.className = "status-dot offline";
        robotStateText.innerText = "OFFLINE";
    }

    const brakeText = document.getElementById('brake-state-text');
    const brakeIcon = document.getElementById('brake-icon');
    if (primary.parking_brake) {
        brakeText.innerText = "LOCKED";
        brakeText.style.color = "#f59e0b";
        brakeIcon.className = "fa-solid fa-parking-brake text-warning";
    } else {
        brakeText.innerText = "RELEASED";
        brakeText.style.color = "#10b981";
        brakeIcon.className = "fa-solid fa-parking-brake text-success";
    }

    // 3. Per-Robot: map pose/target, camera feed, select-button summary
    for (const robotName of Object.keys(ROBOT_LABELS)) {
        const r = robots[robotName] || {};
        const pose = r.pose || { x: 0, y: 0, yaw: 0 };

        if (mapRenderer) {
            mapRenderer.updateRobotPose(robotName, pose.x, pose.y, pose.yaw || 0);
        }

        const activeOrderId = (data.active_order_ids || {})[robotName];
        const activeOrder = activeOrderId ? (data.orders || []).find(o => o.order_id === activeOrderId) : null;
        if (mapRenderer) {
            mapRenderer.setActiveTargetTable(robotName, activeOrder ? activeOrder.table_number : null);
        }

        updateCameraFeed(
            `camera-stream-img${robotName === 'robot2' ? '-2' : ''}`,
            `camera-placeholder${robotName === 'robot2' ? '-2' : ''}`,
            `camera-status-text${robotName === 'robot2' ? '-2' : ''}`,
            r.camera_connected, r.camera_image
        );

        updateRobotSelectButton(robotName, r);
    }

    if (mapRenderer && data.obstacle) {
        mapRenderer.setObstaclePerson(data.obstacle.active, data.obstacle.x, data.obstacle.y);
    }

    // 4. If a robot's detail modal is open, keep it live-updating too.
    if (openModalRobot) {
        renderRobotModal(openModalRobot, data);
    }
}

function updateRobotSelectButton(robotName, r) {
    const dot = document.getElementById(`robot-select-dot-${robotName}`);
    const stateEl = document.getElementById(`robot-select-state-${robotName}`);
    const battEl = document.getElementById(`robot-select-battery-${robotName}`);
    if (dot) dot.className = `robot-select-dot ${r.connected ? 'online' : 'offline'}`;
    if (stateEl) stateEl.innerText = r.connected ? (r.state || 'READY') : 'OFFLINE';
    if (battEl) battEl.innerText = r.connected ? `${(r.battery || 0).toFixed(0)}%` : '--%';
}

function updateCameraFeed(imgId, placeholderId, statusId, connected, imgData) {
    const camImgEl = document.getElementById(imgId);
    const camPlaceholder = document.getElementById(placeholderId);
    const camStatus = document.getElementById(statusId);

    if (connected && imgData && imgData.length > 0) {
        if (camImgEl) {
            camImgEl.src = imgData;
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

// ---- Per-Robot Detail Modal ----

function openRobotModal(robotName) {
    openModalRobot = robotName;
    document.getElementById('robot-modal-title').innerHTML =
        `<i class="fa-solid fa-robot"></i> ${ROBOT_LABELS[robotName]}`;
    document.getElementById('robot-modal-overlay').classList.add('visible');
    if (lastTelemetryData) renderRobotModal(robotName, lastTelemetryData);
}

function closeRobotModal() {
    openModalRobot = null;
    document.getElementById('robot-modal-overlay').classList.remove('visible');
}

function renderRobotModal(robotName, data) {
    const r = (data.robots || {})[robotName] || {};
    const pose = r.pose || { x: 0, y: 0 };

    // Metrics grid
    const metricsEl = document.getElementById('robot-modal-metrics');
    metricsEl.innerHTML = `
        <div class="metric-card">
            <div class="metric-icon"><i class="fa-solid fa-location-crosshairs"></i></div>
            <div class="metric-body">
                <span class="metric-title">로봇 위치 (Pose)</span>
                <span class="metric-value">(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}) m</span>
            </div>
        </div>
        <div class="metric-card">
            <div class="metric-icon"><i class="fa-solid fa-battery-three-quarters"></i></div>
            <div class="metric-body">
                <span class="metric-title">배터리 잔량</span>
                <span class="metric-value">${(r.battery || 0).toFixed(1)} %</span>
            </div>
        </div>
        <div class="metric-card">
            <div class="metric-icon"><i class="fa-solid fa-parking-brake"></i></div>
            <div class="metric-body">
                <span class="metric-title">주차 브레이크</span>
                <span class="metric-value">${r.parking_brake ? 'LOCKED' : 'RELEASED'}</span>
            </div>
        </div>
        <div class="metric-card">
            <div class="metric-icon"><i class="fa-solid fa-gears"></i></div>
            <div class="metric-body">
                <span class="metric-title">상태</span>
                <span class="metric-value">${r.connected ? (r.state || 'READY') : 'OFFLINE'}</span>
            </div>
        </div>
    `;

    // Mission stepper for this robot's currently active order
    const activeOrderId = (data.active_order_ids || {})[robotName];
    const activeOrder = activeOrderId ? (data.orders || []).find(o => o.order_id === activeOrderId) : null;
    const labelEl = document.getElementById('robot-modal-mission-label');
    const stepperEl = document.getElementById('robot-modal-stepper');

    let activeIndex = -1;
    if (activeOrder) {
        labelEl.innerText = `${activeOrder.order_id} (T${activeOrder.table_number} - ${activeOrder.status})`;
        activeIndex = MISSION_STEPS.findIndex(s => s.key === activeOrder.status);
        if (activeIndex === -1 && activeOrder.status === 'SERVING') activeIndex = 3;
    } else {
        labelEl.innerText = "없음 (대기 중)";
    }

    stepperEl.innerHTML = MISSION_STEPS.map((step, i) => `
        <div class="step-item ${i <= activeIndex ? 'active' : ''}">
            <div class="step-counter"><i class="fa-solid ${step.icon}"></i></div>
            <div class="step-name">${step.name}</div>
        </div>
    `).join('');

    // Orders handled by this robot
    const orders = (data.orders || []).filter(o => o.assigned_robot === robotName);
    const tbody = document.getElementById('robot-modal-orders-body');
    if (orders.length === 0) {
        tbody.innerHTML = `<tr><td colspan="5" class="empty-msg">처리한 주문이 없습니다.</td></tr>`;
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
                        `<button class="test-btn remove" style="padding:4px 8px; font-size:0.75rem;" onclick="cancelOrder('${order.order_id}')">취소</button>` : '-'}
                </td>
            </tr>
        `;
    }).join('');
}

function cancelOrder(orderId) {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    if (currentDriveMode === 'LIVE') {
        alert("LIVE 모드 주문 취소는 manager 취소 API가 없어 지원되지 않습니다.");
        return;
    }
    socket.send(JSON.stringify({
        type: "UPDATE_ORDER_STATUS",
        order_id: orderId,
        status: "CANCELLED"
    }));
}

function resetManagerFault() {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    socket.send(JSON.stringify({ type: "RESET_FAULT" }));
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

function triggerTypingAnimation() {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
        alert("백엔드 서버와 연결되어 있지 않습니다.");
        return;
    }
    socket.send(JSON.stringify({
        type: "TRIGGER_TYPING"
    }));
    const statusEl = document.getElementById('test-service-status');
    if (statusEl) {
        statusEl.innerText = "타이핑 요청 전송";
        statusEl.style.color = "#f59e0b";
    }
}

function setObstacleTestVisible(visible) {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
        alert("백엔드 서버와 연결되어 있지 않습니다.");
        return;
    }
    socket.send(JSON.stringify({
        type: "SET_OBSTACLE_TEST_VISIBLE",
        visible: visible
    }));
    const statusEl = document.getElementById('test-service-status');
    if (statusEl) {
        statusEl.innerText = visible ? "복도 사람 스폰 요청됨 (Visible)" : "복도 사람 제거 요청됨 (Removed)";
        statusEl.style.color = visible ? "#f59e0b" : "#10b981";
    }
}
