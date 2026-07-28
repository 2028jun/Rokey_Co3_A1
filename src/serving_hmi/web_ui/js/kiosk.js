/**
 * Customer Kiosk Application JavaScript
 * Order creation only -- no admin/telemetry concerns live here.
 */
let socket = null;
let selectedTable = 1;
const cart = {};

document.addEventListener('DOMContentLoaded', () => {
    connectWebSocket();
});

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    socket = new WebSocket(wsUrl);

    socket.onopen = () => {
        console.log("Kiosk connected to HMI Backend WebSocket");
    };

    socket.onclose = () => {
        console.warn("WebSocket disconnected. Retrying in 2 seconds...");
        setTimeout(connectWebSocket, 2000);
    };

    socket.onerror = (err) => {
        console.error("WebSocket error:", err);
    };
}

// Category Tab Switching (메뉴 / 추가 주문 / 테이블 선택)
function switchCategory(name) {
    document.querySelectorAll('.cat-panel').forEach(panel => panel.classList.remove('active'));
    document.querySelectorAll('.cat-tab').forEach(btn => btn.classList.remove('active'));

    const panel = document.getElementById(`cat-panel-${name}`);
    const tab = document.getElementById(`cat-tab-${name}`);
    if (panel) panel.classList.add('active');
    if (tab) tab.classList.add('active');
}

// Tapping anywhere on a card (outside the qty stepper) adds one unit
function quickAdd(id, name, price) {
    updateCart(id, name, price, 1);
}

function updateCart(id, name, price, delta) {
    if (!cart[id]) {
        cart[id] = { id, name, price, quantity: 0 };
    }
    cart[id].quantity += delta;
    // Cutlery is a one-set option; a plate rack carries one to four plates.
    if (id === 'm5' && cart[id].quantity > 1) {
        cart[id].quantity = 1;
    }
    if (id === 'm6' && cart[id].quantity > 4) {
        cart[id].quantity = 4;
    }
    if (cart[id].quantity <= 0) {
        delete cart[id];
    }

    const count = cart[id] ? cart[id].quantity : 0;
    const countSpan = document.getElementById(`qty-${id}`);
    const menuCard = document.getElementById(`menu-card-${id}`);
    if (countSpan) countSpan.innerText = count;
    if (menuCard) {
        if (count > 0) menuCard.classList.add('has-items');
        else menuCard.classList.remove('has-items');
    }

    renderCart();
}

function renderCart() {
    const items = Object.values(cart);
    const listEl = document.getElementById('cart-items-list');
    const totalEl = document.getElementById('summary-total-price');
    const submitBtn = document.getElementById('submit-order-btn');

    if (items.length === 0) {
        listEl.innerHTML = `
            <div class="cart-empty-state">
                <i class="fa-solid fa-pizza-slice"></i>
                <p>메뉴를 선택해 주세요</p>
            </div>`;
        totalEl.innerText = "0원";
        if (submitBtn) submitBtn.disabled = true;
        return;
    }

    listEl.innerHTML = items.map(item => `
        <div class="cart-item-row">
            <div class="cart-item-info">
                <span class="cart-item-name">${item.name}</span>
                <span class="cart-item-sub">${item.price > 0 ? (item.price * item.quantity).toLocaleString() + '원' : '무료'}</span>
            </div>
            <div class="cart-item-qty-controls">
                <button class="qty-btn" onclick="updateCart('${item.id}', '${item.name}', ${item.price}, -1)">−</button>
                <span class="qty-count">${item.quantity}</span>
                <button class="qty-btn" onclick="updateCart('${item.id}', '${item.name}', ${item.price}, 1)">+</button>
            </div>
        </div>
    `).join('');

    const totalPrice = items.reduce((sum, item) => sum + (item.price * item.quantity), 0);
    totalEl.innerText = `${totalPrice.toLocaleString()}원`;
    if (submitBtn) submitBtn.disabled = false;
}

function selectTable(tableNum) {
    selectedTable = tableNum;
    document.querySelectorAll('.table-card').forEach(btn => btn.classList.remove('selected'));
    const card = document.getElementById(`table-card-${tableNum}`);
    if (card) card.classList.add('selected');
    document.getElementById('summary-table-num').innerText = tableNum;
}

function submitOrder() {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
        showToast("백엔드 서버와 연결되어 있지 않습니다.", true);
        return;
    }

    const orderItems = Object.values(cart);
    if (orderItems.length === 0) {
        showToast("최소 1개 이상의 메뉴를 선택해 주세요!", true);
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
    showOrderConfirmation(selectedTable);

    // Reset cart and return to the start screen for the next customer
    for (const key in cart) {
        delete cart[key];
        const countSpan = document.getElementById(`qty-${key}`);
        const menuCard = document.getElementById(`menu-card-${key}`);
        if (countSpan) countSpan.innerText = 0;
        if (menuCard) menuCard.classList.remove('has-items');
    }
    renderCart();
    switchCategory('pizza');
}

function showOrderConfirmation(tableNum) {
    const overlay = document.getElementById('kiosk-confirm-overlay');
    const detail = document.getElementById('kiosk-confirm-detail');
    if (detail) detail.innerText = `Table ${tableNum}으로 로봇이 곧 서빙을 시작합니다.`;
    if (overlay) {
        overlay.classList.add('visible');
        setTimeout(() => overlay.classList.remove('visible'), 3500);
    }
}

let toastTimer = null;
function showToast(message, isError = false) {
    const toast = document.getElementById('kiosk-toast');
    if (!toast) return;
    toast.innerText = message;
    toast.classList.toggle('error', !!isError);
    toast.classList.add('visible');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => toast.classList.remove('visible'), 2800);
}
