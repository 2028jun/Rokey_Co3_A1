/**
 * Restaurant Map Canvas Renderer for HMI
 * Accurately synced with lightweight_pizza_restaurant.usda stage
 */
class RestaurantMap {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        
        // World scale: 1 meter = 32 pixels
        this.scale = 32;
        
        // Actual USD Stage TableSet Positions (X, Y in meters)
        this.tables = {
            1: { x: -3.2, y: -2.2, label: "Table 1" },
            2: { x: 3.2,  y: -2.2, label: "Table 2" },
            3: { x: -3.2, y: 0.7,  label: "Table 3" },
            4: { x: 3.2,  y: 0.7,  label: "Table 4" }
        };
        
        // Kitchen Entry at rear wall gap (y=4.5)
        this.kitchenZone = { x: 0.0, y: 4.5, label: "Lightwheel Kitchen" };
        this.robotPose = { x: -1.82, y: -2.20, yaw: 0.0 };
        this.activeTargetTable = 1;
        
        this.init();
    }

    init() {
        this.resize();
        window.addEventListener('resize', () => this.resize());
        this.render();
    }

    resize() {
        if (!this.canvas.parentElement) return;
        this.canvas.width = this.canvas.parentElement.clientWidth;
        this.canvas.height = this.canvas.parentElement.clientHeight || 380;
        this.originX = this.canvas.width / 2;
        this.originY = this.canvas.height / 2 + 20; // slight offset for kitchen top
        this.render();
    }

    worldToCanvas(wx, wy) {
        // ROS/Isaac convention: +X right, +Y up
        const px = this.originX + wx * this.scale;
        const py = this.originY - wy * this.scale;
        return { x: px, y: py };
    }

    updateRobotPose(x, y, yaw) {
        this.robotPose.x = x;
        this.robotPose.y = y;
        this.robotPose.yaw = yaw || 0.0;
        this.render();
    }

    setActiveTargetTable(tableNum) {
        this.activeTargetTable = tableNum;
        this.render();
    }

    render() {
        const ctx = this.ctx;
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // 1. Draw Outer Restaurant Boundary (12m x 10m: X=[-6, 6], Y=[-5, 5])
        const wallMin = this.worldToCanvas(-6.0, 5.0);
        const wallMax = this.worldToCanvas(6.0, -5.0);
        const wallWidth = wallMax.x - wallMin.x;
        const wallHeight = wallMax.y - wallMin.y;

        ctx.fillStyle = "rgba(15, 23, 42, 0.9)";
        ctx.strokeStyle = "rgba(255, 255, 255, 0.25)";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.roundRect(wallMin.x, wallMin.y, wallWidth, wallHeight, 10);
        ctx.fill();
        ctx.stroke();

        // 2. Draw Floor Grid
        ctx.strokeStyle = "rgba(255, 255, 255, 0.04)";
        ctx.lineWidth = 1;
        const gridSize = 1 * this.scale;
        for (let x = wallMin.x; x <= wallMax.x; x += gridSize) {
            ctx.beginPath();
            ctx.moveTo(x, wallMin.y);
            ctx.lineTo(x, wallMax.y);
            ctx.stroke();
        }
        for (let y = wallMin.y; y <= wallMax.y; y += gridSize) {
            ctx.beginPath();
            ctx.moveTo(wallMin.x, y);
            ctx.lineTo(wallMax.x, y);
            ctx.stroke();
        }

        // 3. Draw Lightwheel Kitchen Pickup Zone (Top Rear Gap)
        const kPos = this.worldToCanvas(this.kitchenZone.x, this.kitchenZone.y);
        ctx.fillStyle = "rgba(245, 158, 11, 0.25)";
        ctx.strokeStyle = "#f59e0b";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.roundRect(kPos.x - 65, kPos.y - 45, 130, 45, 8);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = "#f59e0b";
        ctx.font = "bold 11px Outfit";
        ctx.textAlign = "center";
        ctx.fillText("🍕 Lightwheel Kitchen (Pickup)", kPos.x, kPos.y - 20);

        // 4. Draw Central Mobile Base Aisle (Aisle Line)
        const aisleTop = this.worldToCanvas(0, 4.0);
        const aisleBottom = this.worldToCanvas(0, -4.5);
        ctx.strokeStyle = "rgba(99, 102, 241, 0.15)";
        ctx.lineWidth = 40;
        ctx.beginPath();
        ctx.moveTo(aisleTop.x, aisleTop.y);
        ctx.lineTo(aisleBottom.x, aisleBottom.y);
        ctx.stroke();

        // 5. Draw 4 Tables (TableSet_00 ~ 03)
        for (const [tNum, tbl] of Object.entries(this.tables)) {
            const pos = this.worldToCanvas(tbl.x, tbl.y);
            const isTarget = parseInt(tNum) === this.activeTargetTable;

            // Table Box (1.80m x 0.94m in world scale)
            const tw = 1.80 * this.scale;
            const th = 0.94 * this.scale;

            ctx.fillStyle = isTarget ? "rgba(168, 85, 247, 0.4)" : "rgba(30, 41, 59, 0.85)";
            ctx.strokeStyle = isTarget ? "#a855f7" : "rgba(255, 255, 255, 0.3)";
            ctx.lineWidth = isTarget ? 3 : 1.5;

            ctx.beginPath();
            ctx.roundRect(pos.x - tw / 2, pos.y - th / 2, tw, th, 6);
            ctx.fill();
            ctx.stroke();

            // Table Label
            ctx.fillStyle = isTarget ? "#ffffff" : "#cbd5e1";
            ctx.font = isTarget ? "bold 12px Outfit" : "11px Outfit";
            ctx.textAlign = "center";
            ctx.fillText(tbl.label, pos.x, pos.y + 4);
        }

        // 6. Draw Path line from Robot to Target Table
        if (this.activeTargetTable && this.tables[this.activeTargetTable]) {
            const targetPos = this.worldToCanvas(this.tables[this.activeTargetTable].x, this.tables[this.activeTargetTable].y);
            const rPos = this.worldToCanvas(this.robotPose.x, this.robotPose.y);

            ctx.strokeStyle = "#a855f7";
            ctx.lineWidth = 2.5;
            ctx.setLineDash([6, 6]);
            ctx.beginPath();
            ctx.moveTo(rPos.x, rPos.y);
            ctx.lineTo(targetPos.x, targetPos.y);
            ctx.stroke();
            ctx.setLineDash([]);
        }

        // 7. Draw Robot (Ridgeback + M0609 Arm)
        const rPos = this.worldToCanvas(this.robotPose.x, this.robotPose.y);
        ctx.save();
        ctx.translate(rPos.x, rPos.y);
        ctx.rotate(-this.robotPose.yaw);

        // Ridgeback base body
        ctx.fillStyle = "#6366f1";
        ctx.strokeStyle = "#a855f7";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.roundRect(-16, -14, 32, 28, 5);
        ctx.fill();
        ctx.stroke();

        // Robot Heading Indicator
        ctx.fillStyle = "#ffffff";
        ctx.beginPath();
        ctx.arc(10, 0, 3.5, 0, Math.PI * 2);
        ctx.fill();

        // M0609 Arm Base
        ctx.fillStyle = "#38bdf8";
        ctx.beginPath();
        ctx.arc(-5, 0, 5, 0, Math.PI * 2);
        ctx.fill();

        ctx.restore();

        // Robot Label
        ctx.fillStyle = "#818cf8";
        ctx.font = "bold 11px Outfit";
        ctx.textAlign = "center";
        ctx.fillText("PIZZA BOT", rPos.x, rPos.y - 18);

        // 8. Draw Obstacle Person on Map if Active
        if (this.obstaclePerson && this.obstaclePerson.active) {
            const oPos = this.worldToCanvas(this.obstaclePerson.x || 0.0, this.obstaclePerson.y || 2.8);

            // Pulsing Red Warning Outer Aura
            ctx.fillStyle = "rgba(239, 68, 68, 0.25)";
            ctx.strokeStyle = "#ef4444";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(oPos.x, oPos.y, 18, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();

            // Inner Person Circle
            ctx.fillStyle = "#dc2626";
            ctx.beginPath();
            ctx.arc(oPos.x, oPos.y, 8, 0, Math.PI * 2);
            ctx.fill();

            // Obstacle Label
            ctx.fillStyle = "#f87171";
            ctx.font = "bold 11px Outfit";
            ctx.textAlign = "center";
            ctx.fillText("⚠️ PERSON (STOPPED)", oPos.x, oPos.y - 22);
        }
    }

    setObstaclePerson(active, x = 0.0, y = 2.8) {
        this.obstaclePerson = { active: !!active, x: x, y: y };
        this.render();
    }
}
