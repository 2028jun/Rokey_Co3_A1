/**
 * Restaurant Map Canvas Renderer for HMI
 * Accurately synced with lightweight_pizza_restaurant.usda stage
 * Tracks two independent serving robots.
 */
const ROBOT_MAP_COLORS = {
    robot1: { body: "#6366f1", accent: "#a855f7", label: "#818cf8" },
    robot2: { body: "#0ea5e9", accent: "#38bdf8", label: "#38bdf8" },
};

class RestaurantMap {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');

        // World scale (pixels per meter) -- recomputed in resize() to fit
        // whatever canvas size the layout gives us, so a bigger panel
        // actually draws a bigger map instead of leaving empty margins.
        this.scale = 32;
        // Restaurant floor plan bounds drawn in render(): X=[-6,6], Y=[-5,5]
        this.worldWidth = 12.0;
        this.worldHeight = 10.0;

        // Actual USD Stage TableSet Positions (X, Y in meters)
        this.tables = {
            1: { x: -3.2, y: -2.2, label: "Table 1" },
            2: { x: 3.2,  y: -2.2, label: "Table 2" },
            3: { x: -3.2, y: 0.7,  label: "Table 3" },
            4: { x: 3.2,  y: 0.7,  label: "Table 4" }
        };

        // Kitchen Entry at rear wall gap (y=4.5)
        this.kitchenZone = { x: 0.0, y: 4.5, label: "Lightwheel Kitchen" };

        this.robotPoses = {
            robot1: { x: -1.82, y: -2.20, yaw: 0.0 },
            robot2: { x: 1.82, y: -2.20, yaw: 0.0 },
        };
        this.activeTargetTables = { robot1: null, robot2: null };

        // Robot-local (forward=+x) safety zones.  Kept in sync with the
        // collision_monitor polygons in nav2_params.yaml and the
        // OBSTACLE_STOP_*/OBSTACLE_SLOWDOWN_* constants in
        // nav_restaurant_demo.py -- update all three together.
        this.stopZone = { front: 0.75, back: -0.55, halfWidth: 0.60 };
        this.slowdownZone = { front: 1.35, back: -0.75, halfWidth: 0.85 };

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

        // Fit the fixed-size restaurant floor plan to whatever canvas size
        // we actually got, with a small margin so walls don't touch the edge.
        const padding = 48;
        const scaleX = (this.canvas.width - padding * 2) / this.worldWidth;
        const scaleY = (this.canvas.height - padding * 2) / this.worldHeight;
        this.scale = Math.max(10, Math.min(scaleX, scaleY));

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

    updateRobotPose(robotName, x, y, yaw) {
        if (!this.robotPoses[robotName]) return;
        this.robotPoses[robotName].x = x;
        this.robotPoses[robotName].y = y;
        this.robotPoses[robotName].yaw = yaw || 0.0;
        this.render();
    }

    setActiveTargetTable(robotName, tableNum) {
        if (!(robotName in this.activeTargetTables)) return;
        this.activeTargetTables[robotName] = tableNum || null;
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
        const targetedTables = new Set(Object.values(this.activeTargetTables).filter(Boolean).map(Number));
        for (const [tNum, tbl] of Object.entries(this.tables)) {
            const pos = this.worldToCanvas(tbl.x, tbl.y);
            const isTarget = targetedTables.has(parseInt(tNum));

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

        // 6. Draw Path line from each Robot to its own Target Table
        for (const [robotName, targetTable] of Object.entries(this.activeTargetTables)) {
            if (!targetTable || !this.tables[targetTable]) continue;
            const colors = ROBOT_MAP_COLORS[robotName];
            const targetPos = this.worldToCanvas(this.tables[targetTable].x, this.tables[targetTable].y);
            const rPos = this.worldToCanvas(this.robotPoses[robotName].x, this.robotPoses[robotName].y);

            ctx.strokeStyle = colors.accent;
            ctx.lineWidth = 2.5;
            ctx.setLineDash([6, 6]);
            ctx.beginPath();
            ctx.moveTo(rPos.x, rPos.y);
            ctx.lineTo(targetPos.x, targetPos.y);
            ctx.stroke();
            ctx.setLineDash([]);
        }

        // 7. Draw both robots (Ridgeback + M0609 Arm)
        for (const [robotName, pose] of Object.entries(this.robotPoses)) {
            const colors = ROBOT_MAP_COLORS[robotName];
            const rPos = this.worldToCanvas(pose.x, pose.y);
            ctx.save();
            ctx.translate(rPos.x, rPos.y);
            ctx.rotate(-pose.yaw);

            // Safety zones (drawn under the robot body, local +x = forward,
            // matching the heading dot below).
            const drawZone = (zone, fillStyle, strokeStyle, dash) => {
                const zx = zone.back * this.scale;
                const zy = -zone.halfWidth * this.scale;
                const zw = (zone.front - zone.back) * this.scale;
                const zh = 2 * zone.halfWidth * this.scale;
                ctx.fillStyle = fillStyle;
                ctx.strokeStyle = strokeStyle;
                ctx.lineWidth = 1.5;
                ctx.setLineDash(dash || []);
                ctx.beginPath();
                ctx.roundRect(zx, zy, zw, zh, 6);
                ctx.fill();
                ctx.stroke();
                ctx.setLineDash([]);
            };
            drawZone(this.slowdownZone, "rgba(245, 158, 11, 0.12)", "rgba(245, 158, 11, 0.65)", [5, 4]);
            drawZone(this.stopZone, "rgba(239, 68, 68, 0.18)", "rgba(239, 68, 68, 0.75)");

            // Ridgeback base body
            ctx.fillStyle = colors.body;
            ctx.strokeStyle = colors.accent;
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
            ctx.fillStyle = colors.label;
            ctx.font = "bold 11px Outfit";
            ctx.textAlign = "center";
            ctx.fillText(robotName === "robot1" ? "ROBOT 1" : "ROBOT 2", rPos.x, rPos.y - 18);
        }

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

        // 9. Legend (fixed, unrotated corner overlay)
        const legendX = 10;
        let legendY = this.canvas.height - 52;
        ctx.font = "10px Outfit";
        ctx.textAlign = "left";

        ctx.fillStyle = ROBOT_MAP_COLORS.robot1.body;
        ctx.fillRect(legendX, legendY - 8, 12, 12);
        ctx.fillStyle = "#f1f5f9";
        ctx.fillText("Robot 1", legendX + 18, legendY + 1);

        legendY += 18;
        ctx.fillStyle = ROBOT_MAP_COLORS.robot2.body;
        ctx.fillRect(legendX, legendY - 8, 12, 12);
        ctx.fillStyle = "#f1f5f9";
        ctx.fillText("Robot 2", legendX + 18, legendY + 1);

        legendY += 18;
        ctx.fillStyle = "rgba(239, 68, 68, 0.75)";
        ctx.fillRect(legendX, legendY - 8, 12, 12);
        ctx.fillStyle = "#f1f5f9";
        ctx.fillText("Stop zone", legendX + 18, legendY + 1);
    }

    setObstaclePerson(active, x = 0.0, y = 2.8) {
        this.obstaclePerson = { active: !!active, x: x, y: y };
        this.render();
    }
}
