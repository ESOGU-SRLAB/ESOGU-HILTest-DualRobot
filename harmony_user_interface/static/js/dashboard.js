/**
 * Harmony — Sensing-Cleaning Robot Dashboard
 * Frontend JavaScript: SocketIO, Chart.js live graphs, process control
 */

// ==============================================================================
// Socket.IO Connection
// ==============================================================================
const socket = io();
let autoScroll = true;

// ==============================================================================
// Status Handling
// ==============================================================================

socket.on("status_update", (data) => {
    console.log("[Harmony] Status:", data.hil_status, "Mission:", data.mission_status);

    // Show/hide confirm banner
    const banner = document.getElementById("confirm-banner");
    if (
        data.hil_status === "running" &&
        !data.robot_confirmed &&
        data.mission_status === "stopped"
    ) {
        banner.classList.add("visible");
    } else {
        banner.classList.remove("visible");
    }
});

// ==============================================================================
// Robot Info Handling (Mode + Defect Count)
// ==============================================================================

socket.on("robot_info", (data) => {
    const modeEl = document.getElementById("current-mode");
    const defectEl = document.getElementById("defect-count");

    if (modeEl) modeEl.textContent = data.mode || "IDLE";
    if (defectEl) defectEl.textContent = data.defect_count || 0;
});

// ==============================================================================
// Defect Table
// ==============================================================================

const DEFECT_STATUS_CLASS = {
    DETECTED: "status-detected",
    CLEANING: "status-cleaning",
    CLEANED: "status-cleaned",
    FAILED: "status-failed",
};

socket.on("defect_list", (data) => {
    const tbody = document.getElementById("defect-tbody");
    if (!tbody) return;

    const defects = (data && data.defects) || [];
    tbody.innerHTML = "";

    if (defects.length === 0) {
        const row = document.createElement("tr");
        row.className = "defect-empty-row";
        row.innerHTML =
            '<td colspan="6">No defects reported yet. Run Sensing Start.</td>';
        tbody.appendChild(row);
        return;
    }

    defects.forEach((d) => {
        const status = d.status || "DETECTED";
        const statusClass = DEFECT_STATUS_CLASS[status] || "status-detected";
        const row = document.createElement("tr");
        row.innerHTML = `
            <td class="defect-id">${escapeHtml(d.id || "")}</td>
            <td>${escapeHtml(d.type || "")}</td>
            <td class="defect-num">${Number(d.x).toFixed(3)}</td>
            <td class="defect-num">${Number(d.y).toFixed(3)}</td>
            <td class="defect-num">${Number(d.z).toFixed(3)}</td>
            <td><span class="defect-status ${statusClass}">${escapeHtml(status)}</span></td>
        `;
        tbody.appendChild(row);
    });

    const badge = document.getElementById("defect-frame-badge");
    if (badge && defects[0]) {
        badge.textContent = `frame: ${defects[0].frame_id || "world"} · metre`;
    }
});

// ==============================================================================
// Scenario Fault Banner (KPI3)
// ==============================================================================

let currentFaultCode = null;

socket.on("scenario_fault", (data) => {
    currentFaultCode = data.code || "?";

    const codeEl = document.getElementById("fault-code");
    const detail = document.getElementById("fault-detail");
    const timeEl = document.getElementById("fault-time");
    if (codeEl) codeEl.textContent = data.code || "";
    if (detail) detail.textContent = data.message || "";
    if (timeEl) timeEl.textContent = data.timestamp ? `${data.timestamp}` : "";

    const overlay = document.getElementById("fault-overlay");
    if (overlay) {
        overlay.classList.add("visible");
        const btn = document.getElementById("btn-fault-ok");
        if (btn) btn.focus();
    }
});

function ackFault() {
    const overlay = document.getElementById("fault-overlay");
    if (overlay) overlay.classList.remove("visible");
    socket.emit("ack_fault", { code: currentFaultCode });
    currentFaultCode = null;
}

function isFaultModalOpen() {
    const overlay = document.getElementById("fault-overlay");
    return !!(overlay && overlay.classList.contains("visible"));
}

// ==============================================================================
// Sensing Complete Modal
// ==============================================================================

socket.on("sensing_complete", (data) => {
    const detail = document.getElementById("notice-detail");
    const timeEl = document.getElementById("notice-time");
    const n = data.count || 0;
    if (detail) {
        detail.textContent =
            `Sensing tamamlandı. ${n} defect raporlandı ve listelendi.`;
    }
    if (timeEl) timeEl.textContent = data.timestamp || "";

    const overlay = document.getElementById("notice-overlay");
    if (overlay) {
        overlay.classList.add("visible");
        const btn = document.getElementById("btn-notice-ok");
        if (btn) btn.focus();
    }
});

function ackNotice() {
    const overlay = document.getElementById("notice-overlay");
    if (overlay) overlay.classList.remove("visible");
}

function isNoticeModalOpen() {
    const overlay = document.getElementById("notice-overlay");
    return !!(overlay && overlay.classList.contains("visible"));
}

// ==============================================================================
// Log Handling
// ==============================================================================

const MAX_LOG_ENTRIES = 500;

socket.on("log_message", (data) => {
    const container = document.getElementById("log-container");
    const entry = document.createElement("div");
    entry.className = "log-entry";

    let sourceClass = "system";
    const src = data.source.toLowerCase();
    if (src.includes("hil")) sourceClass = "hil";
    else if (src.includes("mission")) sourceClass = "mission";
    else if (src.includes("fault")) sourceClass = "fault";

    entry.innerHTML = `
        <span class="log-time">${data.timestamp}</span>
        <span class="log-source ${sourceClass}">${data.source}</span>
        <span class="log-msg">${escapeHtml(data.message)}</span>
    `;

    container.appendChild(entry);

    while (container.children.length > MAX_LOG_ENTRIES) {
        container.removeChild(container.firstChild);
    }

    if (autoScroll) {
        container.scrollTop = container.scrollHeight;
    }
});

function escapeHtml(text) {
    const div = document.createElement("div");
    div.textContent = text;
    return div.innerHTML;
}

function toggleAutoScroll() {
    autoScroll = !autoScroll;
    document.getElementById("autoscroll-label").textContent = autoScroll
        ? "ON"
        : "OFF";
}

function clearLogs() {
    const container = document.getElementById("log-container");
    container.innerHTML = "";
}

// ==============================================================================
// Control Functions
// ==============================================================================

function startHIL() {
    const useFakeHardware = document.getElementById("fake-hardware-toggle").checked;
    socket.emit("start_hil", {
        use_fake_hardware: useFakeHardware,
    });
}

function emergencyStop() {
    if (confirm("🛑 All processes will be stopped. Are you sure?")) {
        socket.emit("stop_all");
    }
}

function publishCmd(cmdName) {
    socket.emit("publish_cmd", { cmd: cmdName });

    // Tıklamanın algılandığını operatöre göster (kısa flaş).
    const btn = document.getElementById(`btn-cmd-${cmdName.toLowerCase()}`);
    if (btn) {
        btn.classList.remove("cmd-sent");
        void btn.offsetWidth; // animasyonu yeniden tetiklemek için reflow
        btn.classList.add("cmd-sent");
        setTimeout(() => btn.classList.remove("cmd-sent"), 500);
    }
}

function confirmRobot() {
    socket.emit("confirm_robot");
}

// ==============================================================================
// Chart.js — Live Charts
// ==============================================================================

// Joint color palette
const JOINT_COLORS = [
    "#14b8a6", // teal     — shoulder_pan
    "#06b6d4", // cyan     — shoulder_lift
    "#10b981", // emerald  — elbow
    "#f59e0b", // amber    — wrist_1
    "#ef4444", // red      — wrist_2
    "#a855f7", // purple   — wrist_3
    "#ec4899", // pink     — base_to_robot_mount
];

// Joint name display mapping
const JOINT_DISPLAY_NAMES = {
    "shoulder_pan_joint": "Shoulder Pan",
    "shoulder_lift_joint": "Shoulder Lift",
    "elbow_joint": "Elbow",
    "wrist_1_joint": "Wrist 1",
    "wrist_2_joint": "Wrist 2",
    "wrist_3_joint": "Wrist 3",
    "base_to_robot_mount": "Linear Axis",
};

// Chart configuration factory
function createChart(canvasId, yLabel, tooltipUnit) {
    const ctx = document.getElementById(canvasId).getContext("2d");

    return new Chart(ctx, {
        type: "line",
        data: {
            datasets: [],
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: {
                duration: 0,
            },
            interaction: {
                mode: "nearest",
                axis: "x",
                intersect: false,
            },
            plugins: {
                legend: {
                    display: true,
                    position: "bottom",
                    labels: {
                        color: "#94a3b8",
                        font: {
                            family: "'Inter', sans-serif",
                            size: 10,
                            weight: "500",
                        },
                        boxWidth: 14,
                        boxHeight: 3,
                        padding: 8,
                        usePointStyle: false,
                    },
                },
                tooltip: {
                    backgroundColor: "rgba(17, 24, 39, 0.95)",
                    titleColor: "#f1f5f9",
                    bodyColor: "#94a3b8",
                    borderColor: "rgba(20, 184, 166, 0.3)",
                    borderWidth: 1,
                    cornerRadius: 8,
                    titleFont: {
                        family: "'Inter', sans-serif",
                        size: 11,
                        weight: "600",
                    },
                    bodyFont: {
                        family: "'JetBrains Mono', monospace",
                        size: 10,
                    },
                    callbacks: {
                        label: function (context) {
                            return `${context.dataset.label}: ${context.parsed.y.toFixed(4)} ${tooltipUnit}`;
                        },
                    },
                },
            },
            scales: {
                x: {
                    type: "linear",
                    title: {
                        display: true,
                        text: "Time (s)",
                        color: "#64748b",
                        font: { size: 10, family: "'Inter', sans-serif" },
                    },
                    ticks: {
                        color: "#64748b",
                        font: { size: 9, family: "'JetBrains Mono', monospace" },
                        callback: (val) => val.toFixed(0) + "s",
                        maxTicksLimit: 8,
                    },
                    grid: {
                        color: "rgba(255, 255, 255, 0.04)",
                    },
                    min: -20,
                    max: 0,
                },
                y: {
                    title: {
                        display: true,
                        text: yLabel,
                        color: "#64748b",
                        font: { size: 10, family: "'Inter', sans-serif" },
                    },
                    ticks: {
                        color: "#64748b",
                        font: { size: 9, family: "'JetBrains Mono', monospace" },
                        callback: (val) => val.toFixed(2),
                        maxTicksLimit: 8,
                    },
                    grid: {
                        color: "rgba(255, 255, 255, 0.04)",
                    },
                },
            },
        },
    });
}

const positionChart = createChart("position-chart", "Angle (rad)", "rad");
const velocityChart = createChart("velocity-chart", "Velocity (rad/s)", "rad/s");
const forceChart = createChart("force-chart", "Force (N)", "N");

// Track known joint names
let knownPositionJoints = [];
let knownVelocityJoints = [];

function updateJointChart(chart, data, knownJoints, colorPalette) {
    if (!data || data.length === 0) return knownJoints;

    const now = data[data.length - 1].t;

    const jointNames = new Set();
    data.forEach((d) => {
        Object.keys(d).forEach((k) => {
            if (k !== "t") jointNames.add(k);
        });
    });

    jointNames.forEach((name) => {
        if (!knownJoints.includes(name)) {
            knownJoints.push(name);
        }
    });

    const datasets = knownJoints.map((jointName, idx) => {
        const points = [];
        data.forEach((d) => {
            if (d[jointName] !== undefined) {
                points.push({
                    x: d.t - now,
                    y: d[jointName],
                });
            }
        });

        const color = colorPalette[idx % colorPalette.length];
        const displayName = JOINT_DISPLAY_NAMES[jointName] || jointName;

        return {
            label: displayName,
            data: points,
            borderColor: color,
            backgroundColor: color + "20",
            borderWidth: 1.5,
            pointRadius: 0,
            tension: 0.3,
            fill: false,
        };
    });

    chart.data.datasets = datasets;
    chart.update("none");

    return knownJoints;
}

function updateForceChart(chart, data) {
    if (!data || data.length === 0) return;

    const now = data[data.length - 1].t;

    const points = data.map((d) => ({
        x: d.t - now,
        y: d.force_z,
    }));

    chart.data.datasets = [
        {
            label: "Force Z",
            data: points,
            borderColor: "#f97316",
            backgroundColor: "rgba(249, 115, 22, 0.1)",
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.3,
            fill: true,
        },
    ];
    chart.update("none");
}

// ==============================================================================
// Chart Data via SocketIO
// ==============================================================================

socket.on("chart_data", (data) => {
    knownPositionJoints = updateJointChart(
        positionChart,
        data.positions,
        knownPositionJoints,
        JOINT_COLORS
    );
    knownVelocityJoints = updateJointChart(
        velocityChart,
        data.velocities,
        knownVelocityJoints,
        JOINT_COLORS
    );
    updateForceChart(forceChart, data.forces);
});

// ==============================================================================
// Connection Status
// ==============================================================================

socket.on("connect", () => {
    console.log("[Harmony] Connected to backend.");
});

socket.on("disconnect", () => {
    console.warn("[Harmony] Disconnected from backend.");
});

// ==============================================================================
// Keyboard Shortcuts
// ==============================================================================

document.addEventListener("keydown", (e) => {
    // Pop-up açıkken Escape/Enter pop-up'ı kapatır, acil durdurmayı tetiklemez.
    if (isFaultModalOpen()) {
        if (e.key === "Escape" || e.key === "Enter") {
            e.preventDefault();
            ackFault();
        }
        return;
    }
    if (isNoticeModalOpen()) {
        if (e.key === "Escape" || e.key === "Enter") {
            e.preventDefault();
            ackNotice();
        }
        return;
    }

    if (e.key === "Escape") {
        emergencyStop();
    }
});
