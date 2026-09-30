/**
 * Doors — Multirobot Sensing / Cleaning Dashboard
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
    console.log("[Doors] Status:", data.hil_status, "Mission:", data.mission_status);

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
// Cleaning Complete Modal
// ==============================================================================

socket.on("cleaning_complete", (data) => {
    const detail = document.getElementById("cleaning-detail");
    const timeEl = document.getElementById("cleaning-time");
    const cleaned = data.cleaned || 0;
    const total = (data.total !== undefined && data.total !== null) ? data.total : cleaned;
    if (detail) {
        detail.textContent =
            `Cleaning tamamlandı. ${cleaned}/${total} nokta temizlendi.`;
    }
    if (timeEl) timeEl.textContent = data.timestamp || "";

    const overlay = document.getElementById("cleaning-overlay");
    if (overlay) {
        overlay.classList.add("visible");
        const btn = document.getElementById("btn-cleaning-ok");
        if (btn) btn.focus();
    }
});

function ackCleaning() {
    const overlay = document.getElementById("cleaning-overlay");
    const wasOpen = !!(overlay && overlay.classList.contains("visible"));
    if (overlay) overlay.classList.remove("visible");
    // Cleaning bitişini onaylayınca arayüz ilk mission gibi sıfırlanır.
    if (wasOpen) {
        socket.emit("reset_mission");
    }
}

function isCleaningModalOpen() {
    const overlay = document.getElementById("cleaning-overlay");
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
    // The doors mission launch takes two knobs the HARMONY one does not. They are
    // read here rather than at startHIL() so the operator can still change them
    // after the HIL system is up.
    const fake = document.getElementById("fake-hardware-toggle").checked;
    socket.emit("confirm_robot", {
        // No real cameras exist under fake hardware, so that case forces sim-only.
        only_sim: fake || document.getElementById("only-sim-toggle").checked,
        force_replan: document.getElementById("force-replan-toggle").checked,
    });
}

// Sunucudan gelen sıfırlama sinyali: grafikleri ve göstergeleri temizle.
// (defect tablosu, sunucunun yayınladığı boş defect_list ile temizlenir)
socket.on("mission_reset", () => {
    clearCharts();
    const modeEl = document.getElementById("current-mode");
    const defectEl = document.getElementById("defect-count");
    if (modeEl) modeEl.textContent = "IDLE";
    if (defectEl) defectEl.textContent = "0";
});

// ==============================================================================
// Chart.js — Live Charts
// ==============================================================================

// ── Joint series encoding ─────────────────────────────────────────────────────
// FOURTEEN series now share each chart (7 UR + 7 Kawasaki). Fourteen categorical
// hues cannot be told apart -- measured, not assumed: the widest 14-hue set that
// fits the dark lightness band still leaves adjacent pairs at OKLab dE 3.7 under
// deuteranopia and 12.5 under normal vision, against floors of 8 and 15.
//
// So identity is carried by TWO channels instead of one:
//     hue        = the joint's ROLE (both arms have 6 joints + 1 rail, so the
//                  roles line up one-to-one)
//     line style = which arm (UR solid, Kawasaki dashed)
//
// Seven hues DO validate. This set was generated at OKLCH L 0.62 and checked
// against the panel surface #1a2035: lightness band PASS, chroma floor PASS,
// adjacent CVD dE 10.1 (deutan) PASS, adjacent normal-vision dE 23.0 PASS,
// contrast >= 3:1 PASS. Order matters -- it is what the adjacent-pair check was
// run on, so do not reshuffle these without re-validating.
const JOINT_COLORS = [
    "#d35574", // rose    — joint 1 / shoulder pan
    "#2395ab", // teal    — joint 2 / shoulder lift
    "#c06f1d", // ochre   — joint 3 / elbow
    "#5680e6", // blue    — joint 4 / wrist 1
    "#8a8c1e", // olive   — joint 5 / wrist 2
    "#ac63c5", // violet  — joint 6 / wrist 3
    "#239d6f", // green   — the rail
];

// role: index into JOINT_COLORS. arm: picks solid vs dashed.
const JOINT_META = {
    // UR10e — solid
    "shoulder_pan_joint":  { role: 0, arm: "UR", label: "UR Shoulder Pan" },
    "shoulder_lift_joint": { role: 1, arm: "UR", label: "UR Shoulder Lift" },
    "elbow_joint":         { role: 2, arm: "UR", label: "UR Elbow" },
    "wrist_1_joint":       { role: 3, arm: "UR", label: "UR Wrist 1" },
    "wrist_2_joint":       { role: 4, arm: "UR", label: "UR Wrist 2" },
    "wrist_3_joint":       { role: 5, arm: "UR", label: "UR Wrist 3" },
    "base_to_robot_mount": { role: 6, arm: "UR", label: "UR Linear Axis" },
    // Kawasaki RS005L — dashed
    "joint1":       { role: 0, arm: "KAWA", label: "KAWA Joint 1" },
    "joint2":       { role: 1, arm: "KAWA", label: "KAWA Joint 2" },
    "joint3":       { role: 2, arm: "KAWA", label: "KAWA Joint 3" },
    "joint4":       { role: 3, arm: "KAWA", label: "KAWA Joint 4" },
    "joint5":       { role: 4, arm: "KAWA", label: "KAWA Joint 5" },
    "joint6":       { role: 5, arm: "KAWA", label: "KAWA Joint 6" },
    "world_to_agv": { role: 6, arm: "KAWA", label: "KAWA AGV Rail" },
};

const KAWASAKI_DASH = [5, 4];

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
                        // Tightened for the joint charts: 14 series (both arms) used
                        // to push the legend over half the panel. The swatch keeps
                        // its line style, so a dashed key reads as "Kawasaki"
                        // straight from the legend.
                        boxWidth: 18,
                        boxHeight: 2,
                        padding: 5,
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
const simPositionChart = createChart("sim-position-chart", "Angle (rad)", "rad");
const forceChart = createChart("force-chart", "Force (N)", "N");

// Track known joint names
let knownPositionJoints = [];
let knownSimJoints = [];

// Grafikleri boşaltır (reset sırasında çağrılır).
function clearCharts() {
    [positionChart, simPositionChart, forceChart].forEach((chart) => {
        if (chart) {
            chart.data.datasets = [];
            chart.update("none");
        }
    });
    knownPositionJoints = [];
    knownSimJoints = [];
}

// Legend/series order: UR first, then Kawasaki, each in kinematic order. Fixed,
// not arrival-ordered, so a joint that starts publishing late does not repaint or
// reorder the series already on screen.
const JOINT_ORDER = Object.keys(JOINT_META);
const jointRank = (name) => {
    const i = JOINT_ORDER.indexOf(name);
    return i === -1 ? JOINT_ORDER.length : i;   // unknown joints sort to the end
};

function updateJointChart(chart, data, knownJoints) {
    if (!data || data.length === 0) return knownJoints;

    const now = data[data.length - 1].t;

    const jointNames = new Set();
    data.forEach((d) => {
        Object.keys(d).forEach((k) => {
            if (k !== "t") jointNames.add(k);
        });
    });

    let changed = false;
    jointNames.forEach((name) => {
        if (!knownJoints.includes(name)) {
            knownJoints.push(name);
            changed = true;
        }
    });
    if (changed) knownJoints.sort((a, b) => jointRank(a) - jointRank(b));

    const datasets = knownJoints.map((jointName) => {
        const points = [];
        data.forEach((d) => {
            if (d[jointName] !== undefined) {
                points.push({
                    x: d.t - now,
                    y: d[jointName],
                });
            }
        });

        // Colour comes from the joint's ROLE, never from its position in the list,
        // so the palette stays stable whichever joints happen to be publishing.
        const meta = JOINT_META[jointName];
        const color = meta ? JOINT_COLORS[meta.role] : "#94a3b8";
        const isKawa = meta && meta.arm === "KAWA";

        return {
            label: meta ? meta.label : jointName,
            data: points,
            borderColor: color,
            backgroundColor: color + "20",
            borderWidth: 1.5,
            borderDash: isKawa ? KAWASAKI_DASH : [],
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
        knownPositionJoints
    );
    knownSimJoints = updateJointChart(
        simPositionChart,
        data.sim_positions,
        knownSimJoints
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
    if (isNoticeModalOpen()) {
        if (e.key === "Escape" || e.key === "Enter") {
            e.preventDefault();
            ackNotice();
        }
        return;
    }
    if (isCleaningModalOpen()) {
        if (e.key === "Escape" || e.key === "Enter") {
            e.preventDefault();
            ackCleaning();
        }
        return;
    }

    if (e.key === "Escape") {
        emergencyStop();
    }
});
