/**
 * ESOGÜ Robotics Lab — Scenario Control Dashboard
 * Frontend JavaScript: SocketIO, Chart.js live graphs, process control
 */

// ==============================================================================
// Socket.IO Connection
// ==============================================================================
const socket = io();
let autoScroll = true;

// Scenario label map
const SCENARIO_LABELS = {
    multi_robot_inspection: "Multi-Robot Inspection",
    ur10e_inspection: "UR10e Inspection",
    pick_and_place: "Pick & Place",
    human_robot_collaboration: "Human-Robot Collaboration",
};

// ==============================================================================
// Status Handling
// ==============================================================================

const STATUS_TEXT = {
    stopped: "Offline",
    starting: "Starting...",
    running: "Running",
    stopping: "Stopping...",
};

socket.on("status_update", (data) => {
    // HIL status
    const hilDot = document.getElementById("hil-dot");
    const hilText = document.getElementById("hil-status-text");
    hilDot.className = `status-dot ${data.hil_status}`;
    hilText.textContent = STATUS_TEXT[data.hil_status] || data.hil_status;

    // Scenario status
    const scenarioDot = document.getElementById("scenario-dot");
    const scenarioText = document.getElementById("scenario-status-text");
    scenarioDot.className = `status-dot ${data.scenario_status}`;
    scenarioText.textContent = STATUS_TEXT[data.scenario_status] || data.scenario_status;

    // Active scenario
    const activeText = document.getElementById("active-scenario-text");
    activeText.textContent = data.current_scenario
        ? SCENARIO_LABELS[data.current_scenario] || data.current_scenario
        : "—";

    // Update button states
    document.querySelectorAll(".scenario-btn").forEach((btn) => {
        btn.classList.remove("active");
    });
    if (data.current_scenario) {
        const activeBtn = document.getElementById(`btn-${data.current_scenario}`);
        if (activeBtn) activeBtn.classList.add("active");
    }

    // Show/hide confirm banner
    const banner = document.getElementById("confirm-banner");
    if (
        data.hil_status === "running" &&
        !data.robot_confirmed &&
        data.scenario_status === "stopped"
    ) {
        banner.classList.add("visible");
    } else {
        banner.classList.remove("visible");
    }

    // "Send Command" yalnızca serbest metin komut alan senaryo çalışırken.
    // Senaryo durunca gizlenir; duran bir düğüme komut yollamanın anlamı yok.
    const sendBtn = document.getElementById("btn-send-command");
    const canSend = data.command_prompt && data.scenario_status === "running";
    if (sendBtn) sendBtn.style.display = canSend ? "inline-flex" : "none";
    if (!canSend) closeCommandModal();
});

// ==============================================================================
// Log Handling
// ==============================================================================

const MAX_LOG_ENTRIES = 500;

socket.on("log_message", (data) => {
    const container = document.getElementById("log-container");
    const entry = document.createElement("div");
    entry.className = "log-entry";

    // Determine source class
    let sourceClass = "system";
    const src = data.source.toLowerCase();
    if (src.includes("hil")) sourceClass = "hil";
    else if (src.includes("senaryo") || src.includes("scenario"))
        sourceClass = "scenario";

    entry.innerHTML = `
        <span class="log-time">${data.timestamp}</span>
        <span class="log-source ${sourceClass}">${data.source}</span>
        <span class="log-msg">${escapeHtml(data.message)}</span>
    `;

    container.appendChild(entry);

    // Limit log entries
    while (container.children.length > MAX_LOG_ENTRIES) {
        container.removeChild(container.firstChild);
    }

    // Auto-scroll
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

function startScenario(scenarioKey) {
    const useFakeHardware = document.getElementById("fake-hardware-toggle").checked;
    socket.emit("start_scenario", {
        scenario: scenarioKey,
        use_fake_hardware: useFakeHardware
    });
}

function confirmRobot() {
    socket.emit("confirm_robot");
}

function emergencyStop() {
    if (confirm("🛑 All processes will be stopped. Are you sure?")) {
        socket.emit("stop_all");
    }
}

// ==============================================================================
// Task Command Modal (/gemini/command)
// ==============================================================================

// Son gönderilen komut: pencere yeniden açıldığında kutuda hazır bekler,
// çünkü aynı görev çoğu zaman ufak bir değişiklikle tekrar deneniyor.
let lastCommandText = "";

function openCommandModal() {
    const modal = document.getElementById("command-modal");
    const box = document.getElementById("command-text");
    setCommandStatus("", "");
    if (lastCommandText) box.value = lastCommandText;
    modal.classList.add("visible");
    // Odak ve imleç sona: kullanıcı hemen yazmaya/düzeltmeye başlayabilsin.
    setTimeout(() => {
        box.focus();
        box.setSelectionRange(box.value.length, box.value.length);
    }, 50);
}

function closeCommandModal() {
    const modal = document.getElementById("command-modal");
    if (modal) modal.classList.remove("visible");
}

function onCommandOverlayClick(event) {
    // Yalnızca karartılmış alana tıklanınca kapat; kutunun içine tıklamak
    // (metin seçmek dahil) kapatmamalı.
    if (event.target.id === "command-modal") closeCommandModal();
}

function setCommandStatus(message, kind) {
    const el = document.getElementById("command-status");
    el.textContent = message;
    el.className = `modal-status${kind ? " " + kind : ""}`;
}

function sendCommand() {
    const text = document.getElementById("command-text").value.trim();
    if (!text) {
        setCommandStatus("Please write the task first.", "error");
        return;
    }
    lastCommandText = text;
    setCommandStatus("Publishing… waiting for a subscriber on /gemini/command.", "pending");
    document.getElementById("btn-command-confirm").disabled = true;
    socket.emit("send_command", { text: text });
}

socket.on("command_prompt", () => {
    openCommandModal();
});

socket.on("command_result", (data) => {
    document.getElementById("btn-command-confirm").disabled = false;
    if (data.ok) {
        setCommandStatus("✅ " + data.message, "ok");
        // Başarılıysa pencereyi kapat, ama sonucu okuyacak kadar bekle.
        setTimeout(closeCommandModal, 1200);
    } else {
        setCommandStatus("❌ " + data.message, "error");
    }
});

document.addEventListener("keydown", (event) => {
    const modal = document.getElementById("command-modal");
    if (!modal || !modal.classList.contains("visible")) return;
    if (event.key === "Escape") closeCommandModal();
    if (event.key === "Enter" && (event.ctrlKey || event.metaKey)) sendCommand();
});

// ==============================================================================
// Chart.js — Live Joint State Graphs
// ==============================================================================

// Joint color palette (vibrant, distinguishable)
const JOINT_COLORS = [
    "#6366f1", // indigo   — joint 1 / shoulder_pan
    "#06b6d4", // cyan     — joint 2 / shoulder_lift
    "#10b981", // emerald  — joint 3 / elbow
    "#f59e0b", // amber    — joint 4 / wrist_1
    "#ef4444", // red      — joint 5 / wrist_2
    "#a855f7", // purple   — joint 6 / wrist_3
    "#ec4899", // pink     — linear_axis
    "#14b8a6", // teal     — world_to_agv
    "#f97316", // orange   — kawasaki joint 1
    "#84cc16", // lime     — kawasaki joint 2
    "#3b82f6", // blue     — kawasaki joint 3
    "#e879f9", // fuchsia  — kawasaki joint 4
    "#fbbf24", // yellow   — kawasaki joint 5
    "#22d3ee", // sky      — kawasaki joint 6
];

// Chart configuration factory
function createJointChart(canvasId, titleLabel) {
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
                duration: 0, // No animation for real-time performance
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
                    borderColor: "rgba(99, 102, 241, 0.3)",
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
                            return `${context.dataset.label}: ${context.parsed.y.toFixed(4)} rad`;
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
                        text: "Angle (rad)",
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

const realChart = createJointChart("real-chart", "Real Joint States");
const simChart = createJointChart("sim-chart", "Sim Joint States");

// Track known joint names to maintain consistent colors
let knownRealJoints = [];
let knownSimJoints = [];

function updateChart(chart, data, knownJoints) {
    if (!data || data.length === 0) return knownJoints;

    const now = data[data.length - 1].t;

    // Discover joint names from the data
    const jointNames = new Set();
    data.forEach((d) => {
        Object.keys(d).forEach((k) => {
            if (k !== "t") jointNames.add(k);
        });
    });

    // Merge with known joints (maintain consistent order & colors)
    jointNames.forEach((name) => {
        if (!knownJoints.includes(name)) {
            knownJoints.push(name);
        }
    });

    // Build datasets
    const datasets = knownJoints.map((jointName, idx) => {
        const points = [];
        data.forEach((d) => {
            if (d[jointName] !== undefined) {
                points.push({
                    x: d.t - now, // Relative time (negative seconds ago)
                    y: d[jointName],
                });
            }
        });

        const color = JOINT_COLORS[idx % JOINT_COLORS.length];

        return {
            label: jointName,
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
    chart.update("none"); // 'none' mode skips animations

    return knownJoints;
}

// ==============================================================================
// Joint State Data via SocketIO
// ==============================================================================

socket.on("joint_states", (data) => {
    knownRealJoints = updateChart(realChart, data.real, knownRealJoints);
    knownSimJoints = updateChart(simChart, data.sim, knownSimJoints);
});

// ==============================================================================
// Connection Status
// ==============================================================================

socket.on("connect", () => {
    console.log("[Dashboard] Connected to backend.");
});

socket.on("disconnect", () => {
    console.warn("[Dashboard] Disconnected from backend.");
});

// ==============================================================================
// Keyboard Shortcuts
// ==============================================================================

document.addEventListener("keydown", (e) => {
    // Escape = Emergency Stop
    if (e.key === "Escape") {
        emergencyStop();
    }
});
