/**
 * ESOGÜ Robotics Lab — Data Analytics tab
 * Read-only visualization of collected data from Elasticsearch.
 * Mirrors the existing Grafana dashboard panels (2D time-series + 3D TCP path).
 *
 * The home dashboard (dashboard.js) is completely independent of this file.
 */

// ==============================================================================
// Configuration
// ==============================================================================

const ANALYTICS_JOINT_COLORS = [
    "#6366f1", "#06b6d4", "#10b981", "#f59e0b",
    "#f97316", "#a855f7", "#ef4444", "#84cc16",
];

// UR10e joint base names, in a sensible kinematic order.
const UR10E_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "base_to_robot_mount",
];

function prettyLabel(name) {
    return name
        .replace(/_/g, " ")
        .replace(/\bjoint\b/gi, "")
        .replace(/\s+/g, " ")
        .trim();
}

// Build a field list ({key,label,color}) for a UR10e-style index.
function ur10eFields(prefix, suffix) {
    return UR10E_JOINTS.map((j, i) => ({
        key: `${prefix}${j}.${suffix}`,
        label: prettyLabel(j),
        color: ANALYTICS_JOINT_COLORS[i % ANALYTICS_JOINT_COLORS.length],
    }));
}

// Panel definitions — order/layout mirrors the Grafana dashboard.
const PANELS = [
    // Row 1 — UR10e positions: real | sim
    {
        id: "ur10e_pos_real", type: "line", side: "live",
        title: "UR10e — Joint Positions (Real)",
        index: "ros-joint-states", time_field: "@timestamp",
        unit: "rad", fields: ur10eFields("ur10e_", "position"),
    },
    {
        id: "ur10e_pos_sim", type: "line", side: "sim",
        title: "UR10e — Joint Positions (Sim)",
        index: "ros-sim-joint-states", time_field: "@timestamp",
        unit: "rad", fields: ur10eFields("sim_ur10e_", "position"),
    },
    // Row 2 — Kawasaki + AGV positions: real | sim
    {
        id: "kawa_pos_real", type: "line", side: "live",
        title: "Kawasaki — Joint Positions (Real)",
        index: "ros-kawasaki-joint-states", time_field: "@timestamp",
        unit: "rad",
        fields: [1, 2, 3, 4, 5, 6].map((n, i) => ({
            key: `joint${n}.position`, label: `joint ${n}`,
            color: ANALYTICS_JOINT_COLORS[i % ANALYTICS_JOINT_COLORS.length],
        })),
    },
    {
        id: "kawa_pos_sim", type: "line", side: "sim",
        title: "Kawasaki + AGV — Joint Positions (Sim)",
        index: "ros-sim-joint-states", time_field: "@timestamp",
        unit: "rad",
        fields: [
            ...[1, 2, 3, 4, 5, 6].map((n, i) => ({
                key: `sim_kawasaki_joint${n}.position`, label: `joint ${n}`,
                color: ANALYTICS_JOINT_COLORS[i % ANALYTICS_JOINT_COLORS.length],
            })),
            { key: "world_to_agv.position", label: "AGV", color: ANALYTICS_JOINT_COLORS[6] },
        ],
    },
    // Row 3 — UR10e velocities: real | sim
    {
        id: "ur10e_vel_real", type: "line", side: "live",
        title: "UR10e — Joint Velocities (Real)",
        index: "ros-joint-states", time_field: "@timestamp",
        unit: "rad/s", fields: ur10eFields("ur10e_", "velocity"),
    },
    {
        id: "ur10e_vel_sim", type: "line", side: "sim",
        title: "UR10e — Joint Velocities (Sim)",
        index: "ros-sim-joint-states", time_field: "@timestamp",
        unit: "rad/s", fields: ur10eFields("sim_ur10e_", "velocity"),
    },
    // Row 4 — UR10e efforts: real | sim
    {
        id: "ur10e_eff_real", type: "line", side: "live",
        title: "UR10e — Joint Efforts (Real)",
        index: "ros-joint-states", time_field: "@timestamp",
        unit: "Nm", fields: ur10eFields("ur10e_", "effort"),
    },
    {
        id: "ur10e_eff_sim", type: "line", side: "sim",
        title: "UR10e — Joint Efforts (Sim)",
        index: "ros-sim-joint-states", time_field: "@timestamp",
        unit: "Nm", fields: ur10eFields("sim_ur10e_", "effort"),
    },
    // Row 5 — 3D TCP path (full width)
    {
        id: "tcp_3d", type: "scatter3d", side: "live",
        title: "TCP Position — 3D Path",
        index: "ros-tcp-pose-topic",
        time_field: "header.sec", time_unit: "s",
        x: "pose.position.x", y: "pose.position.y", z: "pose.position.z",
    },
];

// ==============================================================================
// State
// ==============================================================================

const charts = {};        // panel id → Chart.js instance
let analyticsBuilt = false;
let refreshTimer = null;
let currentTab = "home";

// ==============================================================================
// Tab switching
// ==============================================================================

function switchTab(tab) {
    currentTab = tab;
    const home = document.getElementById("view-home");
    const analytics = document.getElementById("view-analytics");
    const btnHome = document.getElementById("tab-btn-home");
    const btnAnalytics = document.getElementById("tab-btn-analytics");

    if (tab === "analytics") {
        home.hidden = true;
        analytics.hidden = false;
        btnHome.classList.remove("active");
        btnAnalytics.classList.add("active");

        // Surface any build/render failure instead of silently doing nothing.
        try {
            if (typeof Chart === "undefined") {
                throw new Error("Chart.js failed to load (check internet/CDN access).");
            }
            if (!analyticsBuilt) {
                buildPanels();
                analyticsBuilt = true;
            }
            refreshAnalytics();
            startAutoRefresh();
        } catch (err) {
            console.error("[analytics] switchTab failed:", err);
            showFatal(err.message || String(err));
        }
    } else {
        analytics.hidden = true;
        home.hidden = false;
        btnAnalytics.classList.remove("active");
        btnHome.classList.add("active");
        stopAutoRefresh();
    }
}
window.switchTab = switchTab;

// Show a prominent error inside the analytics view so failures are never silent.
function showFatal(message) {
    const grid = document.getElementById("analytics-grid");
    if (grid) {
        grid.innerHTML =
            '<div class="a-panel full-width"><div class="a-panel-empty" ' +
            'style="position:relative;color:#ef4444;display:flex;">' +
            "⚠️ Analytics error: " + message +
            "<br><br>Open the browser console (F12) for details." +
            "</div></div>";
    }
    const status = document.getElementById("analytics-status");
    if (status) { status.textContent = message; status.className = "tb-status error"; }
}

// Catch any uncaught error (e.g. a failed CDN script) and make it visible.
window.addEventListener("error", (e) => {
    if (currentTab === "analytics") {
        showFatal((e.message || "Unknown error") +
            (e.filename ? " @ " + e.filename.split("/").pop() + ":" + e.lineno : ""));
    }
});

// ==============================================================================
// Panel construction
// ==============================================================================

function buildPanels() {
    const grid = document.getElementById("analytics-grid");
    grid.innerHTML = "";

    PANELS.forEach((p) => {
        const panel = document.createElement("div");
        panel.className = "a-panel" + (p.type === "scatter3d" ? " full-width" : "");

        const badgeClass = p.side === "sim" ? "sim" : "live";
        const badgeText = p.side === "sim" ? "● SIM" : "● LIVE";

        panel.innerHTML = `
            <div class="a-panel-header">
                <h3>${p.title}</h3>
                <span class="a-panel-badge ${badgeClass}">${badgeText}</span>
            </div>
            <div class="a-panel-body">
                ${p.type === "line"
                    ? `<canvas id="canvas-${p.id}"></canvas>`
                    : `<div id="plot-${p.id}" style="width:100%;height:100%;"></div>`}
                <div class="a-panel-empty" id="empty-${p.id}" style="display:none;">
                    No data in selected range.
                </div>
            </div>
        `;
        grid.appendChild(panel);

        if (p.type === "line") {
            createLineChart(p);
        }
    });
}

function createLineChart(p) {
    const ctx = document.getElementById(`canvas-${p.id}`).getContext("2d");
    charts[p.id] = new Chart(ctx, {
        type: "line",
        data: {
            datasets: p.fields.map((f) => ({
                label: f.label,
                borderColor: f.color,
                backgroundColor: f.color,
                data: [],
                borderWidth: 1.5,
                pointRadius: 0,
                tension: 0.15,
                spanGaps: true,
            })),
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            interaction: { mode: "nearest", axis: "x", intersect: false },
            scales: {
                x: {
                    type: "linear",
                    ticks: {
                        maxTicksLimit: 8,
                        color: "#64748b",
                        callback: (v) => new Date(v).toLocaleTimeString(),
                    },
                    grid: { color: "rgba(255,255,255,0.04)" },
                },
                y: {
                    title: { display: !!p.unit, text: p.unit, color: "#64748b" },
                    ticks: { color: "#64748b" },
                    grid: { color: "rgba(255,255,255,0.04)" },
                },
            },
            plugins: {
                legend: {
                    labels: { color: "#94a3b8", boxWidth: 12, font: { size: 10 } },
                },
                tooltip: {
                    callbacks: {
                        title: (items) =>
                            items.length
                                ? new Date(items[0].parsed.x).toLocaleString()
                                : "",
                    },
                },
            },
        },
    });
}

// ==============================================================================
// Data fetching / refresh
// ==============================================================================

function getRangeParams() {
    const preset = document.getElementById("range-preset").value;
    if (preset === "all") return {};

    if (preset === "custom") {
        // datetime-local inputs are interpreted as UTC and passed to ES as
        // ISO strings (ES parses a zone-less ISO timestamp as UTC).
        const from = document.getElementById("range-from").value;
        const to = document.getElementById("range-to").value;
        const r = {};
        if (from) r.from = from;
        if (to) r.to = to;
        return r;
    }

    const span = parseInt(preset, 10);
    const now = Date.now();
    return { from: now - span, to: now };
}

// Toggle the custom absolute-range inputs based on the preset selection.
function onPresetChange() {
    const isCustom = document.getElementById("range-preset").value === "custom";
    const disp = isCustom ? "" : "none";
    document.getElementById("custom-from-field").style.display = disp;
    document.getElementById("custom-to-field").style.display = disp;
    document.getElementById("btn-fit").style.display = isCustom ? "" : "none";
    if (!isCustom) refreshAnalytics();
}

// Fill the custom range with the full min/max span of the busiest index.
async function fitToData() {
    setStatus("Fetching data range…", "loading");
    try {
        const res = await fetch("/api/es/range?index=ros-joint-states");
        const d = await res.json();
        if (d.error || d.min == null) throw new Error(d.error || "no data");
        // Convert epoch ms → 'YYYY-MM-DDTHH:MM:SS' in UTC for datetime-local.
        const toLocalInput = (ms) => new Date(ms).toISOString().slice(0, 19);
        document.getElementById("range-from").value = toLocalInput(d.min);
        document.getElementById("range-to").value = toLocalInput(d.max);
        refreshAnalytics();
    } catch (err) {
        setStatus("Error: " + err.message, "error");
    }
}
window.fitToData = fitToData;

function setStatus(text, cls) {
    const el = document.getElementById("analytics-status");
    el.textContent = text;
    el.className = "tb-status" + (cls ? " " + cls : "");
}

async function refreshAnalytics() {
    if (!analyticsBuilt) return;
    const points = document.getElementById("point-count").value;
    const range = getRangeParams();
    setStatus("Loading…", "loading");

    try {
        await Promise.all(PANELS.map((p) =>
            p.type === "line"
                ? updateLinePanel(p, range, points)
                : updateScatter3d(p, range)
        ));
        setStatus("Updated " + new Date().toLocaleTimeString(), "ok");
    } catch (err) {
        console.error(err);
        setStatus("Error: " + err.message, "error");
    }
}
window.refreshAnalytics = refreshAnalytics;

async function updateLinePanel(p, range, points) {
    const params = new URLSearchParams({
        index: p.index,
        fields: p.fields.map((f) => f.key).join(","),
        time_field: p.time_field,
        points: points,
    });
    if (range.from) params.set("from", range.from);
    if (range.to) params.set("to", range.to);

    const res = await fetch("/api/es/timeseries?" + params.toString());
    const data = await res.json();
    if (data.error) throw new Error(data.error);

    const time = data.time || [];
    const series = data.series || {};
    const chart = charts[p.id];

    p.fields.forEach((f, i) => {
        const vals = series[f.key] || [];
        chart.data.datasets[i].data = time.map((t, k) => ({ x: t, y: vals[k] }));
    });
    chart.update("none");

    toggleEmpty(p.id, time.length === 0);
}

async function updateScatter3d(p, range) {
    const params = new URLSearchParams({
        index: p.index,
        x: p.x, y: p.y, z: p.z,
        time_field: p.time_field,
        time_unit: p.time_unit || "s",
        limit: 4000,
    });
    if (range.from) params.set("from", range.from);
    if (range.to) params.set("to", range.to);

    const res = await fetch("/api/es/scatter3d?" + params.toString());
    const data = await res.json();
    if (data.error) throw new Error(data.error);

    const hasData = (data.x || []).length > 0;
    toggleEmpty(p.id, !hasData);

    const trace = {
        type: "scatter3d",
        mode: "lines+markers",
        x: data.x, y: data.y, z: data.z,
        line: { width: 3, color: data.z, colorscale: "Viridis" },
        marker: { size: 2, color: data.z, colorscale: "Viridis" },
    };
    const layout = {
        paper_bgcolor: "rgba(0,0,0,0)",
        plot_bgcolor: "rgba(0,0,0,0)",
        margin: { l: 0, r: 0, t: 0, b: 0 },
        scene: {
            xaxis: { title: "X (m)", color: "#94a3b8", gridcolor: "rgba(255,255,255,0.08)" },
            yaxis: { title: "Y (m)", color: "#94a3b8", gridcolor: "rgba(255,255,255,0.08)" },
            zaxis: { title: "Z (m)", color: "#94a3b8", gridcolor: "rgba(255,255,255,0.08)" },
        },
        font: { color: "#94a3b8" },
    };
    Plotly.react(`plot-${p.id}`, [trace], layout, { responsive: true, displaylogo: false });
}

function toggleEmpty(id, show) {
    const el = document.getElementById(`empty-${id}`);
    if (el) el.style.display = show ? "flex" : "none";
}

// ==============================================================================
// Auto-refresh
// ==============================================================================

function startAutoRefresh() {
    stopAutoRefresh();
    const interval = parseInt(document.getElementById("refresh-interval").value, 10);
    if (interval > 0 && currentTab === "analytics") {
        refreshTimer = setInterval(refreshAnalytics, interval);
    }
}

function stopAutoRefresh() {
    if (refreshTimer) {
        clearInterval(refreshTimer);
        refreshTimer = null;
    }
}

// ==============================================================================
// Wire up toolbar controls
// ==============================================================================

document.addEventListener("DOMContentLoaded", () => {
    document.getElementById("refresh-interval").addEventListener("change", startAutoRefresh);
    document.getElementById("range-preset").addEventListener("change", onPresetChange);
    document.getElementById("point-count").addEventListener("change", refreshAnalytics);
    document.getElementById("range-from").addEventListener("change", refreshAnalytics);
    document.getElementById("range-to").addEventListener("change", refreshAnalytics);
});
