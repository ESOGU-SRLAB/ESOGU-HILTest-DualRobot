/**
 * ESOGÜ Robotics Lab — Data Analytics tab
 * Read-only visualization of collected data from Elasticsearch.
 *
 * Three layers:
 *   1. Filters  — use-case chips + ad-hoc filter pills, applied to EVERY panel
 *                 on top of the time range (Kibana-style).
 *   2. Panels   — line / envelope / histogram / box / heatmap / scatter / bar,
 *                 each backed by an aggregation endpoint in app.py.
 *   3. Layout   — panels are data, not code; the user can add and remove them
 *                 and the layout is persisted in localStorage.
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

// Distinct palette for grouped (split-by) series, so a use-case breakdown never
// collides with the per-joint colours above.
const GROUP_COLORS = [
    "#6366f1", "#f97316", "#10b981", "#ef4444", "#a855f7", "#06b6d4",
];

// Fixed colors for the four known use cases (+ IDLE). Used everywhere a group
// gets colored -- chips, line/histogram/scatter split series, the insight
// banner -- so the same scenario always reads as the same color instead of
// whatever position it happened to land in that panel's group list.
const USE_CASE_COLORS = {
    PICKPLACE: "#f97316",
    HRC: "#a855f7",
    UR10E_INSPECTION: "#06b6d4",
    MULTIROBOT_INSPECTION: "#10b981",
    IDLE: "#64748b",
};

function colorForGroup(name) {
    const key = String(name);
    if (USE_CASE_COLORS[key]) return USE_CASE_COLORS[key];
    // Unknown label (a custom split_by field, or a future use case): hash it
    // to a stable index so the same value always gets the same color across
    // panels and reloads, without needing a static entry per name.
    let h = 0;
    for (let i = 0; i < key.length; i++) h = (h * 31 + key.charCodeAt(i)) >>> 0;
    return GROUP_COLORS[h % GROUP_COLORS.length];
}

const LAYOUT_STORAGE_KEY = "esogu.analytics.layout.v1";
const CUSTOM_LAYOUT_NAME = "My layout";

// UR10e joint base names, in a sensible kinematic order.
const UR10E_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "base_to_robot_mount",
];

// Indices the filter/panel editors offer before Elasticsearch has been asked.
const FALLBACK_INDICES = [
    "ros-joint-states", "ros-sim-joint-states",
    "ros-kawasaki-joint-states", "ros-tcp-pose-topic",
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

// The built-in panel set. Rows 1-5 mirror the original Grafana dashboard; the
// last two are the cross-use-case comparisons that the tagging makes possible.
function defaultPanels() {
    return [
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
        // Row 4 — UR10e efforts: real (envelope) | sim
        {
            id: "ur10e_eff_real", type: "envelope", side: "live",
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
        // Row 5 — cross-use-case comparisons (only meaningful once tagged)
        {
            id: "eff_box_by_use_case", type: "box", side: "live",
            title: "Elbow / Shoulder Effort — distribution per use case",
            index: "ros-joint-states", time_field: "@timestamp",
            unit: "Nm", split_by: "use_case",
            fields: [
                { key: "ur10e_elbow_joint.effort", label: "elbow" },
                { key: "ur10e_shoulder_lift_joint.effort", label: "shoulder lift" },
            ],
        },
        {
            id: "eff_hist_by_use_case", type: "histogram", side: "live",
            title: "Elbow Effort — histogram per use case",
            index: "ros-joint-states", time_field: "@timestamp",
            unit: "Nm", split_by: "use_case",
            fields: [{ key: "ur10e_elbow_joint.effort", label: "elbow effort" }],
        },
        // Row 6 — 3D TCP path (full width)
        {
            id: "tcp_3d", type: "scatter3d", side: "live", width: "full",
            title: "TCP Position — 3D Path",
            index: "ros-tcp-pose-topic",
            time_field: "header.sec", time_unit: "s",
            x: "pose.position.x", y: "pose.position.y", z: "pose.position.z",
        },
    ];
}

// KPI tiles across the top. Each is one /api/es/stats call.
const KPI_INDEX = "ros-joint-states";
const KPI_TIME_FIELD = "@timestamp";
const KPI_FIELDS = [
    "ur10e_elbow_joint.effort",
    "ur10e_shoulder_lift_joint.effort",
    "ur10e_elbow_joint.velocity",
];

// The auto-insight compares this one metric across use cases -- elbow effort
// is a reasonable proxy for "how hard the arm is working" across all four
// scenarios. Piggybacks on the same /api/es/stats call the KPI tiles already
// make (split_by=use_case), so it costs nothing extra.
const INSIGHT_FIELD = "ur10e_elbow_joint.effort";
const INSIGHT_LABEL = "Elbow effort";
const INSIGHT_UNIT = "Nm";
// Below this ratio the difference is called out as "not meaningful" rather
// than dressed up as a finding -- an insight banner that always claims
// something notable stops being trustworthy.
const INSIGHT_RATIO_THRESHOLD = 1.3;

// ==============================================================================
// State
// ==============================================================================

const charts = {};        // panel id → Chart.js instance
let analyticsBuilt = false;
let refreshTimer = null;
let currentTab = "home";

const state = {
    useCases: [],         // selected use-case names ([] = no use-case filter)
    knownUseCases: [],    // names offered as chips
    useCaseCounts: null,  // name → doc count in the current range
    useCaseMissing: 0,    // documents with no use_case at all
    filters: [],          // [{field, op, value}] ad-hoc filters
    panels: [],           // active layout
    layoutName: "default",
    fieldCache: {},       // index → {fields, numeric, keyword, date}
    indices: [],
};

// ==============================================================================
// Tab switching
// ==============================================================================

const TABS = ["home", "analytics", "anomaly"];

function switchTab(tab) {
    if (!TABS.includes(tab)) tab = "home";
    currentTab = tab;
    closeSideMenu();

    // Görünürlük ve buton durumu: sekme sayısından bağımsız.
    TABS.forEach((name) => {
        const view = document.getElementById("view-" + name);
        const btn = document.getElementById("tab-btn-" + name);
        if (view) view.hidden = name !== tab;
        if (btn) btn.classList.toggle("active", name === tab);
    });

    // Analytics dışına çıkılınca otomatik yenilemeyi her hâlükârda durdur.
    if (tab !== "analytics") stopAutoRefresh();

    if (tab === "analytics") {
        // Surface any build/render failure instead of silently doing nothing.
        try {
            if (typeof Chart === "undefined") {
                throw new Error("Chart.js failed to load (check internet/CDN access).");
            }
            if (!analyticsBuilt) {
                initAnalytics();
                analyticsBuilt = true;
            }
            refreshAnalytics();
            startAutoRefresh();
        } catch (err) {
            console.error("[analytics] switchTab failed:", err);
            showFatal(err.message || String(err));
        }
    } else if (tab === "anomaly") {
        // Grafik ilk açılışta kurulur; soket dinleyicisi anomaly.js içinde
        // sekmeden bağımsız bağlanır, o yüzden burada sadece kurulum var.
        try {
            if (typeof initAnomalyTab === "function") initAnomalyTab();
        } catch (err) {
            console.error("[anomaly] switchTab failed:", err);
        }
    }
}
window.switchTab = switchTab;

// ==============================================================================
// Side menu (hamburger)
// ==============================================================================
// :hover alone opens the flyout for mouse/trackpad use. This click toggle is
// the fallback for touch screens -- this panel can run on a lab kiosk display
// where :hover never fires, and without it a touch operator would have no way
// to reach Data Analytics or Anomaly Detection at all.

function toggleSideMenu() {
    const menu = document.getElementById("side-menu");
    const btn = document.getElementById("hamburger-btn");
    if (!menu || !btn) return;
    const open = menu.classList.toggle("open");
    btn.setAttribute("aria-expanded", open ? "true" : "false");
}
window.toggleSideMenu = toggleSideMenu;

function closeSideMenu() {
    const menu = document.getElementById("side-menu");
    const btn = document.getElementById("hamburger-btn");
    if (menu) menu.classList.remove("open");
    if (btn) btn.setAttribute("aria-expanded", "false");
}

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
    setStatus(message, "error");
}

// Catch any uncaught error (e.g. a failed CDN script) and make it visible.
window.addEventListener("error", (e) => {
    if (currentTab === "analytics") {
        showFatal((e.message || "Unknown error") +
            (e.filename ? " @ " + e.filename.split("/").pop() + ":" + e.lineno : ""));
    }
});

// ==============================================================================
// Initialisation
// ==============================================================================

function initAnalytics() {
    state.panels = loadLayout();
    registerCrosshairPlugin();     // must run before the first buildPanels()
    buildPanels();
    initCrosshairSync();
    populateLayoutSelect();
    loadIndices();
    loadUseCases();
    renderFilterPills();
}

// ==============================================================================
// Layout persistence
// ==============================================================================

function loadLayout() {
    try {
        const raw = localStorage.getItem(LAYOUT_STORAGE_KEY);
        if (raw) {
            const saved = JSON.parse(raw);
            if (Array.isArray(saved) && saved.length) {
                state.layoutName = CUSTOM_LAYOUT_NAME;
                return saved;
            }
        }
    } catch (err) {
        console.warn("[analytics] saved layout unreadable, using default:", err);
    }
    state.layoutName = "default";
    return defaultPanels();
}

function saveLayout() {
    try {
        localStorage.setItem(LAYOUT_STORAGE_KEY, JSON.stringify(state.panels));
        state.layoutName = CUSTOM_LAYOUT_NAME;
        populateLayoutSelect();
    } catch (err) {
        console.warn("[analytics] could not persist layout:", err);
    }
}

function populateLayoutSelect() {
    const sel = document.getElementById("layout-select");
    if (!sel) return;
    const hasCustom = !!localStorage.getItem(LAYOUT_STORAGE_KEY);
    sel.innerHTML =
        '<option value="default">Built-in default</option>' +
        (hasCustom ? `<option value="custom">${CUSTOM_LAYOUT_NAME} (saved)</option>` : "");
    sel.value = state.layoutName === CUSTOM_LAYOUT_NAME ? "custom" : "default";
}

function onLayoutChange() {
    const sel = document.getElementById("layout-select");
    if (sel.value === "default") {
        state.panels = defaultPanels();
        state.layoutName = "default";
    } else {
        state.panels = loadLayout();
    }
    buildPanels();
    refreshAnalytics();
}
window.onLayoutChange = onLayoutChange;

function resetLayout() {
    localStorage.removeItem(LAYOUT_STORAGE_KEY);
    state.panels = defaultPanels();
    state.layoutName = "default";
    populateLayoutSelect();
    buildPanels();
    refreshAnalytics();
}
window.resetLayout = resetLayout;

function removePanel(id) {
    state.panels = state.panels.filter((p) => p.id !== id);
    destroyChart(id);
    saveLayout();
    buildPanels();
    refreshAnalytics();
}
window.removePanel = removePanel;

// ==============================================================================
// Filters
// ==============================================================================

// Every request carries this: the use-case chip selection plus the ad-hoc pills.
function allFilters() {
    const out = state.filters.slice();
    if (state.useCases.length) {
        out.push({ field: "use_case", op: "one_of", value: state.useCases });
    }
    return out;
}

async function loadUseCases() {
    // Static list first, so the chips are usable even with Elasticsearch down.
    try {
        const res = await fetch("/api/es/use_cases");
        const d = await res.json();
        state.knownUseCases = d.use_cases || [];
    } catch (err) {
        state.knownUseCases = ["PICKPLACE", "MULTIROBOT_INSPECTION",
                               "UR10E_INSPECTION", "HRC", "IDLE"];
    }
    renderUseCaseChips();
    return refreshUseCaseCounts();
}

// Counts follow the current time range and the ad-hoc filters — but NOT the
// use-case selection itself. Including it would collapse every other chip to
// zero the moment you select one, which is exactly when you still need to see
// how much data the alternatives hold.
async function refreshUseCaseCounts() {
    const params = new URLSearchParams({ index: KPI_INDEX, field: "use_case", size: 50 });
    const range = getRangeParams();
    if (range.from != null) params.set("from", range.from);
    if (range.to != null) params.set("to", range.to);
    if (state.filters.length) params.set("filters", JSON.stringify(state.filters));

    try {
        const d = await getJson("/api/es/terms?" + params.toString());
        const counts = {};
        (d.values || []).forEach((v) => {
            counts[v.value] = v.count;
            if (!state.knownUseCases.includes(v.value)) state.knownUseCases.push(v.value);
        });
        state.useCaseCounts = counts;
        state.useCaseMissing = d.missing || 0;
    } catch (err) {
        console.warn("[analytics] use-case counts unavailable:", err.message);
        state.useCaseCounts = null;
        state.useCaseMissing = 0;
    }
    renderUseCaseChips();
}

function renderUseCaseChips() {
    const counts = state.useCaseCounts;
    const missing = state.useCaseMissing;
    const host = document.getElementById("use-case-chips");
    if (!host) return;
    host.innerHTML = "";

    state.knownUseCases.forEach((name) => {
        const chip = document.createElement("button");
        const active = state.useCases.includes(name);
        chip.className = "uc-chip" + (active ? " active" : "");
        const color = colorForGroup(name);
        chip.style.setProperty("--uc-color", color);
        chip.style.setProperty("--uc-bg", hexAlpha(color, 0.22));
        const n = counts ? counts[name] : undefined;
        chip.innerHTML = `<span class="uc-dot"></span>${escapeHtml(name)}` +
            (n !== undefined ? ` <small>${formatCount(n)}</small>` : "");
        if (counts && n === undefined) chip.classList.add("empty");
        chip.title = n !== undefined
            ? `${n.toLocaleString()} documents tagged ${name}`
            : `No documents tagged ${name} in the current range`;
        chip.onclick = () => toggleUseCase(name);
        host.appendChild(chip);
    });

    // Documents collected before tagging existed carry no use_case at all.
    // Say so rather than letting them look like a gap in the data.
    if (missing) {
        const note = document.createElement("span");
        note.className = "uc-missing";
        note.textContent = `${formatCount(missing)} untagged (collected before tagging)`;
        host.appendChild(note);
    }
}

function toggleUseCase(name) {
    const i = state.useCases.indexOf(name);
    if (i >= 0) state.useCases.splice(i, 1);
    else state.useCases.push(name);
    renderUseCaseChips();
    refreshAnalytics();
}

function clearUseCases() {
    state.useCases = [];
    renderUseCaseChips();
    refreshAnalytics();
}
window.clearUseCases = clearUseCases;

const OP_LABELS = {
    is: "is", is_not: "is not", one_of: "is one of", not_one_of: "is not one of",
    gt: ">", gte: "≥", lt: "<", lte: "≤", between: "between",
    exists: "exists", missing: "does not exist", contains: "contains",
};

function renderFilterPills() {
    const host = document.getElementById("filter-pills");
    const hint = document.getElementById("filter-hint");
    if (!host) return;
    host.innerHTML = "";

    state.filters.forEach((f, i) => {
        const pill = document.createElement("span");
        pill.className = "filter-pill";
        let val = "";
        if (f.op === "between" && Array.isArray(f.value)) val = f.value.join(" … ");
        else if (Array.isArray(f.value)) val = f.value.join(", ");
        else if (f.op !== "exists" && f.op !== "missing") val = String(nz(f.value, ""));
        pill.innerHTML =
            `<b>${escapeHtml(f.field)}</b> ${OP_LABELS[f.op] || f.op}` +
            (val ? ` <i>${escapeHtml(val)}</i>` : "");
        const x = document.createElement("button");
        x.className = "filter-pill-x";
        x.textContent = "✕";
        x.title = "Remove this filter";
        x.onclick = () => { state.filters.splice(i, 1); renderFilterPills(); refreshAnalytics(); };
        pill.appendChild(x);
        host.appendChild(pill);
    });

    if (hint) {
        hint.style.display = state.filters.length ? "none" : "";
    }
}

// ---- Filter editor modal ----

function openFilterEditor() {
    const modal = document.getElementById("filter-modal");
    populateIndexSelect("filter-index");
    document.getElementById("filter-field").value = "use_case";
    document.getElementById("filter-op").value = "is";
    document.getElementById("filter-value").value = "";
    document.getElementById("filter-status").textContent = "";
    onFilterOpChange();
    loadFilterFields();
    modal.classList.add("visible");
}
window.openFilterEditor = openFilterEditor;

function closeFilterEditor() {
    document.getElementById("filter-modal").classList.remove("visible");
}
window.closeFilterEditor = closeFilterEditor;

function onFilterOverlayClick(e) {
    if (e.target.id === "filter-modal") closeFilterEditor();
}
window.onFilterOverlayClick = onFilterOverlayClick;

function onFilterOpChange() {
    const op = document.getElementById("filter-op").value;
    const noValue = op === "exists" || op === "missing";
    document.getElementById("filter-value-field").style.display = noValue ? "none" : "";
    document.getElementById("filter-value2-field").style.display =
        op === "between" ? "" : "none";
}
window.onFilterOpChange = onFilterOpChange;

async function loadFilterFields() {
    const index = document.getElementById("filter-index").value;
    const meta = await fetchFields(index);
    fillDatalist("filter-field-list", meta.fields.map((f) => f.path));
    onFilterFieldChange();
}
window.loadFilterFields = loadFilterFields;

// Offer the field's real distinct values as suggestions — much faster than
// having to remember exactly how a use case is spelled.
async function onFilterFieldChange() {
    const index = document.getElementById("filter-index").value;
    const field = document.getElementById("filter-field").value.trim();
    if (!field) return;
    try {
        const res = await fetch(
            `/api/es/terms?index=${encodeURIComponent(index)}` +
            `&field=${encodeURIComponent(field)}&size=100`);
        const d = await res.json();
        fillDatalist("filter-value-list", (d.values || []).map((v) => String(v.value)));
    } catch (err) {
        fillDatalist("filter-value-list", []);
    }
}
window.onFilterFieldChange = onFilterFieldChange;

function applyFilterEditor() {
    const field = document.getElementById("filter-field").value.trim();
    const op = document.getElementById("filter-op").value;
    const raw = document.getElementById("filter-value").value.trim();
    const raw2 = document.getElementById("filter-value2").value.trim();
    const status = document.getElementById("filter-status");

    if (!field) { status.textContent = "Field is required."; return; }

    let value = raw;
    if (op === "exists" || op === "missing") {
        value = null;
    } else if (op === "between") {
        if (!raw && !raw2) { status.textContent = "Give at least one bound."; return; }
        value = [numOrString(raw), numOrString(raw2)];
    } else if (op === "one_of" || op === "not_one_of") {
        value = raw.split(",").map((v) => numOrString(v.trim())).filter((v) => v !== "");
        if (!value.length) { status.textContent = "Give a comma-separated list."; return; }
    } else {
        if (!raw) { status.textContent = "Value is required."; return; }
        value = numOrString(raw);
    }

    state.filters.push({ field, op, value });
    renderFilterPills();
    closeFilterEditor();
    refreshAnalytics();
}
window.applyFilterEditor = applyFilterEditor;

// ==============================================================================
// Field / index metadata
// ==============================================================================

async function loadIndices() {
    try {
        const res = await fetch("/api/es/indices");
        const d = await res.json();
        if (d.error) throw new Error(d.error);
        state.indices = (d.indices || []).map((i) => i.index);
    } catch (err) {
        state.indices = FALLBACK_INDICES.slice();
    }
    if (!state.indices.length) state.indices = FALLBACK_INDICES.slice();
    populateIndexSelect("filter-index");
    populateIndexSelect("pb-index");
    populateIndexSelect("discover-index");
}

function populateIndexSelect(id) {
    const sel = document.getElementById(id);
    if (!sel) return;
    const prev = sel.value;
    sel.innerHTML = state.indices
        .map((i) => `<option value="${escapeHtml(i)}">${escapeHtml(i)}</option>`)
        .join("");
    if (prev && state.indices.includes(prev)) sel.value = prev;
}

async function fetchFields(index) {
    if (state.fieldCache[index]) return state.fieldCache[index];
    let meta = { fields: [], numeric: [], keyword: [], date: [] };
    try {
        const res = await fetch(`/api/es/fields?index=${encodeURIComponent(index)}`);
        const d = await res.json();
        if (!d.error) meta = d;
    } catch (err) {
        console.warn("[analytics] field list unavailable for", index, err.message);
    }
    state.fieldCache[index] = meta;
    return meta;
}

function fillDatalist(id, values) {
    const dl = document.getElementById(id);
    if (!dl) return;
    dl.innerHTML = values.map((v) => `<option value="${escapeHtml(v)}">`).join("");
}

// ==============================================================================
// Panel construction
// ==============================================================================

function destroyChart(id) {
    if (charts[id]) {
        try { charts[id].destroy(); } catch (e) { /* already gone */ }
        delete charts[id];
    }
}

// Plotly-rendered types keep their own DOM node; Chart.js types get a canvas.
const PLOTLY_TYPES = new Set(["scatter3d", "box", "heatmap"]);

function buildPanels() {
    const grid = document.getElementById("analytics-grid");
    grid.innerHTML = "";
    Object.keys(charts).forEach(destroyChart);
    CROSSHAIR_CHARTS.clear();
    crosshairX = null;

    state.panels.forEach((p) => {
        const panel = document.createElement("div");
        panel.className = "a-panel" +
            (p.width === "full" || p.type === "scatter3d" ? " full-width" : "");

        const badgeClass = p.side === "sim" ? "sim" : "live";
        const badgeText = p.side === "sim" ? "● SIM" : "● LIVE";
        const splitNote = p.split_by
            ? `<span class="a-panel-split">split: ${escapeHtml(p.split_by)}</span>` : "";

        panel.innerHTML = `
            <div class="a-panel-header">
                <h3>${escapeHtml(p.title)}</h3>
                <div class="a-panel-tools">
                    ${splitNote}
                    <span class="a-panel-badge ${badgeClass}">${badgeText}</span>
                    <button class="a-panel-x" title="Remove this panel"
                            onclick="removePanel('${p.id}')">✕</button>
                </div>
            </div>
            <div class="a-panel-body">
                ${PLOTLY_TYPES.has(p.type)
                    ? `<div id="plot-${p.id}" style="width:100%;height:100%;"></div>`
                    : `<canvas id="canvas-${p.id}"></canvas>`}
                <div class="a-panel-empty" id="empty-${p.id}" style="display:none;">
                    No data in selected range.
                </div>
            </div>
        `;
        grid.appendChild(panel);

        if (!PLOTLY_TYPES.has(p.type)) createChartJsPanel(p);
    });
}

function baseChartOptions(p, opts = {}) {
    return {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: { mode: "nearest", axis: "x", intersect: false },
        scales: {
            x: Object.assign({
                type: "linear",
                ticks: {
                    maxTicksLimit: 8,
                    color: "#64748b",
                    callback: (v) => new Date(v).toLocaleTimeString(),
                },
                grid: { color: "rgba(255,255,255,0.04)" },
            }, opts.x || {}),
            y: Object.assign({
                title: { display: !!p.unit, text: p.unit, color: "#64748b" },
                ticks: { color: "#64748b" },
                grid: { color: "rgba(255,255,255,0.04)" },
            }, opts.y || {}),
        },
        plugins: {
            legend: {
                labels: { color: "#94a3b8", boxWidth: 12, font: { size: 10 },
                          filter: (item) => !item.text.startsWith("__") },
            },
            tooltip: Object.assign({
                callbacks: {
                    title: (items) =>
                        items.length ? new Date(items[0].parsed.x).toLocaleString() : "",
                },
            }, opts.tooltip || {}),
        },
    };
}

function createChartJsPanel(p) {
    const el = document.getElementById(`canvas-${p.id}`);
    if (!el) return;
    const ctx = el.getContext("2d");

    if (p.type === "histogram") {
        charts[p.id] = new Chart(ctx, {
            type: "bar",
            data: { labels: [], datasets: [] },
            options: baseChartOptions(p, {
                x: { type: "category",
                     title: { display: true, text: p.unit || "value", color: "#64748b" },
                     ticks: { color: "#64748b", maxTicksLimit: 12 } },
                y: { title: { display: true, text: "count", color: "#64748b" },
                     ticks: { color: "#64748b" } },
                tooltip: { callbacks: {} },
            }),
        });
        return;
    }

    if (p.type === "bar") {
        charts[p.id] = new Chart(ctx, {
            type: "bar",
            data: { labels: [], datasets: [] },
            options: baseChartOptions(p, {
                x: { type: "category", ticks: { color: "#64748b" } },
                tooltip: { callbacks: {} },
            }),
        });
        return;
    }

    if (p.type === "scatter2d") {
        charts[p.id] = new Chart(ctx, {
            type: "scatter",
            data: { datasets: [] },
            options: baseChartOptions(p, {
                x: { title: { display: true, text: p.x || "x", color: "#64748b" },
                     ticks: { color: "#64748b", callback: (v) => v } },
                y: { title: { display: true, text: p.y || "y", color: "#64748b" } },
                tooltip: { callbacks: {} },
            }),
        });
        return;
    }

    // line / envelope
    charts[p.id] = new Chart(ctx, {
        type: "line",
        data: { datasets: [] },
        options: baseChartOptions(p),
    });
    charts[p.id].$panelId = p.id;
    if (p.type === "line" || p.type === "envelope") CROSSHAIR_CHARTS.add(p.id);
}

// ==============================================================================
// Synchronized crosshair
// ==============================================================================
// Hovering any time-series panel draws a matching dashed vertical line on
// every other one at the same timestamp -- the classic Grafana "shared
// crosshair". Scoped to line/envelope panels only: histogram, box, bar and
// scatter2d don't share a time x-axis, so a "shared moment" between them
// isn't meaningful.
//
// Implementation notes:
//  - ONE delegated mousemove/mouseleave listener on the grid container
//    (not per-chart) so it survives buildPanels() tearing the DOM down and
//    rebuilding it -- no rebinding needed after a layout change.
//  - The plugin only PAINTS (afterDraw); hit-testing which chart the mouse is
//    over and converting pixel -> data-x happens in the delegated listener,
//    using that chart's own x scale. Every other chart then redraws at the
//    same data-x converted through ITS OWN scale, so panels with different
//    zoom/pan states still line up correctly.
//  - Chart.js resolves a chart's effective plugin list at construction time
//    from the plugins registered so far, so registerCrosshairPlugin() must
//    run before the first buildPanels() call.

const CROSSHAIR_CHARTS = new Set();   // panel ids opted into the sync
let crosshairX = null;                // shared hovered x (epoch ms), or null
let crosshairRaf = null;
let crosshairPluginRegistered = false;

const syncCrosshairPlugin = {
    id: "syncCrosshair",
    afterDraw(chart) {
        if (!CROSSHAIR_CHARTS.has(chart.$panelId) || crosshairX == null) return;
        const xScale = chart.scales.x;
        const area = chart.chartArea;
        if (!xScale || !area) return;
        const px = xScale.getPixelForValue(crosshairX);
        if (px < area.left || px > area.right) return;
        const ctx = chart.ctx;
        ctx.save();
        ctx.beginPath();
        ctx.setLineDash([4, 3]);
        ctx.lineWidth = 1;
        ctx.strokeStyle = "rgba(148, 163, 184, 0.6)";
        ctx.moveTo(px, area.top);
        ctx.lineTo(px, area.bottom);
        ctx.stroke();
        ctx.restore();
    },
};

function registerCrosshairPlugin() {
    if (crosshairPluginRegistered || typeof Chart === "undefined") return;
    Chart.register(syncCrosshairPlugin);
    crosshairPluginRegistered = true;
}

function initCrosshairSync() {
    const grid = document.getElementById("analytics-grid");
    if (!grid || grid.dataset.crosshairBound) return;
    grid.dataset.crosshairBound = "1";

    grid.addEventListener("mousemove", (e) => {
        const canvas = e.target.closest ? e.target.closest("canvas") : null;
        if (!canvas || !canvas.id.startsWith("canvas-")) return;
        const panelId = canvas.id.slice("canvas-".length);
        if (!CROSSHAIR_CHARTS.has(panelId)) return;
        const chart = charts[panelId];
        if (!chart || !chart.scales.x) return;
        const rect = canvas.getBoundingClientRect();
        crosshairX = chart.scales.x.getValueForPixel(e.clientX - rect.left);
        broadcastCrosshair();
    });

    grid.addEventListener("mouseleave", () => {
        crosshairX = null;
        broadcastCrosshair();
    });
}

function broadcastCrosshair() {
    if (crosshairRaf) return;
    crosshairRaf = requestAnimationFrame(() => {
        crosshairRaf = null;
        updateCrosshairLabel();
        CROSSHAIR_CHARTS.forEach((id) => {
            const c = charts[id];
            if (c) c.draw();
        });
    });
}

function updateCrosshairLabel() {
    const el = document.getElementById("crosshair-time");
    if (el) el.textContent = crosshairX != null ? new Date(crosshairX).toLocaleTimeString() : "";
}

// ==============================================================================
// Data fetching / refresh
// ==============================================================================

function getRangeParams() {
    const preset = document.getElementById("range-preset").value;
    if (preset === "all") return {};

    if (preset === "custom") {
        // datetime-local has no zone; treat it as UTC (matching the labels) and
        // send epoch milliseconds. Sending epoch consistently keeps the numeric
        // time fields (header.sec) working — an ISO string cannot be compared
        // against a `long` field.
        const from = document.getElementById("range-from").value;
        const to = document.getElementById("range-to").value;
        const r = {};
        if (from) r.from = Date.parse(from + "Z");
        if (to) r.to = Date.parse(to + "Z");
        return r;
    }

    const span = parseInt(preset, 10);
    const now = Date.now();
    return { from: now - span, to: now };
}

// Build the query string shared by every endpoint.
function panelParams(p, extra = {}) {
    const params = new URLSearchParams();
    params.set("index", p.index);
    params.set("time_field", p.time_field || "@timestamp");
    if (p.time_unit) params.set("time_unit", p.time_unit);

    const range = getRangeParams();
    if (range.from != null) params.set("from", range.from);
    if (range.to != null) params.set("to", range.to);

    const filters = allFilters();
    if (filters.length) params.set("filters", JSON.stringify(filters));

    Object.entries(extra).forEach(([k, v]) => {
        if (v !== undefined && v !== null && v !== "") params.set(k, v);
    });
    return params;
}

async function getJson(url) {
    const res = await fetch(url);
    const data = await res.json();
    if (data.error) throw new Error(data.error);
    return data;
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

// Fill the custom range with the full min/max span of the busiest index —
// honouring the active filters, so "fit to data" fits the FILTERED data.
async function fitToData() {
    setStatus("Fetching data range…", "loading");
    try {
        const params = new URLSearchParams({ index: KPI_INDEX });
        const filters = allFilters();
        if (filters.length) params.set("filters", JSON.stringify(filters));
        const d = await getJson("/api/es/range?" + params.toString());
        if (d.min == null) throw new Error("no data");
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
    if (!el) return;
    el.textContent = text;
    el.className = "tb-status" + (cls ? " " + cls : "");
}

async function refreshAnalytics() {
    if (!analyticsBuilt) return;
    const points = document.getElementById("point-count").value;
    setStatus("Loading…", "loading");

    const jobs = state.panels.map((p) => updatePanel(p, points));
    jobs.push(updateKpis());
    jobs.push(refreshUseCaseCounts());
    if (!document.getElementById("discover-panel").hidden) jobs.push(refreshDiscover());

    const results = await Promise.allSettled(jobs);
    const failed = results.filter((r) => r.status === "rejected");
    if (failed.length) {
        console.error("[analytics] panel failures:", failed.map((f) => f.reason));
        setStatus(`${failed.length} panel(s) failed: ${failed[0].reason.message}`, "error");
    } else {
        setStatus("Updated " + new Date().toLocaleTimeString(), "ok");
    }
}
window.refreshAnalytics = refreshAnalytics;

function updatePanel(p, points) {
    switch (p.type) {
        case "line":
        case "envelope":  return updateLinePanel(p, points);
        case "histogram": return updateHistogramPanel(p);
        case "box":       return updateBoxPanel(p);
        case "heatmap":   return updateHeatmapPanel(p, points);
        case "scatter2d": return updateScatter2dPanel(p);
        case "scatter3d": return updateScatter3dPanel(p);
        case "bar":       return updateBarPanel(p);
        default:          return Promise.resolve();
    }
}

function fieldKeys(p) {
    return (p.fields || []).map((f) => (typeof f === "string" ? f : f.key));
}

function fieldLabel(p, key) {
    const f = (p.fields || []).find((x) => (typeof x === "string" ? x : x.key) === key);
    if (!f) return key;
    return typeof f === "string" ? prettyLabel(f.split(".")[0]) : f.label;
}

function fieldColor(p, key, i) {
    const f = (p.fields || []).find((x) => (typeof x === "string" ? x : x.key) === key);
    if (f && typeof f !== "string" && f.color) return f.color;
    return ANALYTICS_JOINT_COLORS[i % ANALYTICS_JOINT_COLORS.length];
}

// ---- line / envelope ----

async function updateLinePanel(p, points) {
    const keys = fieldKeys(p);
    const stat = p.type === "envelope" ? "envelope" : "avg";
    const data = await getJson("/api/es/timeseries?" + panelParams(p, {
        fields: keys.join(","), points, stat, split_by: p.split_by || "",
    }).toString());

    const time = data.time || [];
    const series = data.series || {};
    const groups = data.groups || [];
    const chart = charts[p.id];
    if (!chart) return;

    const datasets = [];
    const combos = groups.length
        ? keys.flatMap((k) => groups.map((g) => ({ key: k, group: g })))
        : keys.map((k) => ({ key: k, group: null }));

    combos.forEach((c, i) => {
        const suffix = c.group === null ? "" : `||${c.group}`;
        const color = groups.length ? colorForGroup(c.group) : fieldColor(p, c.key, i);
        const label = fieldLabel(p, c.key) + (c.group === null ? "" : ` · ${c.group}`);

        if (stat === "envelope") {
            // The band is drawn first (min, then max filling down to it) so the
            // solid mean line stays on top and readable.
            const mins = series[`${c.key}::min${suffix}`] || [];
            const maxs = series[`${c.key}::max${suffix}`] || [];
            datasets.push({
                label: "__min_" + label, borderColor: "transparent",
                backgroundColor: hexAlpha(color, 0.13),
                data: time.map((t, k) => ({ x: t, y: mins[k] })),
                pointRadius: 0, borderWidth: 0, fill: false, spanGaps: true,
            });
            datasets.push({
                label: "__max_" + label, borderColor: "transparent",
                backgroundColor: hexAlpha(color, 0.13),
                data: time.map((t, k) => ({ x: t, y: maxs[k] })),
                pointRadius: 0, borderWidth: 0, fill: "-1", spanGaps: true,
            });
        }

        const avgs = series[`${c.key}::avg${suffix}`] || series[`${c.key}::${stat}${suffix}`] || [];
        datasets.push({
            label, borderColor: color, backgroundColor: color,
            data: time.map((t, k) => ({ x: t, y: avgs[k] })),
            borderWidth: 1.5, pointRadius: 0, tension: 0.15, spanGaps: true,
        });
    });

    chart.data.datasets = datasets;
    chart.update("none");
    toggleEmpty(p.id, time.length === 0);
}

// ---- histogram ----

async function updateHistogramPanel(p) {
    const field = fieldKeys(p)[0];
    const data = await getJson("/api/es/histogram?" + panelParams(p, {
        field, bins: p.bins || 40, split_by: p.split_by || "",
    }).toString());

    const chart = charts[p.id];
    if (!chart) return;
    const bins = data.bins || [];
    const groups = data.groups || [];

    chart.data.labels = bins.map((b) => formatNum(b));
    chart.data.datasets = groups.map((g) => {
        const color = g === "all" ? ANALYTICS_JOINT_COLORS[0] : colorForGroup(g);
        return {
            label: g === "all" ? fieldLabel(p, field) : g,
            data: (data.series || {})[g] || [],
            backgroundColor: hexAlpha(color, 0.65),
            borderColor: color,
            borderWidth: 1,
        };
    });
    chart.update("none");
    toggleEmpty(p.id, bins.length === 0);
}

// ---- box plot (percentiles) ----

async function updateBoxPanel(p) {
    const keys = fieldKeys(p);
    const data = await getJson("/api/es/percentiles?" + panelParams(p, {
        fields: keys.join(","), split_by: p.split_by || "",
    }).toString());

    const groups = data.groups || [];
    const traces = keys.map((key, i) => {
        const q1 = [], med = [], q3 = [], lo = [], hi = [], xs = [];
        groups.forEach((g) => {
            const v = ((data.series || {})[g] || {})[key] || {};
            if (v["50.0"] == null && v["50"] == null) return;
            xs.push(g);
            // ES keys percentiles as stringified floats ("50.0"); accept the
            // bare integer form too.
            lo.push(nz(v["5.0"], v["5"]));
            q1.push(nz(v["25.0"], v["25"]));
            med.push(nz(v["50.0"], v["50"]));
            q3.push(nz(v["75.0"], v["75"]));
            hi.push(nz(v["95.0"], v["95"]));
        });
        return {
            type: "box", name: fieldLabel(p, key), x: xs,
            q1, median: med, q3, lowerfence: lo, upperfence: hi,
            marker: { color: GROUP_COLORS[i % GROUP_COLORS.length] },
            line: { width: 1.5 },
        };
    }).filter((t) => t.x.length);

    const layout = {
        paper_bgcolor: "rgba(0,0,0,0)", plot_bgcolor: "rgba(0,0,0,0)",
        margin: { l: 48, r: 12, t: 8, b: 40 },
        boxmode: "group",
        font: { color: "#94a3b8", size: 10 },
        legend: { orientation: "h", y: 1.12 },
        xaxis: { gridcolor: "rgba(255,255,255,0.06)" },
        yaxis: { title: p.unit || "", gridcolor: "rgba(255,255,255,0.06)" },
    };
    Plotly.react(`plot-${p.id}`, traces, layout, { responsive: true, displaylogo: false });
    toggleEmpty(p.id, traces.length === 0);
}

// ---- heatmap (fields × time) ----

async function updateHeatmapPanel(p, points) {
    const keys = fieldKeys(p);
    const target = Math.min(parseInt(points, 10) || 500, 400);
    const data = await getJson("/api/es/timeseries?" + panelParams(p, {
        fields: keys.join(","), points: target, stat: "avg",
    }).toString());

    const time = data.time || [];
    const z = keys.map((k) => (data.series || {})[`${k}::avg`] || []);
    const trace = {
        type: "heatmap",
        x: time.map((t) => new Date(t).toISOString()),
        y: keys.map((k) => fieldLabel(p, k)),
        z, colorscale: "Viridis", hoverongaps: false,
        colorbar: { thickness: 10, tickfont: { size: 9 } },
    };
    const layout = {
        paper_bgcolor: "rgba(0,0,0,0)", plot_bgcolor: "rgba(0,0,0,0)",
        margin: { l: 110, r: 8, t: 8, b: 40 },
        font: { color: "#94a3b8", size: 10 },
        xaxis: { gridcolor: "rgba(255,255,255,0.06)" },
        yaxis: { gridcolor: "rgba(255,255,255,0.06)", automargin: true },
    };
    Plotly.react(`plot-${p.id}`, [trace], layout, { responsive: true, displaylogo: false });
    toggleEmpty(p.id, time.length === 0);
}

// ---- scatter 2D ----

async function updateScatter2dPanel(p) {
    const data = await getJson("/api/es/points?" + panelParams(p, {
        x: p.x, y: p.y, z: "", color: p.split_by || "", limit: p.limit || 3000,
        mode: p.mode || "spread",
    }).toString());

    const chart = charts[p.id];
    if (!chart) return;
    const xs = data.x || [], ys = data.y || [], cs = data.color || [];

    if (cs.length === xs.length && cs.length) {
        const groups = [...new Set(cs)];
        chart.data.datasets = groups.map((g) => ({
            label: String(g),
            data: xs.map((x, k) => (cs[k] === g ? { x, y: ys[k] } : null)).filter(Boolean),
            backgroundColor: hexAlpha(colorForGroup(g), 0.6),
            pointRadius: 2,
        }));
    } else {
        chart.data.datasets = [{
            label: `${p.y} vs ${p.x}`,
            data: xs.map((x, k) => ({ x, y: ys[k] })),
            backgroundColor: hexAlpha(ANALYTICS_JOINT_COLORS[0], 0.55),
            pointRadius: 2,
        }];
    }
    chart.update("none");
    toggleEmpty(p.id, xs.length === 0);
}

// ---- scatter 3D ----

async function updateScatter3dPanel(p) {
    const data = await getJson("/api/es/points?" + panelParams(p, {
        x: p.x, y: p.y, z: p.z, limit: p.limit || 4000, mode: p.mode || "spread",
    }).toString());

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

// ---- bar (compare groups) ----

async function updateBarPanel(p) {
    const keys = fieldKeys(p);
    const data = await getJson("/api/es/stats?" + panelParams(p, {
        fields: keys.join(","), split_by: p.split_by || "use_case",
    }).toString());

    const chart = charts[p.id];
    if (!chart) return;
    const groups = data.groups || [];
    const metric = p.metric || "avg";

    chart.data.labels = groups;
    chart.data.datasets = keys.map((k, i) => ({
        label: `${fieldLabel(p, k)} (${metric})`,
        data: groups.map((g) => {
            const st = ((data.by_group || {})[g] || {})[k] || {};
            return nz(st[metric], null);
        }),
        backgroundColor: hexAlpha(GROUP_COLORS[i % GROUP_COLORS.length], 0.7),
        borderColor: GROUP_COLORS[i % GROUP_COLORS.length],
        borderWidth: 1,
    }));
    chart.update("none");
    toggleEmpty(p.id, groups.length === 0);
}

function toggleEmpty(id, show) {
    const el = document.getElementById(`empty-${id}`);
    if (el) el.style.display = show ? "flex" : "none";
}

// ==============================================================================
// KPI tiles
// ==============================================================================

async function updateKpis() {
    const host = document.getElementById("kpi-row");
    if (!host) return;
    const pseudo = { index: KPI_INDEX, time_field: KPI_TIME_FIELD };
    let d;
    try {
        d = await getJson("/api/es/stats?" + panelParams(pseudo, {
            fields: KPI_FIELDS.join(","),
            split_by: "use_case",
        }).toString());
    } catch (err) {
        host.innerHTML = `<div class="kpi-tile error">Stats unavailable — ${escapeHtml(err.message)}</div>`;
        const bar = document.getElementById("insight-bar");
        if (bar) bar.hidden = true;
        return;
    }

    const tiles = [
        { label: "documents", value: formatCount(d.count || 0),
          hint: "in the current selection" },
        { label: "time span", value: formatDuration(d.duration_ms),
          hint: d.t_min ? new Date(d.t_min).toLocaleString() : "—" },
        { label: "peak elbow effort",
          value: formatNum(statOf(d.series, "ur10e_elbow_joint.effort", "max")), unit: "Nm",
          hint: "max over the selection" },
        { label: "peak shoulder effort",
          value: formatNum(statOf(d.series, "ur10e_shoulder_lift_joint.effort", "max")), unit: "Nm",
          hint: "max over the selection" },
        { label: "peak elbow velocity",
          value: formatNum(statOf(d.series, "ur10e_elbow_joint.velocity", "max")), unit: "rad/s",
          hint: "max over the selection" },
        { label: "active use case", value: state.useCases.length
            ? state.useCases.join(", ") : "all",
          hint: "click a chip above to filter" },
    ];

    host.innerHTML = tiles.map((t) => `
        <div class="kpi-tile" title="${escapeHtml(t.hint || "")}">
            <div class="kpi-label">${escapeHtml(t.label)}</div>
            <div class="kpi-value">${escapeHtml(String(t.value))}${
                t.unit ? `<span class="kpi-unit">${escapeHtml(t.unit)}</span>` : ""}</div>
        </div>`).join("");

    renderInsight(d);
}

// A plain-language read of the same use-case breakdown the KPI tiles and the
// "distribution per use case" panels already show as numbers. Picks a
// baseline (IDLE if tagged, else the lowest-average group) and the highest
// group excluding it, and only calls out a difference when it clears
// INSIGHT_RATIO_THRESHOLD -- otherwise it says so plainly instead of forcing
// a "finding" out of noise.
function renderInsight(d) {
    const bar = document.getElementById("insight-bar");
    const text = document.getElementById("insight-text");
    if (!bar || !text) return;

    const groups = (d && d.groups) || [];
    const byGroup = (d && d.by_group) || {};
    const usable = groups
        .map((g) => ({ name: g, avg: statOf(byGroup[g], INSIGHT_FIELD, "avg") }))
        .filter((g) => g.avg !== undefined && g.avg !== null);

    if (usable.length < 2) {
        text.textContent = "Not enough tagged use-case data in this range to compare yet.";
        bar.hidden = false;
        return;
    }

    const idle = usable.find((g) => g.name === "IDLE");
    const sorted = [...usable].sort((a, b) => a.avg - b.avg);
    const baseline = idle || sorted[0];
    const peak = usable.filter((g) => g.name !== baseline.name)
        .sort((a, b) => b.avg - a.avg)[0];

    const tag = (g) => `<b class="insight-uc" style="color:${colorForGroup(g.name)}">` +
        `${escapeHtml(g.name)}</b>`;

    if (!peak || baseline.avg <= 1e-6) {
        text.innerHTML = `${INSIGHT_LABEL} levels are similar across use cases in this range.`;
    } else {
        const ratio = peak.avg / baseline.avg;
        if (ratio < INSIGHT_RATIO_THRESHOLD) {
            text.innerHTML = `${INSIGHT_LABEL} levels are similar across use cases in this ` +
                `range (highest: ${tag(peak)}, ${formatNum(peak.avg)} ${INSIGHT_UNIT} avg).`;
        } else {
            text.innerHTML = `${INSIGHT_LABEL} peaks in ${tag(peak)} ` +
                `(${formatNum(peak.avg)} ${INSIGHT_UNIT} avg) — ${ratio.toFixed(1)}× higher ` +
                `than ${tag(baseline)} (${formatNum(baseline.avg)} ${INSIGHT_UNIT} avg).`;
        }
    }
    bar.hidden = false;
}

// ==============================================================================
// Discover table
// ==============================================================================

function toggleDiscover() {
    const on = document.getElementById("toggle-discover").checked;
    document.getElementById("discover-panel").hidden = !on;
    if (on) refreshDiscover();
}
window.toggleDiscover = toggleDiscover;

function discoverPseudoPanel() {
    const index = document.getElementById("discover-index").value || KPI_INDEX;
    // header.sec indices are not date-based; pick a time field that exists.
    const time_field = index === "ros-tcp-pose-topic" ? "header.sec" : "@timestamp";
    const time_unit = index === "ros-tcp-pose-topic" ? "s" : "ms";
    return { index, time_field, time_unit };
}

async function refreshDiscover() {
    const panel = document.getElementById("discover-panel");
    if (!panel || panel.hidden) return;
    const size = document.getElementById("discover-size").value;
    const table = document.getElementById("discover-table");
    const totalEl = document.getElementById("discover-total");

    try {
        const d = await getJson("/api/es/docs?" +
            panelParams(discoverPseudoPanel(), { size }).toString());
        const cols = d.columns || [];
        table.querySelector("thead").innerHTML =
            "<tr>" + cols.map((c) => `<th>${escapeHtml(c)}</th>`).join("") + "</tr>";
        table.querySelector("tbody").innerHTML = (d.rows || []).length
            ? d.rows.map((r) => "<tr>" + r.map((v) =>
                `<td>${escapeHtml(formatCell(v))}</td>`).join("") + "</tr>").join("")
            : `<tr><td class="a-empty-cell" colspan="${cols.length || 1}">No documents in range.</td></tr>`;
        totalEl.textContent = `${formatCount(d.total || 0)} matching`;
    } catch (err) {
        totalEl.textContent = "error: " + err.message;
        throw err;
    }
}
window.refreshDiscover = refreshDiscover;

function exportDiscoverCsv() {
    const size = document.getElementById("discover-size").value;
    const url = "/api/es/docs?" +
        panelParams(discoverPseudoPanel(), { size, format: "csv" }).toString();
    window.open(url, "_blank");
}
window.exportDiscoverCsv = exportDiscoverCsv;

// ==============================================================================
// Panel builder
// ==============================================================================

function openPanelBuilder() {
    populateIndexSelect("pb-index");
    document.getElementById("pb-title").value = "";
    document.getElementById("pb-fields").value = "";
    document.getElementById("pb-status").textContent = "";
    document.getElementById("pb-split").value = "";
    onPanelTypeChange();
    loadPanelFields();
    document.getElementById("panel-modal").classList.add("visible");
}
window.openPanelBuilder = openPanelBuilder;

function closePanelBuilder() {
    document.getElementById("panel-modal").classList.remove("visible");
}
window.closePanelBuilder = closePanelBuilder;

function onPanelOverlayClick(e) {
    if (e.target.id === "panel-modal") closePanelBuilder();
}
window.onPanelOverlayClick = onPanelOverlayClick;

const PANEL_HINTS = {
    line: "Average of each field over time.",
    envelope: "Average plus a min/max band — use this when spikes matter; a plain average hides them.",
    histogram: "Distribution of ONE numeric field. Split by use_case to compare scenarios.",
    box: "Percentile box (5/25/50/75/95) per group. Split by use_case for the scenario comparison.",
    heatmap: "All listed fields as rows, time as columns. Good for seeing which joint is busy when.",
    scatter2d: "One field against another — correlation. Split-by colours the points.",
    scatter3d: "Raw x/y/z path, e.g. the TCP trajectory.",
    bar: "One bar per group (split-by), height = average of each field.",
};

function onPanelTypeChange() {
    const t = document.getElementById("pb-type").value;
    const xyz = t === "scatter2d" || t === "scatter3d";
    document.getElementById("pb-fields-field").style.display = xyz ? "none" : "";
    document.getElementById("pb-x-field").style.display = xyz ? "" : "none";
    document.getElementById("pb-y-field").style.display = xyz ? "" : "none";
    document.getElementById("pb-z-field").style.display = t === "scatter3d" ? "" : "none";
    document.getElementById("pb-hint").textContent = PANEL_HINTS[t] || "";
}
window.onPanelTypeChange = onPanelTypeChange;

async function loadPanelFields() {
    const index = document.getElementById("pb-index").value;
    const meta = await fetchFields(index);
    fillDatalist("pb-field-list", meta.fields.map((f) => f.path));

    // Offer only fields that can actually serve as a time axis.
    const timeSel = document.getElementById("pb-time-field");
    const candidates = [...(meta.date || []), ...(meta.numeric || [])
        .filter((f) => /sec$|time|stamp/i.test(f))];
    const list = candidates.length ? candidates : ["@timestamp"];
    timeSel.innerHTML = list
        .map((f) => `<option value="${escapeHtml(f)}">${escapeHtml(f)}</option>`).join("");
    if (list.includes("@timestamp")) timeSel.value = "@timestamp";
}
window.loadPanelFields = loadPanelFields;

function applyPanelBuilder() {
    const status = document.getElementById("pb-status");
    const type = document.getElementById("pb-type").value;
    const index = document.getElementById("pb-index").value;
    const timeField = document.getElementById("pb-time-field").value || "@timestamp";
    const title = document.getElementById("pb-title").value.trim();
    const split = document.getElementById("pb-split").value.trim();
    const unit = document.getElementById("pb-unit").value.trim();
    const width = document.getElementById("pb-width").value;
    const xyz = type === "scatter2d" || type === "scatter3d";

    const meta = state.fieldCache[index] || { date: [] };
    // header.sec-style fields hold epoch SECONDS; the backend needs to be told.
    const timeUnit = (meta.date || []).includes(timeField) ? "ms" : "s";

    const panel = {
        id: "user_" + Date.now().toString(36),
        type, index, title: title || `${type} — ${index}`,
        time_field: timeField, time_unit: timeUnit,
        unit, width, side: index.includes("sim") ? "sim" : "live",
    };

    if (xyz) {
        panel.x = document.getElementById("pb-x").value.trim();
        panel.y = document.getElementById("pb-y").value.trim();
        panel.z = document.getElementById("pb-z").value.trim();
        if (!panel.x || !panel.y) { status.textContent = "X and Y are required."; return; }
        if (type === "scatter3d" && !panel.z) { status.textContent = "Z is required."; return; }
        if (split) panel.split_by = split;
    } else {
        const fields = document.getElementById("pb-fields").value
            .split(",").map((f) => f.trim()).filter(Boolean);
        if (!fields.length) { status.textContent = "At least one field is required."; return; }
        if (type === "histogram" && fields.length > 1) {
            status.textContent = "A histogram takes exactly one field.";
            return;
        }
        if ((type === "box" || type === "bar") && !split) {
            status.textContent = "This chart needs a split-by field (try use_case).";
            return;
        }
        panel.fields = fields.map((f, i) => ({
            key: f, label: prettyLabel(f.split(".").slice(-2).join(" ")),
            color: ANALYTICS_JOINT_COLORS[i % ANALYTICS_JOINT_COLORS.length],
        }));
        if (split) panel.split_by = split;
    }

    state.panels.push(panel);
    saveLayout();
    closePanelBuilder();
    buildPanels();
    refreshAnalytics();
}
window.applyPanelBuilder = applyPanelBuilder;

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
// Small helpers
// ==============================================================================

// Null-coalescing helpers. The rest of this dashboard targets plain ES2017,
// so `??` / `?.` are spelled out rather than used.
function nz(v, fallback) {
    return (v === null || v === undefined) ? fallback : v;
}

function statOf(series, field, key) {
    const s = series ? series[field] : null;
    return s ? s[key] : undefined;
}

function escapeHtml(s) {
    return String(nz(s, "")).replace(/[&<>"']/g, (c) => ({
        "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
    }[c]));
}

function numOrString(v) {
    if (v === "" || v === null || v === undefined) return v;
    const n = Number(v);
    return Number.isFinite(n) && String(n) === String(v).trim() ? n : v;
}

function hexAlpha(hex, a) {
    const m = /^#?([0-9a-f]{6})$/i.exec(hex || "");
    if (!m) return hex;
    const n = parseInt(m[1], 16);
    return `rgba(${(n >> 16) & 255}, ${(n >> 8) & 255}, ${n & 255}, ${a})`;
}

function formatNum(v) {
    if (v === null || v === undefined || Number.isNaN(v)) return "—";
    const a = Math.abs(v);
    if (a !== 0 && (a < 0.01 || a >= 1e6)) return v.toExponential(2);
    return Number(v).toFixed(a >= 100 ? 1 : 3).replace(/\.?0+$/, "");
}

function formatCount(n) {
    if (n === null || n === undefined) return "—";
    if (n >= 1e9) return (n / 1e9).toFixed(1) + "B";
    if (n >= 1e6) return (n / 1e6).toFixed(1) + "M";
    if (n >= 1e3) return (n / 1e3).toFixed(1) + "k";
    return String(n);
}

function formatDuration(ms) {
    if (!ms || ms <= 0) return "—";
    const s = ms / 1000;
    if (s < 60) return s.toFixed(1) + " s";
    if (s < 3600) return Math.floor(s / 60) + "m " + Math.round(s % 60) + "s";
    if (s < 86400) return Math.floor(s / 3600) + "h " + Math.round((s % 3600) / 60) + "m";
    return (s / 86400).toFixed(1) + " days";
}

function formatCell(v) {
    if (v === null || v === undefined) return "—";
    if (typeof v === "number") return formatNum(v);
    const s = String(v);
    return s.length > 60 ? s.slice(0, 57) + "…" : s;
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

    const hamburger = document.getElementById("hamburger-btn");
    if (hamburger) hamburger.addEventListener("click", toggleSideMenu);

    // Click outside the menu closes it (only matters in the touch/.open
    // path -- hover mode already closes itself when the pointer leaves).
    document.addEventListener("click", (e) => {
        const menu = document.getElementById("side-menu");
        if (menu && menu.classList.contains("open") && !menu.contains(e.target)) {
            closeSideMenu();
        }
    });

    document.addEventListener("keydown", (e) => {
        if (e.key !== "Escape") return;
        closeFilterEditor();
        closePanelBuilder();
        closeSideMenu();
    });
});
