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

// The six revolute arm joints. base_to_robot_mount is the prismatic mount axis:
// on the real robot the UR driver does not own it (effort is always 0), and in
// sim its effort is a force in N that reaches ±1000+ and drowns every torque
// on a shared axis. Effort panels use this list; position/velocity keep the mount.
const UR10E_ARM_JOINTS = UR10E_JOINTS.filter((j) => j !== "base_to_robot_mount");

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
function ur10eFields(prefix, suffix, joints = UR10E_JOINTS) {
    return joints.map((j, i) => ({
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
            id: "ur10e_pos_real", type: "line", side: "live", pair: "ur10e_pos",
            title: "UR10e — Joint Positions (Real)",
            index: "ros-joint-states",
            unit: "rad", fields: ur10eFields("ur10e_", "position"),
        },
        {
            id: "ur10e_pos_sim", type: "line", side: "sim", pair: "ur10e_pos",
            title: "UR10e — Joint Positions (Sim)",
            index: "ros-sim-joint-states",
            unit: "rad", fields: ur10eFields("sim_ur10e_", "position"),
        },
        // Row 2 — Kawasaki + AGV positions: real | sim
        {
            id: "kawa_pos_real", type: "line", side: "live", pair: "kawa_pos",
            title: "Kawasaki — Joint Positions (Real)",
            index: "ros-kawasaki-joint-states",
            unit: "rad",
            fields: [1, 2, 3, 4, 5, 6].map((n, i) => ({
                key: `joint${n}.position`, label: `joint ${n}`,
                color: ANALYTICS_JOINT_COLORS[i % ANALYTICS_JOINT_COLORS.length],
            })),
        },
        {
            id: "kawa_pos_sim", type: "line", side: "sim", pair: "kawa_pos",
            title: "Kawasaki + AGV — Joint Positions (Sim)",
            index: "ros-sim-joint-states",
            unit: "rad",
            fields: [
                ...[1, 2, 3, 4, 5, 6].map((n, i) => ({
                    key: `sim_kawasaki_joint${n}.position`, label: `joint ${n}`,
                    color: ANALYTICS_JOINT_COLORS[i % ANALYTICS_JOINT_COLORS.length],
                })),
                // sim_ prefix, like every other series on this panel. The sim
                // joint-state documents carry sim_world_to_agv; plain
                // world_to_agv only exists on ros-agv-joint-states, which this
                // panel does not read — so the unprefixed key drew nothing.
                { key: "sim_world_to_agv.position", label: "AGV", color: ANALYTICS_JOINT_COLORS[6] },
            ],
        },
        // Row 3 — UR10e velocities: real | sim
        {
            id: "ur10e_vel_real", type: "line", side: "live", pair: "ur10e_vel",
            title: "UR10e — Joint Velocities (Real)",
            index: "ros-joint-states",
            unit: "rad/s", fields: ur10eFields("ur10e_", "velocity"),
        },
        {
            id: "ur10e_vel_sim", type: "line", side: "sim", pair: "ur10e_vel",
            title: "UR10e — Joint Velocities (Sim)",
            index: "ros-sim-joint-states",
            unit: "rad/s", fields: ur10eFields("sim_ur10e_", "velocity"),
        },
        // Row 4 — UR10e effort. NOT a like-for-like pair, so deliberately no
        // `pair` key: the UR driver writes motor CURRENT (A, "actual_current")
        // into JointState.effort, while gz_ros2_control reports joint TORQUE
        // (Nm). Forcing them onto one y-axis flattened the real panel to zero.
        {
            id: "ur10e_eff_real", type: "envelope", side: "live",
            title: "UR10e — Joint Currents (Real)",
            index: "ros-joint-states",
            unit: "A", fields: ur10eFields("ur10e_", "effort", UR10E_ARM_JOINTS),
        },
        {
            id: "ur10e_eff_sim", type: "line", side: "sim",
            title: "UR10e — Joint Torques (Sim)",
            index: "ros-sim-joint-states",
            unit: "Nm", fields: ur10eFields("sim_ur10e_", "effort", UR10E_ARM_JOINTS),
        },
        // Row 5 — cross-use-case comparisons (only meaningful once tagged)
        {
            id: "eff_box_by_use_case", type: "box", side: "live",
            title: "Elbow / Shoulder Current — distribution per use case",
            index: "ros-joint-states",
            unit: "A", split_by: "use_case",
            fields: [
                { key: "ur10e_elbow_joint.effort", label: "elbow" },
                { key: "ur10e_shoulder_lift_joint.effort", label: "shoulder lift" },
            ],
        },
        {
            id: "eff_hist_by_use_case", type: "histogram", side: "live",
            title: "Elbow Current — histogram per use case",
            index: "ros-joint-states",
            unit: "A", split_by: "use_case",
            fields: [{ key: "ur10e_elbow_joint.effort", label: "elbow current" }],
        },
        // Row 6 — 3D TCP path (full width)
        {
            id: "tcp_3d", type: "scatter3d", side: "live", width: "full",
            title: "TCP Position — 3D Path",
            index: "ros-tcp-pose-topic",
            // The cell chassis, read from the same STLs the robot description
            // uses, drawn in the TCP pose's own frame. A path in mid-air says
            // nothing; against the part being worked on it is readable.
            mesh: "chassis",
            x: "pose.position.x", y: "pose.position.y", z: "pose.position.z",
        },
    ];
}

// Default index for the parts of the UI that need one before the user has
// picked (Discover, the query preview). Not a claim that it is special.
const KPI_INDEX = "ros-joint-states";

// A newest document older than this makes the archive worth calling stale in
// the insight bar rather than presenting it as if it were live.
const STALE_AFTER_DAYS = 7;
// Below this ratio the difference is called out as "not meaningful" rather
// than dressed up as a finding -- an insight banner that always claims
// something notable stops being trustworthy.

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
    timeFieldCache: {},   // index → {time_field, time_unit, usable, candidates}
    indices: [],
    q: "",                // query bar (Lucene / KQL comparisons)
    dsl: "",              // raw DSL fragment, AND-ed into bool.filter
    discover: {
        columns: [],      // [] = use the backend's default set
        defaultColumns: [],
        page: 0,
        sort: null,
        order: "desc",
        total: 0,
        timeField: null,
        expanded: {},     // row index → open
        summaries: {},    // field path → /api/es/field_summary result
    },
};

// ==============================================================================
// Tab switching
// ==============================================================================

const TABS = ["home", "analytics", "anomaly", "freemove"];

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
    } else if (tab === "freemove") {
        // 3D sahne bir kere kurulur; soket dinleyicileri de freemove.js içinde
        // sekmeden bağımsız bağlanır (Enable/Disable ise operatör kararı).
        try {
            if (typeof initFreeMove === "function") initFreeMove();
        } catch (err) {
            console.error("[freemove] switchTab failed:", err);
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

// A layout saved before a built-in panel gained a property would keep the old
// shape for ever: the chassis underlay and the live|sim pair keys were both
// added after layouts started being persisted, so anyone who had ever touched
// the panel set silently kept panels without them. Merging the built-in
// definition in UNDERNEATH the saved copy adopts new features while leaving
// every field the user actually changed alone. An explicit null still wins, so
// switching a feature off stays switched off.
//
// Built-in fields that were WRONG in earlier builds. For these the built-in
// definition wins over a saved copy (a key absent from the built-in is dropped,
// which is how the old `pair` goes away). Built-in panels cannot be edited in
// the UI, so a saved value here is only a snapshot of the old default, never a
// user choice. Real UR10e effort is motor current (A), not torque (Nm).
const CORRECTED_BUILTIN_KEYS = {
    ur10e_eff_real: ["title", "unit", "pair", "fields"],
    ur10e_eff_sim: ["title", "unit", "pair", "fields"],
    eff_box_by_use_case: ["title", "unit"],
    eff_hist_by_use_case: ["title", "unit", "fields"],
};

function migratePanels(saved) {
    const builtin = {};
    defaultPanels().forEach((p) => { builtin[p.id] = p; });
    return saved.map((p) => {
        const b = builtin[p.id];
        if (!b) return p;
        const merged = Object.assign({}, b, p);
        (CORRECTED_BUILTIN_KEYS[p.id] || []).forEach((k) => {
            if (k in b) merged[k] = b[k];
            else delete merged[k];
        });
        return merged;
    });
}

function loadLayout() {
    try {
        const raw = localStorage.getItem(LAYOUT_STORAGE_KEY);
        if (raw) {
            const saved = JSON.parse(raw);
            if (Array.isArray(saved) && saved.length) {
                state.layoutName = CUSTOM_LAYOUT_NAME;
                return migratePanels(saved);
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

// Which field this index can be plotted against. Asked once per index and
// cached; the answer is what every panel, the Discover table and the panel
// builder now use instead of assuming "@timestamp".
async function fetchTimeField(index) {
    if (state.timeFieldCache[index]) return state.timeFieldCache[index];
    let r = { time_field: null, time_unit: "ms", usable: false, candidates: [] };
    try {
        const res = await fetch(`/api/es/time_field?index=${encodeURIComponent(index)}`);
        const d = await res.json();
        if (!d.error) r = d;
    } catch (err) {
        console.warn("[analytics] time field unavailable for", index, err.message);
    }
    state.timeFieldCache[index] = r;
    return r;
}

// The unit a chosen time field is STORED in — a date field is epoch millis, a
// header.sec-style long is epoch seconds, and the backend has to be told which.
function unitForField(index, field) {
    const known = ((state.timeFieldCache[index] || {}).candidates || [])
        .find((c) => c.path === field);
    if (known) return known.unit;
    const meta = state.fieldCache[index] || {};
    return (meta.date || []).includes(field) ? "ms" : "s";
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
                    <button class="a-panel-x" title="Open ${escapeHtml(p.index)} in Discover"
                            onclick="drillToDiscover('${p.id}')">🔎</button>
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
            ${p.mesh ? `<div class="a-panel-note" id="chassis-note-${p.id}"></div>` : ""}
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
                onClick: isolateSeries,
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
    // Only sent when a panel deliberately overrides it. Left out, the backend
    // resolves the field that actually carries values in this index — the UI
    // used to assume "@timestamp", which silently emptied every index the
    // ingest never stamped.
    if (p.time_field) params.set("time_field", p.time_field);
    if (p.time_unit) params.set("time_unit", p.time_unit);

    const range = getRangeParams();
    if (range.from != null) params.set("from", range.from);
    if (range.to != null) params.set("to", range.to);

    const filters = allFilters();
    if (filters.length) params.set("filters", JSON.stringify(filters));
    if (state.q) params.set("q", state.q);
    if (state.dsl) params.set("dsl", state.dsl);

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
    applyPairedScales();
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
    const [data, chassis] = await Promise.all([
        getJson("/api/es/points?" + panelParams(p, {
            x: p.x, y: p.y, z: p.z, limit: p.limit || 4000, mode: p.mode || "spread",
        }).toString()),
        fetchChassisMesh(p),
    ]);

    const hasData = (data.x || []).length > 0;
    toggleEmpty(p.id, !hasData);

    const trace = {
        type: "scatter3d",
        mode: "lines+markers",
        name: "TCP path",
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
    const drawable = chassis && !chassis.error;
    const traces = drawable ? [chassisTrace(chassis), trace] : [trace];
    Plotly.react(`plot-${p.id}`, traces, layout, { responsive: true, displaylogo: false });
    renderChassisNote(p, chassis);
}

// ---- cell geometry underlay ----

// The chassis is fixed in the cell while the robot rides the linear axis, so in
// the TCP pose's frame the chassis position depends on where that axis was.
// The backend picks the median over the selected range and reports the spread;
// the note under the panel says which, because a wide spread means the outline
// is only representative.
async function fetchChassisMesh(p) {
    if (p.mesh !== "chassis") return null;
    const params = new URLSearchParams();
    const range = getRangeParams();
    if (range.from != null) params.set("from", range.from);
    if (range.to != null) params.set("to", range.to);
    if (p.mesh_q !== undefined && p.mesh_q !== "") params.set("q", p.mesh_q);
    try {
        return await getJson("/api/mesh/chassis?" + params.toString());
    } catch (err) {
        console.warn("[analytics] chassis mesh unavailable:", err.message);
        return { error: err.message };
    }
}

function chassisTrace(m) {
    return {
        type: "mesh3d",
        name: "Chassis",
        x: m.x, y: m.y, z: m.z,
        i: m.i, j: m.j, k: m.k,
        color: "#8fa3c8",
        // 0.28 was too faint to read against the dark scene once WebGL had
        // blended 16 k translucent triangles over each other.
        opacity: 0.5,
        flatshading: true,
        // The path is what a cursor should find, not the scenery behind it.
        hoverinfo: "skip",
        showscale: false,
        lighting: { ambient: 0.85, diffuse: 0.6, specular: 0.05 },
    };
}

function renderChassisNote(p, m) {
    const el = document.getElementById(`chassis-note-${p.id}`);
    if (!el) return;
    if (!m) { el.textContent = ""; return; }
    if (m.error) {
        el.textContent = "Chassis outline unavailable: " + m.error;
        el.classList.add("note-err");
        return;
    }
    el.classList.remove("note-err");

    const spread = (m.q_p95 != null && m.q_p5 != null) ? m.q_p95 - m.q_p5 : null;
    let text = `Chassis: ${formatCount(m.triangles)} tris from ${m.parts} STL parts`;
    if (m.q_source === "median") {
        text += ` · linear axis ${m.q.toFixed(3)} m (median)`;
        // Half a metre of travel moves the chassis visibly against the path.
        if (spread !== null && spread > 0.05) {
            text += ` — axis moved ${m.q_min.toFixed(2)}–${m.q_max.toFixed(2)} m ` +
                    "in this range, so the outline is indicative";
        }
    } else if (m.q_source === "requested") {
        text += ` · linear axis pinned at ${m.q.toFixed(3)} m`;
    } else {
        text += " · no joint states in range, drawn at axis origin";
    }
    el.textContent = text;
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

// Pure: overview payload -> the tiles to render. Split out from updateKpis so
// the wording and the arithmetic can be exercised without a DOM.
function kpiTiles(d) {
    const t = (d && d.totals) || {};
    const docs = t.docs || 0;
    const days = t.span_ms ? Math.max(1, Math.round(t.span_ms / 86400000)) : null;
    const span = days === null ? "—" : `${days} ${days === 1 ? "day" : "days"}`;
    const dateRange = (t.t_min && t.t_max)
        ? `${shortDate(t.t_min)} → ${shortDate(t.t_max)}` : "no timestamps";

    // `tagged` is counted inside the current range, so when one is selected the
    // whole-corpus total is the wrong denominator.
    const tagBase = d && d.ranged ? (t.in_range || 0) : docs;
    const tagPct = tagBase ? (t.tagged / tagBase) * 100 : 0;

    return [
        { label: "documents", value: formatCount(docs),
          hint: `${docs.toLocaleString()} rows across every index` },
        { label: "topics", value: String(t.indices || 0),
          hint: `${t.usable || 0} ${t.usable === 1 ? "carries" : "carry"} ` +
                "a usable time field" +
                (t.unusable
                    ? `, ${t.unusable} ${t.unusable === 1 ? "does" : "do"} not`
                    : "") },
        { label: "on disk", value: formatBytes(t.bytes || 0),
          hint: "primary + replica store size" },
        { label: "coverage", value: span, hint: dateRange },
        { label: d && d.ranged ? "in range" : "plottable",
          value: formatCount(t.in_range || 0),
          hint: d && d.ranged
              ? "documents inside the selected window"
              : "documents with a usable timestamp — the rest cannot be charted" },
        { label: "use-case tagged",
          value: tagPct >= 0.05 ? tagPct.toFixed(1) + "%" : "none",
          hint: `${(t.tagged || 0).toLocaleString()} of ${tagBase.toLocaleString()} ` +
                `documents${d && d.ranged ? " in the selected window" : ""}` },
    ];
}

// The tiles describe the archive, not one joint. They used to report the peak
// effort of ur10e_elbow_joint over the selection, which is a number about a
// single joint of a single robot dressed up as a headline about the data set.
async function updateKpis() {
    const host = document.getElementById("kpi-row");
    if (!host) return;

    const params = new URLSearchParams();
    const range = getRangeParams();
    if (range.from != null) params.set("from", range.from);
    if (range.to != null) params.set("to", range.to);

    let d;
    try {
        d = await getJson("/api/es/overview?" + params.toString());
    } catch (err) {
        host.innerHTML = `<div class="kpi-tile error">Overview unavailable — ${escapeHtml(err.message)}</div>`;
        const bar = document.getElementById("insight-bar");
        if (bar) bar.hidden = true;
        return;
    }

    host.innerHTML = kpiTiles(d).map((t2) => `
        <div class="kpi-tile" title="${escapeHtml(t2.hint || "")}">
            <div class="kpi-label">${escapeHtml(t2.label)}</div>
            <div class="kpi-value">${escapeHtml(String(t2.value))}${
                t2.unit ? `<span class="kpi-unit">${escapeHtml(t2.unit)}</span>` : ""}</div>
            <div class="kpi-hint">${escapeHtml(t2.hint || "")}</div>
        </div>`).join("");

    renderInsight(d);
}

function shortDate(ms) {
    return new Date(ms).toLocaleDateString(undefined,
        { year: "numeric", month: "short", day: "numeric" });
}

function formatBytes(n) {
    if (!n) return "0 B";
    const u = ["B", "kB", "MB", "GB", "TB"];
    const i = Math.min(u.length - 1, Math.floor(Math.log10(n) / 3));
    return (n / Math.pow(1000, i)).toFixed(i ? 1 : 0) + " " + u[i];
}

// One line about the state of the archive, picked from what is actually worth
// saying rather than always manufacturing a comparison. Ordered by how much it
// should change what the reader does next.
function renderInsight(d) {
    const bar = document.getElementById("insight-bar");
    const text = document.getElementById("insight-text");
    if (!bar || !text) return;

    const t = (d && d.totals) || {};
    const rows = (d && d.indices) || [];
    if (!t.docs) {
        text.textContent = "Elasticsearch holds no collected data yet.";
        bar.hidden = false;
        return;
    }

    const b = (x) => `<b>${escapeHtml(String(x))}</b>`;
    const notes = [];

    if (!t.tagged) {
        notes.push(`None of the ${b(t.docs.toLocaleString())} collected documents ` +
            "carry a <code>use_case</code> yet, so the per-scenario panels stay " +
            "empty until the next collection run.");
    }

    const dead = rows.filter((r) => r.docs && !r.usable).map((r) => r.index);
    if (dead.length) {
        const many = dead.length > 1;
        notes.push(`${b(dead.length)} topic${many ? "s" : ""} ` +
            `(${escapeHtml(dead.join(", "))}) ` +
            `${many ? "carry" : "carries"} no field that can serve as a time axis, ` +
            `so ${many ? "they" : "it"} cannot be charted.`);
    }

    if (t.t_max) {
        const ageDays = (Date.now() - t.t_max) / 86400000;
        if (ageDays > STALE_AFTER_DAYS) {
            notes.push(`The newest document is ${b(Math.round(ageDays) + " days")} old — ` +
                "a recent-window preset will look empty.");
        }
    }

    const top = rows[0];
    if (top && top.docs / t.docs > 0.5) {
        notes.push(`${b(top.index)} alone is ` +
            `${((top.docs / t.docs) * 100).toFixed(0)}% of everything collected.`);
    }

    if (!notes.length) {
        const gb = (t.bytes / 1e9).toFixed(1);
        notes.push(`${b(t.docs.toLocaleString())} documents across ` +
            `${b(t.indices)} topics, ${gb} GB on disk.`);
    }

    text.innerHTML = notes[0] +
        (notes.length > 1
            ? ` <span class="insight-more">+${notes.length - 1} more</span>`
            : "");
    text.title = notes.map((n) => n.replace(/<[^>]+>/g, "")).join("\n\n");
    bar.hidden = false;
}

// ==============================================================================
// Query bar
// ==============================================================================
//
// Two levels on purpose. The one-line bar covers the common case in Lucene
// syntax (with KQL-style `field > 5` comparisons rewritten server-side, since
// that is what everyone types first and plain Lucene answers it with a silent
// zero). The DSL box is the escape hatch: a hand-written clause, AND-ed into
// the same bool.filter, for everything the bar cannot say.

function applyQueryBar() {
    state.q = document.getElementById("query-bar").value.trim();
    state.discover.page = 0;
    refreshAnalytics();
}
window.applyQueryBar = applyQueryBar;

function toggleDslBox() {
    const row = document.getElementById("dsl-row");
    row.hidden = !row.hidden;
    document.getElementById("btn-dsl").classList.toggle("active", !row.hidden);
    if (!row.hidden) document.getElementById("query-dsl").focus();
}
window.toggleDslBox = toggleDslBox;

function applyDsl() {
    const raw = document.getElementById("query-dsl").value.trim();
    const status = document.getElementById("dsl-status");
    if (raw) {
        // Fail here rather than as eight identical panel errors.
        try {
            const parsed = JSON.parse(raw);
            if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) {
                throw new Error("expected a JSON object holding one query clause");
            }
        } catch (err) {
            status.textContent = "✕ " + err.message;
            status.classList.add("err");
            return;
        }
    }
    status.classList.remove("err");
    status.innerHTML = raw
        ? "✅ Applied — AND-ed into <code>bool.filter</code>."
        : "AND-ed into <code>bool.filter</code>. Read-only: no aggs, no scripts.";
    state.dsl = raw;
    state.discover.page = 0;
    refreshAnalytics();
}
window.applyDsl = applyDsl;

function clearDsl() {
    document.getElementById("query-dsl").value = "";
    applyDsl();
}
window.clearDsl = clearDsl;

// ------------------------------------------------------------------------------
// "Show query" — the DSL the whole page is actually sending
// ------------------------------------------------------------------------------

let lastQueryPreview = null;

async function showQueryPreview() {
    const modal = document.getElementById("query-modal");
    const pre = document.getElementById("query-json");
    modal.classList.add("open");
    document.getElementById("query-copy-status").textContent = "";
    pre.textContent = "loading…";
    try {
        const index = document.getElementById("discover-panel").hidden
            ? KPI_INDEX : discoverIndex();
        const d = await getJson("/api/es/query_preview?" +
            panelParams({ index }).toString());
        lastQueryPreview = d;
        pre.textContent = JSON.stringify(d.body, null, 2);
        document.getElementById("query-modal-desc").textContent =
            `Index ${d.index}, time field ${d.time_field} (${d.time_unit}). ` +
            "Range, use-case chips, filter pills and the query bar all fold into " +
            "this one read-only query.";
    } catch (err) {
        pre.textContent = "error: " + err.message;
    }
}
window.showQueryPreview = showQueryPreview;

function closeQueryPreview() {
    document.getElementById("query-modal").classList.remove("open");
}
window.closeQueryPreview = closeQueryPreview;

function onQueryOverlayClick(e) {
    if (e.target.id === "query-modal") closeQueryPreview();
}
window.onQueryOverlayClick = onQueryOverlayClick;

function copyQueryText(kind) {
    if (!lastQueryPreview) return;
    const text = kind === "curl"
        ? lastQueryPreview.curl
        : JSON.stringify(lastQueryPreview.body, null, 2);
    copyText(text, kind === "curl" ? "curl copied." : "JSON copied.",
             "query-copy-status");
}
window.copyQueryText = copyQueryText;

function copyText(text, okMsg, statusId) {
    const done = (msg) => {
        if (!statusId) { setStatus(msg, "ok"); return; }
        const el = document.getElementById(statusId);
        if (el) el.textContent = msg;
    };
    if (navigator.clipboard && window.isSecureContext) {
        navigator.clipboard.writeText(text).then(() => done(okMsg),
                                                 () => done("Copy failed."));
        return;
    }
    // The dashboard is normally reached over plain http on the lab network,
    // where navigator.clipboard is not available.
    const ta = document.createElement("textarea");
    ta.value = text;
    ta.style.position = "fixed";
    ta.style.opacity = "0";
    document.body.appendChild(ta);
    ta.select();
    try { done(document.execCommand("copy") ? okMsg : "Copy failed."); }
    catch (e) { done("Copy failed."); }
    document.body.removeChild(ta);
}

// ==============================================================================
// Discover — raw documents, field sidebar, volume histogram
// ==============================================================================
//
// The table used to render the mapping's first 25 leaf paths, which on
// ros-joint-states meant `digital_input_11` and firmware version numbers while
// every joint sat off-screen. Columns are now chosen: the backend supplies a
// sane default set, the sidebar shows what each field actually contains, and a
// row expands to the untouched _source — the nesting is the point of a ROS
// document, and flattening threw it away.

let discoverHits = [];        // the raw hits behind the current page

function discoverIndex() {
    return document.getElementById("discover-index").value || KPI_INDEX;
}

function discoverSize() {
    return parseInt(document.getElementById("discover-size").value, 10) || 50;
}

// Read a dotted path out of a nested _source, mirroring the backend's _dig.
function dig(src, path) {
    let cur = src;
    for (const part of String(path).split(".")) {
        if (cur && typeof cur === "object" && part in cur) cur = cur[part];
        else return undefined;
    }
    return cur;
}

function toggleDiscover() {
    const on = document.getElementById("toggle-discover").checked;
    document.getElementById("discover-panel").hidden = !on;
    if (on) refreshDiscover();
}
window.toggleDiscover = toggleDiscover;

function discoverPseudoPanel() {
    return { index: discoverIndex() };
}

function onDiscoverIndexChange() {
    state.discover.columns = [];
    state.discover.page = 0;
    state.discover.sort = null;
    state.discover.summaries = {};
    state.discover.expanded = {};
    refreshDiscover();
}
window.onDiscoverIndexChange = onDiscoverIndexChange;

function onDiscoverSizeChange() {
    state.discover.page = 0;
    refreshDiscover();
}
window.onDiscoverSizeChange = onDiscoverSizeChange;

function resetDiscoverColumns() {
    state.discover.columns = [];
    refreshDiscover();
}
window.resetDiscoverColumns = resetDiscoverColumns;

function discoverPage(delta) {
    const next = state.discover.page + delta;
    if (next < 0) return;
    state.discover.page = next;
    state.discover.expanded = {};
    refreshDiscover();
}
window.discoverPage = discoverPage;

function sortDiscoverBy(col) {
    const d = state.discover;
    if (d.sort === col) d.order = d.order === "desc" ? "asc" : "desc";
    else { d.sort = col; d.order = "desc"; }
    d.page = 0;
    refreshDiscover();
}
window.sortDiscoverBy = sortDiscoverBy;

function toggleDiscoverRow(i) {
    state.discover.expanded[i] = !state.discover.expanded[i];
    renderDiscoverRows();
}
window.toggleDiscoverRow = toggleDiscoverRow;

async function refreshDiscover() {
    const panel = document.getElementById("discover-panel");
    if (!panel || panel.hidden) return;
    const totalEl = document.getElementById("discover-total");
    const d = state.discover;
    const index = discoverIndex();
    const size = discoverSize();

    const extra = { size, format: "raw", offset: d.page * size };
    if (d.sort) { extra.sort = d.sort; extra.order = d.order; }

    try {
        const [docs] = await Promise.all([
            getJson("/api/es/docs?" + panelParams(discoverPseudoPanel(), extra).toString()),
            refreshDiscoverHistogram(),
        ]);

        discoverHits = docs.hits || [];
        d.total = docs.total || 0;
        d.defaultColumns = docs.columns || [];
        d.timeField = docs.time_field;

        document.getElementById("discover-timefield").textContent =
            docs.time_field ? `time: ${docs.time_field}` : "no time field";

        renderDiscoverHead();
        renderDiscoverRows();
        renderDiscoverPager();
        // After the columns are known, so the sidebar's add/remove state is not
        // a refresh behind what the table is showing.
        await renderFieldSidebar();
        totalEl.textContent = `${formatCount(d.total)} matching`;
    } catch (err) {
        totalEl.textContent = "error: " + err.message;
        throw err;
    }
}
window.refreshDiscover = refreshDiscover;

function discoverColumns() {
    const d = state.discover;
    return d.columns.length ? d.columns : (d.defaultColumns || []);
}

function renderDiscoverHead() {
    const d = state.discover;
    const cols = discoverColumns();
    const arrow = (c) => (d.sort === c ? (d.order === "desc" ? " ▾" : " ▴") : "");
    document.getElementById("discover-table").querySelector("thead").innerHTML =
        "<tr><th class='dt-expander'></th>" +
        cols.map((c) =>
            `<th class="dt-col" title="Sort by ${escapeHtml(c)}"
                 onclick="sortDiscoverBy(${jsAttr(c)})">${escapeHtml(c)}${arrow(c)}` +
            (d.columns.length
                ? ` <span class="dt-drop" title="Remove this column"
                        onclick="event.stopPropagation();removeDiscoverColumn(${jsAttr(c)})">✕</span>`
                : "") +
            "</th>").join("") + "</tr>";
}

function renderDiscoverRows() {
    const cols = discoverColumns();
    const tbody = document.getElementById("discover-table").querySelector("tbody");

    if (!discoverHits.length) {
        tbody.innerHTML =
            `<tr><td class="a-empty-cell" colspan="${cols.length + 1}">No documents in range.</td></tr>`;
        return;
    }

    tbody.innerHTML = discoverHits.map((h, i) => {
        const open = !!state.discover.expanded[i];
        const cells = cols.map((c) =>
            `<td>${escapeHtml(formatCell(dig(h._source, c)))}</td>`).join("");
        const row =
            `<tr class="${open ? "dt-open" : ""}">
                <td class="dt-expander" onclick="toggleDiscoverRow(${i})"
                    title="Show the raw document">${open ? "▾" : "▸"}</td>
                ${cells}
             </tr>`;
        if (!open) return row;
        return row +
            `<tr class="dt-raw-row">
                <td colspan="${cols.length + 1}">
                    <div class="dt-raw-head">
                        <code>_id: ${escapeHtml(h._id)}</code>
                        <button class="tb-btn ghost" onclick="copyDoc(${i})">📋 Copy JSON</button>
                    </div>
                    <pre class="dt-raw">${escapeHtml(JSON.stringify(h._source, null, 2))}</pre>
                </td>
             </tr>`;
    }).join("");
}

function copyDoc(i) {
    const h = discoverHits[i];
    if (!h) return;
    copyText(JSON.stringify(h._source, null, 2), "Document copied.");
}
window.copyDoc = copyDoc;

function renderDiscoverPager() {
    const d = state.discover;
    const size = discoverSize();
    const from = d.page * size;
    const shown = discoverHits.length;
    document.getElementById("dp-label").textContent = shown
        ? `${formatCount(from + 1)}–${formatCount(from + shown)} of ${formatCount(d.total)}`
        : "—";
    document.getElementById("dp-prev").disabled = d.page === 0;
    // Elasticsearch refuses a from+size window past 10 000 without a scroll or
    // search_after, and the backend caps `offset` at 9 000 to match.
    const atWindowEnd = from + size >= 9000;
    document.getElementById("dp-next").disabled =
        atWindowEnd || from + shown >= d.total;
    document.getElementById("dp-next").title = atWindowEnd
        ? "Elasticsearch's 10 000-document paging window ends here — narrow the range or add a filter"
        : "";
}

// ------------------------------------------------------------------------------
// Document-volume histogram above the table
// ------------------------------------------------------------------------------

async function refreshDiscoverHistogram() {
    const canvas = document.getElementById("discover-hist-canvas");
    if (!canvas) return;
    let d;
    try {
        d = await getJson("/api/es/timeseries?" +
            panelParams(discoverPseudoPanel(), { stat: "count", points: 60 }).toString());
    } catch (err) {
        d = { time: [], series: {} };
    }
    const times = d.time || [];
    const counts = (d.series || {})["doc_count::count"] || [];
    document.getElementById("dh-empty").hidden = times.length > 0;

    if (charts.__discoverHist) charts.__discoverHist.destroy();
    if (!times.length) { charts.__discoverHist = null; return; }

    charts.__discoverHist = new Chart(canvas.getContext("2d"), {
        type: "bar",
        data: {
            labels: times,
            datasets: [{
                data: counts,
                backgroundColor: "rgba(56, 189, 248, 0.55)",
                borderColor: "rgba(56, 189, 248, 0.9)",
                borderWidth: 1,
                barPercentage: 1.0,
                categoryPercentage: 1.0,
            }],
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            onClick: (evt, els) => {
                if (!els.length) return;
                zoomToBucket(times, els[0].index);
            },
            scales: {
                x: { type: "category",
                     ticks: { color: "#64748b", maxTicksLimit: 8,
                              callback: (v, i) => shortTime(times[i]) },
                     grid: { display: false } },
                y: { ticks: { color: "#64748b", maxTicksLimit: 4 },
                     grid: { color: "rgba(255,255,255,0.04)" } },
            },
            plugins: {
                legend: { display: false },
                tooltip: {
                    callbacks: {
                        title: (items) => new Date(times[items[0].dataIndex]).toLocaleString(),
                        label: (item) => `${formatCount(item.parsed.y)} documents`,
                    },
                },
            },
        },
    });
}

function shortTime(ms) {
    const d = new Date(ms);
    return d.toLocaleDateString(undefined, { month: "short", day: "numeric" });
}

// Clicking a bar narrows the whole page to that bucket — the drill-down that
// makes the histogram worth showing rather than decoration.
function zoomToBucket(times, i) {
    const start = times[i];
    const end = i + 1 < times.length ? times[i + 1] : start + (times[1] - times[0] || 1000);
    const toInput = (ms) => new Date(ms).toISOString().slice(0, 19);
    document.getElementById("range-preset").value = "custom";
    onPresetChange();
    document.getElementById("range-from").value = toInput(start);
    document.getElementById("range-to").value = toInput(end);
    state.discover.page = 0;
    refreshAnalytics();
}

// ------------------------------------------------------------------------------
// Field sidebar
// ------------------------------------------------------------------------------

async function renderFieldSidebar() {
    const list = document.getElementById("df-list");
    if (!list) return;
    const index = discoverIndex();
    const meta = await fetchFields(index);
    const needle = (document.getElementById("df-search").value || "").toLowerCase();
    const shown = discoverColumns();

    const fields = (meta.fields || [])
        .filter((f) => !f.path.endsWith(".keyword"))
        .filter((f) => !needle || f.path.toLowerCase().includes(needle));

    if (!fields.length) {
        list.innerHTML = `<div class="df-empty">${
            meta.fields && meta.fields.length ? "No field matches." : "No mapped fields."}</div>`;
        return;
    }

    list.innerHTML = fields.map((f) => {
        const active = shown.includes(f.path);
        const sum = state.discover.summaries[f.path];
        return `<div class="df-item ${active ? "active" : ""}">
            <div class="df-row" onclick="toggleFieldSummary(${jsAttr(f.path)})">
                <span class="df-type" title="${escapeHtml(f.type)}">${typeGlyph(f.type)}</span>
                <span class="df-name" title="${escapeHtml(f.path)}">${escapeHtml(f.path)}</span>
                <button class="df-add" title="${active ? "Remove column" : "Add as column"}"
                        onclick="event.stopPropagation();${active
                            ? `removeDiscoverColumn(${jsAttr(f.path)})`
                            : `addDiscoverColumn(${jsAttr(f.path)})`}">${active ? "−" : "＋"}</button>
            </div>
            ${sum ? renderFieldSummary(sum) : ""}
        </div>`;
    }).join("");
}
window.renderFieldSidebar = renderFieldSidebar;

function typeGlyph(t) {
    if (["long", "integer", "short", "byte", "double", "float",
         "half_float", "scaled_float"].includes(t)) return "#";
    if (t === "date") return "🕓";
    if (t === "boolean") return "◑";
    return "t";
}

async function toggleFieldSummary(field) {
    const cache = state.discover.summaries;
    if (cache[field]) { delete cache[field]; renderFieldSidebar(); return; }
    cache[field] = { loading: true, field };
    renderFieldSidebar();
    try {
        cache[field] = await getJson("/api/es/field_summary?" +
            panelParams(discoverPseudoPanel(), { field }).toString());
    } catch (err) {
        cache[field] = { field, error: err.message };
    }
    renderFieldSidebar();
}
window.toggleFieldSummary = toggleFieldSummary;

function renderFieldSummary(s) {
    if (s.loading) return `<div class="df-sum">loading…</div>`;
    if (s.error) return `<div class="df-sum df-err">${escapeHtml(s.error)}</div>`;

    const coverage = s.total
        ? `${((s.present / s.total) * 100).toFixed(0)}% of ${formatCount(s.total)} docs`
        : "no documents";
    let body = "";

    if (s.kind === "terms" && (s.top || []).length) {
        body = s.top.map((t) => `
            <div class="df-val">
                <div class="df-val-head">
                    <span class="df-val-name" title="${escapeHtml(t.value)}">${escapeHtml(t.value)}</span>
                    <span class="df-val-pct">${t.pct.toFixed(1)}%</span>
                    <button class="df-val-add" title="Filter to this value"
                            onclick="event.stopPropagation();filterToValue(${jsAttr(s.field)},${jsAttr(t.value)})">🔍</button>
                </div>
                <div class="df-bar"><span style="width:${Math.max(2, t.pct)}%"></span></div>
            </div>`).join("");
    } else if (s.kind === "number" && s.stats) {
        const p = s.percentiles || {};
        body = `<table class="df-stats">
            <tr><td>min</td><td>${formatNum(s.stats.min)}</td><td>p05</td><td>${formatNum(p["5.0"])}</td></tr>
            <tr><td>p25</td><td>${formatNum(p["25.0"])}</td><td>p50</td><td>${formatNum(p["50.0"])}</td></tr>
            <tr><td>p75</td><td>${formatNum(p["75.0"])}</td><td>p95</td><td>${formatNum(p["95.0"])}</td></tr>
            <tr><td>max</td><td>${formatNum(s.stats.max)}</td><td>avg</td><td>${formatNum(s.stats.avg)}</td></tr>
        </table>`;
    } else if (s.kind === "date" && s.stats) {
        body = `<div class="df-span">
            ${escapeHtml(s.stats.min_as_string || "—")}<br>→ ${escapeHtml(s.stats.max_as_string || "—")}
        </div>`;
    } else {
        body = `<div class="df-note">No values in the current selection.</div>`;
    }

    return `<div class="df-sum">
        <div class="df-cov">${coverage}${
            s.distinct != null ? ` · ${formatCount(s.distinct)} distinct` : ""}</div>
        ${body}
    </div>`;
}

function addDiscoverColumn(field) {
    const d = state.discover;
    if (!d.columns.length) d.columns = (d.defaultColumns || []).slice();
    if (!d.columns.includes(field)) d.columns.push(field);
    renderDiscoverHead();
    renderDiscoverRows();
    renderFieldSidebar();
}
window.addDiscoverColumn = addDiscoverColumn;

function removeDiscoverColumn(field) {
    const d = state.discover;
    if (!d.columns.length) d.columns = (d.defaultColumns || []).slice();
    d.columns = d.columns.filter((c) => c !== field);
    renderDiscoverHead();
    renderDiscoverRows();
    renderFieldSidebar();
}
window.removeDiscoverColumn = removeDiscoverColumn;

// A value in the sidebar becomes a real filter pill, so it applies to every
// panel rather than only to the table it was clicked in.
function filterToValue(field, value) {
    state.filters.push({ field, op: "is", value: numOrString(value) });
    renderFilterPills();
    refreshAnalytics();
}
window.filterToValue = filterToValue;

function exportDiscoverCsv() {
    const d = state.discover;
    const extra = { size: discoverSize(), format: "csv", offset: d.page * discoverSize() };
    if (d.columns.length) extra.fields = d.columns.join(",");
    if (d.sort) { extra.sort = d.sort; extra.order = d.order; }
    window.open("/api/es/docs?" +
        panelParams(discoverPseudoPanel(), extra).toString(), "_blank");
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
    document.getElementById("pb-mesh-field").style.display = t === "scatter3d" ? "" : "none";
    document.getElementById("pb-hint").textContent = PANEL_HINTS[t] || "";
}
window.onPanelTypeChange = onPanelTypeChange;

async function loadPanelFields() {
    const index = document.getElementById("pb-index").value;
    const [meta, tf] = await Promise.all([fetchFields(index), fetchTimeField(index)]);
    fillDatalist("pb-field-list", meta.fields.map((f) => f.path));

    // Candidates the resolver confirmed hold documents come first and carry
    // their coverage, so a field that exists but is empty is visibly a bad pick.
    const timeSel = document.getElementById("pb-time-field");
    const confirmed = (tf.candidates || []).filter((c) => c.docs > 0);
    const confirmedPaths = confirmed.map((c) => c.path);
    const others = [...(meta.date || []), ...(meta.numeric || [])
        .filter((f) => /sec$|time|stamp/i.test(f))]
        .filter((f) => !confirmedPaths.includes(f));

    const opts = confirmed.map((c) =>
        `<option value="${escapeHtml(c.path)}">${escapeHtml(c.path)} — ${formatCount(c.docs)} docs</option>`)
        .concat(others.map((f) =>
            `<option value="${escapeHtml(f)}">${escapeHtml(f)}</option>`));
    timeSel.innerHTML = opts.length ? opts.join("")
        : '<option value="@timestamp">@timestamp</option>';
    if (tf.time_field) timeSel.value = tf.time_field;

    const hint = document.getElementById("pb-hint");
    if (hint && !tf.usable) {
        hint.textContent = `⚠ ${index} has no time field holding data — a panel ` +
            "on it will come up empty.";
    }
}
window.loadPanelFields = loadPanelFields;

function applyPanelBuilder() {
    const status = document.getElementById("pb-status");
    const type = document.getElementById("pb-type").value;
    const index = document.getElementById("pb-index").value;
    const timeField = document.getElementById("pb-time-field").value || "";
    const title = document.getElementById("pb-title").value.trim();
    const split = document.getElementById("pb-split").value.trim();
    const unit = document.getElementById("pb-unit").value.trim();
    const width = document.getElementById("pb-width").value;
    const xyz = type === "scatter2d" || type === "scatter3d";

    const panel = {
        id: "user_" + Date.now().toString(36),
        type, index, title: title || `${type} — ${index}`,
        unit, width, side: index.includes("sim") ? "sim" : "live",
    };

    // Keeping the resolved field means storing nothing: the panel then follows
    // the index if the ingest starts stamping a better one later. Only a
    // deliberate override is pinned, and it gets its stored unit with it.
    const resolvedField = (state.timeFieldCache[index] || {}).time_field;
    if (timeField && timeField !== resolvedField) {
        panel.time_field = timeField;
        panel.time_unit = unitForField(index, timeField);
    }

    if (xyz) {
        panel.x = document.getElementById("pb-x").value.trim();
        panel.y = document.getElementById("pb-y").value.trim();
        panel.z = document.getElementById("pb-z").value.trim();
        if (!panel.x || !panel.y) { status.textContent = "X and Y are required."; return; }
        if (type === "scatter3d" && !panel.z) { status.textContent = "Z is required."; return; }
        // Only meaningful for a path already expressed in the UR10e base frame;
        // on any other index the outline would be placed against coordinates it
        // has nothing to do with.
        if (type === "scatter3d" && document.getElementById("pb-mesh").checked) {
            if (index !== "ros-tcp-pose-topic") {
                status.textContent =
                    "The chassis outline is positioned in the ur10e_base frame, " +
                    "which only ros-tcp-pose-topic uses.";
                return;
            }
            panel.mesh = "chassis";
        }
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


// ==============================================================================
// Panel interaction
// ==============================================================================

// Clicking a legend entry isolates that series instead of merely hiding it.
// Seven joints drawn on top of each other is the normal case here, and picking
// one out by hiding the other six was six clicks. Clicking the isolated series
// again brings everything back.
function isolateSeries(e, item, legend) {
    const chart = legend.chart;
    const label = item.text;
    const belongs = (l) => l === label || l === "__min_" + label || l === "__max_" + label;
    const visible = chart.data.datasets
        .map((ds, i) => (!ds.label.startsWith("__") && chart.isDatasetVisible(i) ? ds.label : null))
        .filter(Boolean);
    const isolated = visible.length === 1 && visible[0] === label;

    chart.data.datasets.forEach((ds, i) => {
        chart.setDatasetVisibility(i, isolated ? true : belongs(ds.label));
    });
    chart.update("none");
}

// A live panel and its sim twin are only comparable if they share a y scale.
// Side by side on different scales, a 0.2 rad wobble and a 2 rad sweep look
// identical, which is the opposite of what the pairing is for.
function applyPairedScales() {
    const groups = {};
    state.panels.forEach((p) => {
        if (p.pair) (groups[p.pair] = groups[p.pair] || []).push(p);
    });

    Object.values(groups).forEach((panels) => {
        // Sharing a y-axis only means something for the same quantity. Real
        // effort is current (A), sim effort is torque (Nm): on one axis the
        // larger unit flattens the other to a line at zero.
        if (new Set(panels.map((p) => p.unit || "")).size > 1) return;
        const paired = panels.map((p) => charts[p.id]).filter(Boolean);
        if (paired.length < 2) return;

        let min = Infinity, max = -Infinity;
        paired.forEach((ch) => (ch.data.datasets || []).forEach((ds) => {
            (ds.data || []).forEach((pt) => {
                const y = pt && typeof pt === "object" ? pt.y : pt;
                if (typeof y === "number" && Number.isFinite(y)) {
                    if (y < min) min = y;
                    if (y > max) max = y;
                }
            });
        }));
        if (!Number.isFinite(min) || !Number.isFinite(max)) return;

        const pad = (max - min) * 0.05 || Math.abs(max) * 0.05 || 1;
        paired.forEach((ch) => {
            ch.options.scales.y.min = min - pad;
            ch.options.scales.y.max = max + pad;
            ch.update("none");
        });
    });
}

// Panel → Discover, carrying the index across. The point of the two living on
// one page is being able to go from "that spike" to the documents behind it.
function drillToDiscover(panelId) {
    const p = state.panels.find((x) => x.id === panelId);
    if (!p) return;
    const sel = document.getElementById("discover-index");
    if (![...sel.options].some((o) => o.value === p.index)) {
        sel.insertAdjacentHTML("beforeend",
            `<option value="${escapeHtml(p.index)}">${escapeHtml(p.index)}</option>`);
    }
    sel.value = p.index;
    document.getElementById("toggle-discover").checked = true;
    document.getElementById("discover-panel").hidden = false;
    onDiscoverIndexChange();
    document.getElementById("discover-panel")
        .scrollIntoView({ behavior: "smooth", block: "start" });
}
window.drillToDiscover = drillToDiscover;

function escapeHtml(s) {
    return String(nz(s, "")).replace(/[&<>"']/g, (c) => ({
        "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
    }[c]));
}

// Safe to drop into an inline handler like onclick="fn(...)".
// escapeHtml alone is not: it turns ' into &#39;, which the HTML parser decodes
// back to a bare quote that closes the JS string literal. JSON.stringify does
// the JS-level escaping, escapeHtml then protects the attribute delimiter.
function jsAttr(v) {
    return escapeHtml(JSON.stringify(String(v)));
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

    // Enter runs the query bar; blurring it without Enter does not, so a
    // half-typed clause never fires eight requests on its own.
    const qbar = document.getElementById("query-bar");
    if (qbar) {
        qbar.addEventListener("keydown", (e) => {
            if (e.key === "Enter") { e.preventDefault(); applyQueryBar(); }
        });
    }
    const dslBox = document.getElementById("query-dsl");
    if (dslBox) {
        dslBox.addEventListener("keydown", (e) => {
            if (e.key === "Enter" && (e.ctrlKey || e.metaKey)) {
                e.preventDefault();
                applyDsl();
            }
        });
    }

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
        closeQueryPreview();
        closeSideMenu();
    });
});
