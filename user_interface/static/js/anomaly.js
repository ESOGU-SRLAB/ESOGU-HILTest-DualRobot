// =============================================================================
// Anomaly Detection tab
//
// Data path: app.py -> AnomalyCollector -> socket "anomaly_update" (5 Hz)
//            app.py -> /api/anomaly/events  (olaylar_*.jsonl)
//            app.py -> /api/anomaly/label   (operator label)
//
// NOTE: the JSON keys below (zaman, tepe, giris, etiket, "gercek"/"yanlis", ...)
// are Turkish because the detector node writes them that way on disk. They are a
// wire format, not UI text -- renaming them here would break the log files and
// the existing labels in etiketler.json. Only what the operator sees is English.
// =============================================================================

// The detector decides at 20 Hz while this tab is fed at 5 Hz. The clearest real
// event on record (the pick & place collision of 21 Aug 2026, peak 146.4) lasted
// only 0.25 s. Showing the banner for that long is useless to the human eye, so
// hold it for at least this window.
const AN_LATCH_MS = 5000;

// The y-axis frame is derived from the LIVE threshold, not fixed. It used to be
// a constant 30, chosen when normal operation sat near 1.3 and the threshold at
// 18. Recalibrating the detector on the cell moved the threshold to 1.4 and
// normal operation to ~0.13; on a 0-30 axis the whole trace collapses onto the
// bottom edge and the chart shows nothing. Framing on the threshold keeps the
// alarm line at a fixed height on screen whatever the score scale happens to be.
const AN_Y_FACTOR = 1.7;      // threshold sits at ~59% of the frame height
const AN_Y_FALLBACK = 30;     // only until the first threshold arrives

let anChart = null;
let anBuilt = false;
let anLatchUntil = 0;
let anLastEventFetch = 0;

// Last thresholds seen on the socket; the tooltip and the point styling need
// them outside of onAnomalyUpdate.
// Fallbacks only; replaced by the values the server sends with every update.
// These moved when the detector was recalibrated (fused 18.0 -> 1.4) and when
// the v3 models landed (residual 1.6008 -> 1.3723, raw 3.4461 -> 0.3934).
let anThr = { fused: 1.4, residual: 1.3723, raw: 0.3934 };

// Last event list fetched from /api/anomaly/events, used to enrich the tooltip
// of a hovered spike with the logged event behind it.
let anEvents = [];

// ----------------------------------------------------------------- chart ----

// Base frame: tall enough to show the alarm line with headroom above it.
function anAxisBase() {
    const t = (anThr && isFinite(anThr.fused) && anThr.fused > 0)
        ? anThr.fused : null;
    return t ? t * AN_Y_FACTOR : AN_Y_FALLBACK;
}

// Grow the axis past the base only when a sample needs it, snapping to round
// steps so a spike scrolling through the window does not make it twitch. The
// step is scaled to the threshold, otherwise a 10-unit step is invisible on a
// 1.4-unit axis and enormous on an 18-unit one.
function anAxisMax(peak) {
    const base = anAxisBase();
    if (!isFinite(peak) || peak <= base) return base;
    const target = peak * 1.1;
    const mag = Math.pow(10, Math.floor(Math.log10(target)));
    const step = target / mag > 5 ? mag : mag / 2;
    return Math.ceil(target / step) * step;
}

function buildAnomalyChart() {
    const el = document.getElementById("an-chart");
    if (!el || typeof Chart === "undefined") return;

    anChart = new Chart(el.getContext("2d"), {
        type: "line",
        data: {
            datasets: [
                {
                    label: "fused score",
                    data: [],
                    borderColor: "#6366f1",
                    backgroundColor: "rgba(99, 102, 241, 0.12)",
                    borderWidth: 2,
                    fill: true,
                    tension: 0.15,
                    // Only samples over the threshold get a marker: they are the
                    // ones worth hovering, and they stay findable at a glance.
                    pointRadius: (ctx) => (anPointOver(ctx) ? 4 : 0),
                    pointHoverRadius: (ctx) => (anPointOver(ctx) ? 6 : 3),
                    pointBackgroundColor: "#ef4444",
                    pointBorderColor: "#fca5a5",
                    pointBorderWidth: 1,
                },
                {
                    label: "threshold",
                    data: [],
                    borderColor: "#ef4444",
                    borderWidth: 2,
                    borderDash: [6, 4],
                    pointRadius: 0,
                    fill: false,
                },
            ],
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            interaction: { mode: "nearest", intersect: false, axis: "x" },
            scales: {
                x: {
                    type: "linear",
                    title: { display: true, text: "seconds (now = 0)", color: "#64748b" },
                    ticks: { color: "#64748b", maxTicksLimit: 7 },
                    grid: { color: "rgba(255,255,255,0.04)" },
                },
                y: {
                    type: "linear",
                    min: 0,
                    max: anAxisBase(),
                    title: { display: true, text: "fused score", color: "#64748b" },
                    ticks: { color: "#64748b", maxTicksLimit: 7 },
                    grid: { color: "rgba(255,255,255,0.04)" },
                },
            },
            plugins: {
                legend: { labels: { color: "#94a3b8", boxWidth: 12 } },
                tooltip: {
                    // The flat threshold line has nothing to say on hover.
                    filter: (item) => item.datasetIndex === 0,
                    backgroundColor: "rgba(15, 23, 42, 0.95)",
                    borderColor: "rgba(148, 163, 184, 0.25)",
                    borderWidth: 1,
                    padding: 10,
                    titleFont: { size: 12 },
                    bodyFont: { size: 11 },
                    bodySpacing: 3,
                    caretPadding: 8,
                    callbacks: {
                        title: (items) => anTipTitle(items),
                        label: (c) => `fused score: ${Number(c.parsed.y).toFixed(3)}`,
                        afterBody: (items) => anTipDetails(items),
                    },
                },
            },
        },
    });
}

function anPointOver(ctx) {
    const y = ctx.parsed ? ctx.parsed.y : ctx.raw && ctx.raw.y;
    return typeof y === "number" && y > anThr.fused;
}

// ---------------------------------------------------------------- tooltip ---
function anTipTitle(items) {
    const rec = items.length && items[0].raw && items[0].raw.r;
    if (!rec) return "";
    const over = rec.fused > anThr.fused;
    return (over ? "⚠ ANOMALY — " : "") + fmtClock(rec.t);
}

// Everything the operator needs to judge a spike without leaving the chart:
// the sub-scores, which rules fired, and -- when the sample falls inside a
// logged event -- that event's trigger, peak, duration and label.
function anTipDetails(items) {
    const rec = items.length && items[0].raw && items[0].raw.r;
    if (!rec) return [];

    const lines = [
        `threshold:  ${fmtNum(anThr.fused)}`,
        `residual:   ${fmtNum(rec.s_kal, anThr.residual)}  (θ ${fmtNum(anThr.residual, anThr.residual)})`,
        `raw:        ${fmtNum(rec.s_ham, anThr.raw)}  (θ ${fmtNum(anThr.raw, anThr.raw)})`,
    ];

    const rules = [];
    if (rec.hit_abs) rules.push("absolute");
    if (rec.hit_ad) rules.push("adaptive");
    if (rec.hit_kal) rules.push("residual");
    if (rec.hit_ham) rules.push("raw");
    if (rules.length) lines.push(`rules fired: ${rules.join(", ")}`);

    if (typeof rec.moving === "boolean") {
        lines.push(`motion:     ${rec.moving ? "moving" : "idle"}`
            + (rec.qd_peak != null ? `  (q̇ peak ${fmtNum(rec.qd_peak)})` : ""));
    }

    const ev = anEventAt(rec.t);
    if (ev) {
        lines.push("");
        lines.push(`── logged event #${ev.sira} ──`);
        lines.push(`source:     ${triggerText(ev.tetikleyen, "unknown")}`);
        lines.push(`started:    ${fmtTime(ev.zaman)}`);
        lines.push(`peak:       ${fmtNum(ev.tepe)}   entry: ${fmtNum(ev.giris)}`);
        lines.push(`duration:   ${ev.sure_s == null
            ? (ev.devam ? "ongoing" : "unknown")
            : Number(ev.sure_s).toFixed(2) + " s"}`);
        lines.push(`label:      ${labelText(ev.etiket)}`);
        if (ev.kural) lines.push(`rule:       ${ev.kural}`);
        if (ev.kosu) lines.push(`run:        ${ev.kosu}`);
    }
    return lines;
}

// A live sample belongs to a logged event when it falls inside that event's
// window. The margin absorbs the clock skew between the detector's timestamp
// and the moment the dashboard received the sample.
function anEventAt(tEpoch) {
    if (!tEpoch) return null;
    for (const e of anEvents) {
        if (e._t0 == null) continue;
        const dur = e.sure_s == null ? 2.0 : e.sure_s;
        if (tEpoch >= e._t0 - 0.8 && tEpoch <= e._t0 + dur + 0.8) return e;
    }
    return null;
}

// ------------------------------------------------------------- live feed ----
function onAnomalyUpdate(msg) {
    const conn = document.getElementById("an-conn");
    if (!conn) return;   // tab is not in the DOM yet

    if (msg.connected) {
        conn.textContent = "● DETECTOR LIVE";
        conn.className = "an-conn live";
    } else {
        conn.textContent = "● DETECTOR NOT PUBLISHING";
        conn.className = "an-conn dead";
    }

    anThr = {
        // Sunucu eşikleri her güncellemede gönderiyor; buradaki yedekler yalnız
        // ilk mesaj gelmeden önce geçerli ve modülün başındaki değerlerle AYNI
        // olmalı — ayrı yerlerde iki farklı yedek tutmak, biri güncellenip
        // diğeri unutulduğunda alarm çizgisini sessizce yanlış yere koyar.
        fused: (msg.thresholds && msg.thresholds.fused) || anThr.fused,
        residual: (msg.thresholds && msg.thresholds.residual) || anThr.residual,
        raw: (msg.thresholds && msg.thresholds.raw) || anThr.raw,
    };
    const thr = anThr.fused;
    setText("an-thr", fmtNum(thr));
    setText("an-hz", msg.decision_hz ? msg.decision_hz.toFixed(1) + " Hz" : "—");

    // No numeric fused-score readout here on purpose. The socket pushes at 5 Hz
    // while the detector decides at 20 Hz, so a single sampled value trails the
    // real signal and reads as plainly wrong next to the chart. The chart plots
    // the whole 20 Hz series, and its tooltip gives the exact per-sample value.
    const cur = msg.current;
    if (cur && msg.connected) {
        updateBar("res", cur.s_kal, anThr.residual);
        updateBar("raw", cur.s_ham, anThr.raw);
        // Model eşikleri şablonda SABİT metin olarak duruyordu (1.60 / 3.45 —
        // v2 değerleri). v3'te 1.3723 / 0.3934 oldu ve ekran görüntüleri yanlış
        // eşikle çıktı. Artık sunucunun gönderdiği değerden yazılıyor.
        setText("an-thr-res", fmtNum(anThr.residual, anThr.residual));
        setText("an-thr-raw", fmtNum(anThr.raw, anThr.raw));
    } else {
        clearBar("res");
        clearBar("raw");
    }

    setText("an-since", msg.last_alarm_ago == null ? "—" : fmtAgo(msg.last_alarm_ago));

    // Latch on the edge, not the level: sampling the level at 5 Hz drops short
    // events entirely, so extend the window whenever an edge arrives.
    const now = Date.now();
    if (msg.alarm_edge || msg.alarm) anLatchUntil = now + AN_LATCH_MS;

    const banner = document.getElementById("an-banner");
    const state = document.getElementById("an-banner-state");
    if (!msg.connected) {
        banner.className = "an-banner an-idle";
        state.textContent = "WAITING";
    } else if (msg.alarm || now < anLatchUntil) {
        banner.className = "an-banner an-alarm";
        state.textContent = msg.alarm ? "ANOMALY" : "ANOMALY (ended)";
        // The event just closed: refresh the table so the operator can label it
        // right away, and so the chart tooltip can name its trigger.
        if (!msg.alarm && now - anLastEventFetch > 3000) refreshAnomalyEvents();
    } else {
        banner.className = "an-banner an-ok";
        state.textContent = "NORMAL";
    }

    if (anChart && msg.series) {
        const t0 = msg.now;
        // The whole sample record rides along as `r` so the tooltip can show the
        // sub-scores and the rules that fired for the exact hovered point.
        const pts = msg.series.map((r) => ({ x: r.t - t0, y: r.fused, r: r }));
        let peak = thr;
        for (const p of pts) if (p.y > peak) peak = p.y;

        anChart.data.datasets[0].data = pts;
        anChart.data.datasets[1].data = pts.length
            ? [{ x: pts[0].x, y: thr }, { x: pts[pts.length - 1].x, y: thr }]
            : [];
        anChart.options.scales.y.max = anAxisMax(peak);
        anChart.update("none");
    }
}

function clearBar(which) {
    const fill = document.getElementById("an-bar-" + which);
    const out = document.getElementById("an-val-" + which);
    if (!fill || !out) return;
    fill.style.width = "0%";
    fill.classList.remove("over");
    out.textContent = "—";
}

function updateBar(which, val, thr) {
    const fill = document.getElementById("an-bar-" + which);
    const out = document.getElementById("an-val-" + which);
    if (!fill || !out) return;
    const ratio = val / thr;
    fill.style.width = Math.min(ratio * 100, 100).toFixed(1) + "%";
    fill.classList.toggle("over", ratio > 1);
    out.textContent = fmtNum(val);
}

// ----------------------------------------------------------- event table ----
async function refreshAnomalyEvents() {
    anLastEventFetch = Date.now();
    const body = document.getElementById("an-event-body");
    if (!body) return;
    try {
        const r = await fetch("/api/anomaly/events");
        const j = await r.json();
        if (!j.ok) throw new Error(j.error || "unknown error");
        renderEvents(j.events || []);
    } catch (err) {
        body.innerHTML = `<tr><td colspan="7" class="an-empty">
            Could not read events: ${escapeHtml(String(err.message || err))}</td></tr>`;
    }
}

function renderEvents(events) {
    const body = document.getElementById("an-event-body");
    const stats = document.getElementById("an-event-stats");

    // Cache for the chart tooltip, with the start time pre-parsed to epoch
    // seconds so hover lookups stay cheap.
    anEvents = events.map((e) => Object.assign({}, e, { _t0: parseEventTime(e.zaman) }));

    if (!events.length) {
        body.innerHTML = `<tr><td colspan="7" class="an-empty">No events recorded.</td></tr>`;
        if (stats) stats.textContent = "";
        return;
    }

    const n = { gercek: 0, yanlis: 0, "?": 0 };
    events.forEach((e) => { n[e.etiket] = (n[e.etiket] || 0) + 1; });
    if (stats) {
        stats.textContent = `${events.length} events — ${n.gercek} true, `
            + `${n.yanlis} false alarms, ${n["?"]} unlabelled`;
    }

    body.innerHTML = events.map((e) => {
        const peak = fmtNum(e.tepe);
        const entry = fmtNum(e.giris);
        const duration = e.sure_s == null
            ? (e.devam ? "ongoing" : "—")
            : Number(e.sure_s).toFixed(2) + " s";
        const rowCls = e.etiket === "gercek" ? "an-row-true"
                     : e.etiket === "yanlis" ? "an-row-false" : "";
        // Highlight anything at or above the weakest confirmed real event (31.98).
        const big = e.tepe != null && e.tepe >= 31.98 ? " big" : "";
        return `<tr class="${rowCls}" data-id="${escapeHtml(e.id)}">
            <td>${escapeHtml(fmtTime(e.zaman))}</td>
            <td>${duration}</td>
            <td class="an-peak${big}">${peak}</td>
            <td class="an-entry">${entry}</td>
            <td>${escapeHtml(triggerText(e.tetikleyen))}</td>
            <td class="an-entry">${escapeHtml(e.kosu || "—")}</td>
            <td>${labelButtons(e)}</td>
        </tr>`;
    }).join("");
}

function labelButtons(e) {
    const mk = (val, txt, cls) =>
        `<button class="an-lbl ${e.etiket === val ? "on-" + cls : ""}"
                 onclick="labelAnomaly('${escapeHtml(e.id)}', '${val}', this)">${txt}</button>`;
    return `<span class="an-label-group">
        ${mk("gercek", "True", "true")}
        ${mk("yanlis", "False", "false")}
        ${mk("?", "?", "unknown")}
    </span>`;
}

async function labelAnomaly(id, etiket, btn) {
    const group = btn.parentElement.querySelectorAll(".an-lbl");
    group.forEach((b) => { b.disabled = true; });
    try {
        const r = await fetch("/api/anomaly/label", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ id: id, etiket: etiket }),
        });
        const j = await r.json();
        if (!j.ok) throw new Error(j.error || "could not be saved");
        await refreshAnomalyEvents();
    } catch (err) {
        group.forEach((b) => { b.disabled = false; });
        alert("Could not save label: " + (err.message || err));
    }
}

// ---------------------------------------------------------------- helpers ---
function setText(id, txt) {
    const el = document.getElementById(id);
    if (el) el.textContent = txt;
}

function labelText(etiket) {
    return etiket === "gercek" ? "true anomaly"
         : etiket === "yanlis" ? "false alarm" : "unlabelled";
}

// The detector writes "kalıntı" (residual) / "ham" (raw) / "kalıntı+ham" /
// "yalnız birleşim" (fused-only -- neither model crossed its own threshold,
// only the fused score did) straight to disk in Turkish; only the display
// gets translated, same policy as labelText() above.
const AN_TRIGGER_LABELS = { "kalıntı": "residual", "ham": "raw" };

function triggerText(tetikleyen, fallback) {
    if (!tetikleyen) return fallback !== undefined ? fallback : "—";
    if (tetikleyen === "yalnız birleşim") return "fused only";
    return tetikleyen.split("+").map((t) => AN_TRIGGER_LABELS[t] || t).join("+");
}

// Decimal places scaled to the score magnitude. Two places was fine when the
// fused threshold was 18 and normal operation sat near 1.3, but the detector
// was recalibrated on the cell: the threshold is now 1.4 and normal operation
// ~0.13, where "0.05" and "0.13" throw away most of the resolution the operator
// needs to see a trend.
function anPrec(ref) {
    const r = (isFinite(ref) && ref > 0) ? ref : 1;
    return r >= 10 ? 2 : r >= 1 ? 3 : 4;
}

function fmtNum(v, ref) {
    return v == null ? "—"
        : Number(v).toFixed(anPrec(ref === undefined ? anThr.fused : ref));
}

// "2026-08-21T12:12:43" -> "21/08/2026 12:12:43". Parsed with a regex rather
// than Date: the detector writes local wall-clock time with no zone, and Date
// would silently shift it.
function fmtTime(raw) {
    if (!raw) return "—";
    const m = String(raw).match(/^(\d{4})-(\d{2})-(\d{2})[T ](\d{2}:\d{2}:\d{2})/);
    return m ? `${m[3]}/${m[2]}/${m[1]} ${m[4]}` : String(raw);
}

// Same string -> epoch seconds, built from local components so it matches the
// dashboard's own clock (which is what the live samples are stamped with).
function parseEventTime(raw) {
    const m = String(raw || "").match(/^(\d{4})-(\d{2})-(\d{2})[T ](\d{2}):(\d{2}):(\d{2})/);
    if (!m) return null;
    return new Date(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6]).getTime() / 1000;
}

// Epoch seconds -> "21/08/2026 12:12:43" for the chart tooltip.
function fmtClock(sec) {
    if (!sec) return "—";
    const d = new Date(sec * 1000);
    const p = (n) => String(n).padStart(2, "0");
    return `${p(d.getDate())}/${p(d.getMonth() + 1)}/${d.getFullYear()} `
         + `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

function fmtAgo(sec) {
    if (sec < 60) return sec.toFixed(0) + " s";
    if (sec < 3600) return (sec / 60).toFixed(1) + " min";
    return (sec / 3600).toFixed(1) + " h";
}

function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, (c) => (
        { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]
    ));
}

// ------------------------------------------------------------------ setup ---
// The chart is built once, but the event list is refetched every time the tab is
// opened -- that reload is what the removed refresh button used to provide.
function initAnomalyTab() {
    if (!anBuilt) {
        buildAnomalyChart();
        anBuilt = true;
    }
    refreshAnomalyEvents();
}

// The socket listener is attached INDEPENDENTLY of the tab: alarms that fire
// while the operator is on another tab must still reach the latch and the
// event table.
(function attachAnomalySocket() {
    const tryAttach = () => {
        if (typeof socket === "undefined" || !socket) return false;
        socket.on("anomaly_update", onAnomalyUpdate);
        return true;
    };
    if (!tryAttach()) {
        let tries = 0;
        const iv = setInterval(() => {
            if (tryAttach() || ++tries > 50) clearInterval(iv);
        }, 200);
    }
})();

window.refreshAnomalyEvents = refreshAnomalyEvents;
window.labelAnomaly = labelAnomaly;
window.initAnomalyTab = initAnomalyTab;
