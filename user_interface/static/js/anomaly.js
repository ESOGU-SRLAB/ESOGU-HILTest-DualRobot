// =============================================================================
// Anomaly Detection sekmesi
//
// Veri yolu: app.py -> AnomalyCollector -> socket "anomaly_update" (5 Hz)
//            app.py -> /api/anomaly/events  (olaylar_*.jsonl)
//            app.py -> /api/anomaly/label   (operatör etiketi)
// =============================================================================

// Dedektör 20 Hz karar veriyor, bu sekme 5 Hz besleniyor. Ölçülen en net gerçek
// olay (21 Ağu 2026 pick&place çarpışması, tepe 146,4) yalnız 0,25 s sürdü.
// Bandı bu süre kadar göstermek insan gözü için işe yaramaz; en az bu kadar tut.
const AN_LATCH_MS = 5000;

let anChart = null;
let anBuilt = false;
let anLatchUntil = 0;
let anLastEventFetch = 0;

// ---------------------------------------------------------------- grafik ----
function buildAnomalyChart() {
    const el = document.getElementById("an-chart");
    if (!el || typeof Chart === "undefined") return;

    anChart = new Chart(el.getContext("2d"), {
        type: "line",
        data: {
            datasets: [
                {
                    label: "birleşik skor",
                    data: [],
                    borderColor: "#6366f1",
                    backgroundColor: "rgba(99, 102, 241, 0.12)",
                    borderWidth: 2,
                    pointRadius: 0,
                    fill: true,
                    tension: 0.15,
                },
                {
                    label: "eşik",
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
            interaction: { mode: "nearest", intersect: false },
            scales: {
                x: {
                    type: "linear",
                    title: { display: true, text: "saniye (şimdi = 0)", color: "#64748b" },
                    ticks: { color: "#64748b", maxTicksLimit: 7 },
                    grid: { color: "rgba(255,255,255,0.04)" },
                },
                y: {
                    // Gerçek olaylar 8–196 arasında, normal çalışma medyanı ~1,3.
                    // Doğrusal eksende normal çalışma düz çizgi olur.
                    type: "logarithmic",
                    min: 0.01,
                    title: { display: true, text: "birleşik skor (log)", color: "#64748b" },
                    ticks: { color: "#64748b" },
                    grid: { color: "rgba(255,255,255,0.04)" },
                },
            },
            plugins: {
                legend: { labels: { color: "#94a3b8", boxWidth: 12 } },
                tooltip: {
                    callbacks: {
                        label: (c) => `${c.dataset.label}: ${Number(c.parsed.y).toFixed(3)}`,
                    },
                },
            },
        },
    });
}

// ------------------------------------------------------------ canlı akış ----
function onAnomalyUpdate(msg) {
    const conn = document.getElementById("an-conn");
    if (!conn) return;   // sekme henüz DOM'da değil

    if (msg.connected) {
        conn.textContent = "● dedektör canlı";
        conn.className = "an-conn live";
    } else {
        conn.textContent = "● dedektör yayın yapmıyor";
        conn.className = "an-conn dead";
    }

    const thr = (msg.thresholds && msg.thresholds.fused) || 18.0;
    setText("an-thr", thr.toFixed(2));
    setText("an-thr-head", String(thr));
    setText("an-hz", msg.decision_hz ? msg.decision_hz.toFixed(1) + " Hz" : "—");

    const cur = msg.current;
    const scoreEl = document.getElementById("an-score");
    if (cur && msg.connected) {
        scoreEl.textContent = cur.fused.toFixed(3);
        scoreEl.classList.toggle("over", cur.fused > thr);
        updateBar("kal", cur.s_kal, (msg.thresholds || {}).residual || 1.6008);
        updateBar("ham", cur.s_ham, (msg.thresholds || {}).raw || 3.4461);
    } else {
        scoreEl.textContent = "—";
        scoreEl.classList.remove("over");
    }

    setText("an-since", msg.last_alarm_ago == null ? "—" : fmtAgo(msg.last_alarm_ago));

    // Mandal: kenar geldiyse pencereyi uzat. Seviye örneklemek kısa olayı düşürür.
    const now = Date.now();
    if (msg.alarm_edge || msg.alarm) anLatchUntil = now + AN_LATCH_MS;

    const banner = document.getElementById("an-banner");
    const state = document.getElementById("an-banner-state");
    if (!msg.connected) {
        banner.className = "an-banner an-idle";
        state.textContent = "BEKLENİYOR";
    } else if (msg.alarm || now < anLatchUntil) {
        banner.className = "an-banner an-alarm";
        state.textContent = msg.alarm ? "ANOMALİ" : "ANOMALİ (sona erdi)";
        // Olay kapandıysa tabloyu yenile ki operatör hemen etiketleyebilsin.
        if (!msg.alarm && now - anLastEventFetch > 3000) refreshAnomalyEvents();
    } else {
        banner.className = "an-banner an-ok";
        state.textContent = "NORMAL";
    }

    if (anChart && msg.series) {
        const t0 = msg.now;
        const pts = msg.series.map((r) => ({ x: r.t - t0, y: Math.max(r.fused, 0.01) }));
        anChart.data.datasets[0].data = pts;
        anChart.data.datasets[1].data = pts.length
            ? [{ x: pts[0].x, y: thr }, { x: pts[pts.length - 1].x, y: thr }]
            : [];
        anChart.update("none");
    }
}

function updateBar(which, val, thr) {
    const fill = document.getElementById("an-bar-" + which);
    const out = document.getElementById("an-val-" + which);
    if (!fill || !out) return;
    const oran = val / thr;
    fill.style.width = Math.min(oran * 100, 100).toFixed(1) + "%";
    fill.classList.toggle("over", oran > 1);
    out.textContent = val.toFixed(2);
}

// ------------------------------------------------------------- olay tablosu -
async function refreshAnomalyEvents() {
    anLastEventFetch = Date.now();
    const body = document.getElementById("an-event-body");
    if (!body) return;
    try {
        const r = await fetch("/api/anomaly/events");
        const j = await r.json();
        if (!j.ok) throw new Error(j.error || "bilinmeyen hata");
        renderEvents(j.events || []);
    } catch (err) {
        body.innerHTML = `<tr><td colspan="7" class="an-empty">
            Olaylar okunamadı: ${escapeHtml(String(err.message || err))}</td></tr>`;
    }
}

function renderEvents(events) {
    const body = document.getElementById("an-event-body");
    const stats = document.getElementById("an-event-stats");

    if (!events.length) {
        body.innerHTML = `<tr><td colspan="7" class="an-empty">Kayıtlı olay yok.</td></tr>`;
        if (stats) stats.textContent = "";
        return;
    }

    const n = { gercek: 0, yanlis: 0, "?": 0 };
    events.forEach((e) => { n[e.etiket] = (n[e.etiket] || 0) + 1; });
    if (stats) {
        stats.textContent = `${events.length} olay — ${n.gercek} gerçek, `
            + `${n.yanlis} yanlış alarm, ${n["?"]} etiketsiz`;
    }

    body.innerHTML = events.map((e) => {
        const tepe = e.tepe == null ? "—" : Number(e.tepe).toFixed(2);
        const giris = e.giris == null ? "—" : Number(e.giris).toFixed(2);
        const sure = e.sure_s == null
            ? (e.devam ? "sürüyor" : "—")
            : Number(e.sure_s).toFixed(2) + " s";
        const rowCls = e.etiket === "gercek" ? "an-row-gercek"
                     : e.etiket === "yanlis" ? "an-row-yanlis" : "";
        // Tepe, en zayıf doğrulanmış gerçek olayın (31,98) üstündeyse vurgula.
        const big = e.tepe != null && e.tepe >= 31.98 ? " big" : "";
        return `<tr class="${rowCls}" data-id="${escapeHtml(e.id)}">
            <td>${escapeHtml(e.zaman || "—")}</td>
            <td>${sure}</td>
            <td class="an-peak${big}">${tepe}</td>
            <td class="an-entry">${giris}</td>
            <td>${escapeHtml(e.tetikleyen || "—")}</td>
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
        ${mk("gercek", "Gerçek", "gercek")}
        ${mk("yanlis", "Yanlış", "yanlis")}
        ${mk("?", "?", "bilinmiyor")}
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
        if (!j.ok) throw new Error(j.error || "kaydedilemedi");
        await refreshAnomalyEvents();
    } catch (err) {
        group.forEach((b) => { b.disabled = false; });
        alert("Etiket kaydedilemedi: " + (err.message || err));
    }
}

// ------------------------------------------------------------------ yardım --
function setText(id, txt) {
    const el = document.getElementById(id);
    if (el) el.textContent = txt;
}

function fmtAgo(sec) {
    if (sec < 60) return sec.toFixed(0) + " sn";
    if (sec < 3600) return (sec / 60).toFixed(1) + " dk";
    return (sec / 3600).toFixed(1) + " sa";
}

function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, (c) => (
        { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]
    ));
}

// ------------------------------------------------------------------ kurulum -
function initAnomalyTab() {
    if (anBuilt) return;
    buildAnomalyChart();
    refreshAnomalyEvents();
    anBuilt = true;
}

// Soket dinleyicisi sekmeden BAĞIMSIZ kurulur: kullanıcı başka sekmedeyken olan
// alarmlar da mandala ve olay tablosuna yansımalı.
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
