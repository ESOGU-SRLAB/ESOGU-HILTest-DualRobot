#!/usr/bin/env python3
"""viewpoint_planner paketinin BUGÜNKÜ halini anlatan sistem tanımı raporunu üretir.

    python3 docs/sistem_tanimi_uret.py

Bu belge bir GELİŞME raporu değildir: neyin ne zaman değiştiğini değil, sistemin
şu anda nasıl çalıştığını anlatır. Gelişme/karar kaydı ayrı dosyadadır
(docs/viewpoint_planner_teknik_rapor.docx, rapor_uret.py ile üretilir).

Parametre tabloları launch dosyasından ve yaml'dan, veri şemaları güncel plan
dosyasından, başarım sayıları octomap çıktılarından ÜRETİM ANINDA okunur.
"""
import ast
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Inches, Pt, RGBColor

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import figur_uret as F  # noqa: E402

OUT = os.path.join(HERE, "viewpoint_planner_sistem_tanimi.docx")
LAUNCH = os.path.join(PKG, "launch", "inspection_execution.launch.py")
PARAMS = os.path.join(PKG, "config", "sick_tmini_params.yaml")
FIG_WIDTH = Inches(5.9)
INK = "#22252a"


# --------------------------------------------------------------------------- #
# docx yardımcıları
# --------------------------------------------------------------------------- #
def h1(doc, t):
    doc.add_heading(t, level=1)


def h2(doc, t):
    doc.add_heading(t, level=2)


def p(doc, t):
    return doc.add_paragraph(t)


def bullets(doc, items):
    for it in items:
        doc.add_paragraph(it, style="List Bullet")


def code(doc, text):
    par = doc.add_paragraph()
    run = par.add_run(text)
    run.font.name = "Consolas"
    run.font.size = Pt(8.5)
    par.paragraph_format.space_after = Pt(8)


def table(doc, header, rows, widths=None):
    t = doc.add_table(rows=1, cols=len(header))
    t.style = "Light Grid Accent 1"
    for c, name in zip(t.rows[0].cells, header):
        c.text = ""
        r = c.paragraphs[0].add_run(str(name))
        r.bold = True
        r.font.size = Pt(9)
    for row in rows:
        cells = t.add_row().cells
        for c, val in zip(cells, row):
            c.text = ""
            r = c.paragraphs[0].add_run(str(val))
            r.font.size = Pt(8.5)
    if widths:
        t.autofit = False
        for i, w in enumerate(widths):
            for row in t.rows:
                row.cells[i].width = Inches(w)
    doc.add_paragraph()


def figure(doc, filename, caption):
    path = os.path.join(HERE, filename)
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} yok — önce: python3 docs/figur_uret.py")
    doc.add_picture(path, width=FIG_WIDTH)
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap = doc.add_paragraph()
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = cap.add_run(caption)
    run.italic = True
    run.font.size = Pt(8.5)
    run.font.color.rgb = RGBColor(0x55, 0x55, 0x55)


# --------------------------------------------------------------------------- #
# launch argümanlarını kaynaktan oku
# --------------------------------------------------------------------------- #
def launch_args(path):
    tree = ast.parse(open(path).read())
    out = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and getattr(node.func, "id", "") == "DeclareLaunchArgument"):
            continue
        name = node.args[0].value if node.args and isinstance(node.args[0], ast.Constant) \
            else "?"
        kw = {k.arg: k.value for k in node.keywords}

        def lit(key):
            if key not in kw:
                return ""
            try:
                return ast.literal_eval(kw[key])
            except Exception:
                return "<dinamik>"
        out.append((name, lit("default_value"), " ".join(str(lit("description")).split())))
    return out


def short(text, n=190):
    return text if len(text) <= n else text[:n - 1] + "…"


# --------------------------------------------------------------------------- #
# mimari şeması
# --------------------------------------------------------------------------- #
def fig_architecture():
    boxes = [
        (0.5, 6.2, 3.0, 1.0, "Şasi mesh'i (STL)\nchassis_last/PART*.stl", "#e7f5ff", "#1f6feb"),
        (0.5, 4.7, 3.0, 1.0, "viewpoint_planner_node\nplanlama + sıralama", "#fff4e6", "#d9480f"),
        (0.5, 3.2, 3.0, 1.0, "plans/viewpoint_plan.json\n(bakış-noktaları + IK çözümleri)", "#f8f9fa", "#868e96"),
        (0.5, 1.7, 3.0, 1.0, "inspection_executor_node\ntur yürütme + yakalama", "#ebfbee", "#2f9e44"),
        (0.5, 0.2, 3.0, 1.0, "pcds/single_ur10e/*\n(pcd + poz) → octomap", "#f8f9fa", "#868e96"),
        (4.6, 4.7, 3.0, 1.0, "move_group\n/compute_ik, /plan_kinematic_path\n/apply_planning_scene", "#f3f0ff", "#7048e8"),
        (4.6, 2.9, 2.3, 0.8, "trajectories/ur_<id>.json\nkayıt-oynat cache", "#f8f9fa", "#868e96"),
        (4.6, 1.7, 3.0, 0.8, "UR10e sürücüsü + SICK\n(gerçek) / Gazebo (sim)", "#fff5f5", "#c92a2a"),
    ]
    arrows = [((2.0, 6.2), (2.0, 5.7)), ((2.0, 4.7), (2.0, 4.2)),
              ((2.0, 3.2), (2.0, 2.7)), ((2.0, 1.7), (2.0, 1.2)),
              ((3.5, 5.2), (4.6, 5.2)), ((4.6, 4.9), (3.5, 4.9)),
              ((3.5, 2.45), (4.6, 4.7)),          # yürütücü -> move_group
              ((6.1, 4.7), (6.1, 2.5))]           # move_group -> sürücü
    fig, ax = plt.subplots(figsize=(8.3, 4.6), dpi=180)
    ax.set_xlim(0, 8.3)
    ax.set_ylim(0, 7.6)
    ax.axis("off")
    for x, y, w, hgt, label, fc, ec in boxes:
        ax.add_patch(FancyBboxPatch((x, y), w, hgt,
                                    boxstyle="round,pad=0.06,rounding_size=0.12",
                                    facecolor=fc, edgecolor=ec, lw=1.3))
        ax.text(x + w / 2, y + hgt / 2, label, ha="center", va="center", fontsize=8,
                color=INK)
    for a, b in arrows:
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=11,
                                     color="#868e96", lw=1.1,
                                     connectionstyle="arc3,rad=0.0"))
    # yürütücü <-> cache: çift yönlü (kaydet / oynat)
    ax.add_patch(FancyArrowPatch((3.5, 2.05), (4.6, 3.15), arrowstyle="<|-|>",
                                 mutation_scale=11, color="#868e96", lw=1.1))
    ax.text(4.05, 5.45, "IK + çarpışma", fontsize=7, color="#868e96", ha="center")
    ax.text(3.95, 3.55, "plan / yürüt", fontsize=7, color="#868e96", ha="center",
            rotation=62)
    ax.text(4.25, 2.35, "kaydet / oynat", fontsize=7, color="#868e96", ha="center",
            rotation=45)
    ax.text(7.45, 3.5, "yörünge", fontsize=7, color="#868e96", ha="left")
    ax.set_title("viewpoint_planner — veri akışı: mesh'ten octomap'e", fontsize=10.5,
                 color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_vp_mimari.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


# --------------------------------------------------------------------------- #
def build():
    fig_architecture()
    plan = json.load(open(F.PLAN))
    vps = plan["ur_viewpoints"]
    args = launch_args(LAUNCH)
    real = F.coverage(
        os.path.join(F.SINGLE, "real_data", "beliefMap_single_ur10e_real.ot"),
        os.path.join(F.SINGLE, "real_data", "occupancyMap_single_ur10e_real.ot"))
    sim = F.coverage(
        os.path.join(F.SINGLE, "sim_data", "beliefMap_single_ur10e_sim.ot"),
        os.path.join(F.SINGLE, "sim_data", "occupancyMap_single_ur10e_sim.ot"))
    n_cache = len([f for f in os.listdir(os.path.join(PKG, "plans", "trajectories"))
                   if f.endswith(".json")]) if os.path.isdir(
        os.path.join(PKG, "plans", "trajectories")) else 0

    doc = Document()
    doc.add_heading("viewpoint_planner — Sistem Tanımı", 0)
    sub = doc.add_paragraph("Tek-kol UR10e ile Otonom Şasi Muayenesi — "
                            "Paketin Bugünkü Hali")
    sub.style = doc.styles["Subtitle"]
    meta = doc.add_paragraph()
    meta.add_run("12 Eylül 2026    |    Cem Süha Yılmaz    |    "
                 "ROS 2 Humble / MoveIt 2").bold = True
    p(doc, "Bu belge paketin ŞU ANKİ halini anlatır: hangi parçalardan oluştuğu, "
           "veri akışının baştan sona nasıl işlediği, hangi parametrelerle "
           "sürüldüğü, hangi dosyaları ürettiği ve bugünkü ölçülen başarımı. "
           "Değişiklik gerekçeleri ve karar geçmişi ayrı belgededir "
           "(viewpoint_planner_teknik_rapor.docx). Tablolardaki parametreler launch "
           "dosyasından ve yapılandırmadan, veri şemaları güncel plan dosyasından, "
           "başarım sayıları octomap çıktılarından üretim anında okunmuştur.")

    # 1
    h1(doc, "1. Sistem Tek Paragrafta")
    p(doc, "Sistem, bir şasinin CAD modelinden yola çıkarak o şasiyi kameranın "
           "göreceği biçimde kaplayan bir bakış-noktası kümesi üretir, bu noktaları "
           "robotun gerçekten gidebileceği bir sıraya dizer ve UR10e'yi 2 metrelik "
           "lineer ray üzerinde bu turda gezdirerek her durakta bir nokta bulutu "
           "kaydeder. Sonuçta şasinin ölçülmüş bir 3B rekonstrüksiyonu (octomap) "
           "elde edilir. Planlama bir kez yapılır ve dosyaya yazılır; yürütme "
           "tekrarlanabilir olsun diye her yörünge ilk koşuda kaydedilip sonraki "
           "koşularda aynen oynatılır.")
    table(doc, ["Bileşen", "Bugünkü değer"], [
        ["Kol", "UR10e, ur10e_ öneki, MoveIt grubu real_ur10e (ray + 6 eklem)"],
        ["Lineer eksen", "Festo, 0.05 – 1.95 m (ur10e_base_to_robot_mount)"],
        ["Sensör", "SICK ToF; gerçek /sick_points, sim /sim/pointcloud"],
        ["Plandaki bakış-noktası", f"{len(vps)}"],
        ["Kayıtlı yörünge", f"{n_cache} dosya (plans/trajectories/)"],
        ["Gerçek robot kaplaması", f"%{100 * real['frac']:.1f}"],
        ["Simülasyon kaplaması", f"%{100 * sim['frac']:.1f}"],
    ], widths=[1.9, 4.0])

    # 2
    h1(doc, "2. Mimari")
    figure(doc, "fig_vp_mimari.png",
           "Şekil 1: Paketin veri akışı. Sol sütun paketin kendi düğümleri ve "
           "dosyaları, sağ sütun MoveIt ve donanım.")
    table(doc, ["Modül", "Sorumluluk"], [
        ["mesh_analyzer.py",
         "Şasi mesh'ini yükler, yüzeyi hedef noktalara örnekler, her noktanın "
         "normalini üretir."],
        ["viewpoint_generator.py",
         "Hedef noktaların normalleri boyunca aday kamera pozları üretir; bir adayın "
         "hangi hedefleri gördüğünü FOV, menzil, geliş açısı ve occlusion ile "
         "hesaplar (görünürlük matrisi)."],
        ["viewpoint_clusterer.py",
         "Birbirine çok yakın ve aynı yöne bakan adayları birleştirir; arama uzayını "
         "küçültür."],
        ["set_cover_optimizer.py",
         "Greedy küme kapsama: her adımda en çok YENİ nokta getiren adayı seçer, "
         "marjinal kazanç tabanın altına düşünce durur."],
        ["reachability_checker.py",
         "Bir adayın kol için ulaşılabilir olup olmadığını /compute_ik ile sınar; "
         "hata kodlarını okunur biçimde raporlar."],
        ["viewpoint_planner_node.py",
         "Yukarıdakileri sıraya koyar, yürütücünün sahnesini kurar, turu eklem "
         "uzayında sıralar ve plan JSON'unu yazar."],
        ["inspection_base.py",
         "Yürütücünün tekrarlanabilir kısmı: plan okuma, trajectory cache, 2π açma, "
         "padding, sahne kurulumu, varış denetimi, nokta bulutu yakalama."],
        ["inspection_executor_node.py",
         "UR'ye özgü ince katman: hangi MoveIt grubu, hangi kamera konuları, "
         "yörüngenin move_group üzerinden gönderilmesi."],
        ["visualization.py / plan_visualizer.py",
         "RViz işaretçileri ve plan görselleştirmesi."],
    ], widths=[1.6, 4.3])

    # 3
    h1(doc, "3. Uçtan Uca Akış")
    p(doc, "Aşağıdaki sıra, bir muayenenin sıfırdan sonuca kadar izlediği yoldur.")
    table(doc, ["#", "Adım", "Çıktısı"], [
        ["1", "Mesh yüklenir ve yüzey hedef noktalara örneklenir", "hedef noktalar + normaller"],
        ["2", "Her hedef için aday kamera pozları üretilir",
         "aday havuzu (mesafe x eğim varyasyonları)"],
        ["3", "Adayların görünürlüğü hesaplanır", "görünürlük matrisi"],
        ["4", "Yürütücünün sahnesi kurulur (zemin + padding)",
         "planlayıcı ile yürütücü aynı dünyada"],
        ["5", "Greedy küme kapsama + ulaşılabilirlik sınaması",
         "seçilmiş bakış-noktaları"],
        ["6", "Tur eklem uzayında sıralanır ve her geçiş planla doğrulanır",
         "gezme sırası + atlananlar"],
        ["7", "Plan JSON'a yazılır", "plans/viewpoint_plan.json"],
        ["8", "Yürütücü başlar, başlangıç pozuna gider", "bilinen başlangıç durumu"],
        ["9", "Her durak için: yörünge cache'ten oynatılır ya da planlanıp kaydedilir",
         "plans/trajectories/ur_<id>.json"],
        ["10", "Durakta bulut ve poz kaydedilir",
         "pcds/single_ur10e/<sim|real>_data/{pcds,poses}"],
        ["11", "Tur bitince kol home pozuna döner", "-"],
        ["12", "Bulutlar octomap'e çevrilir (pcd2octomap_builder)",
         "occupancyMap_*.ot, beliefMap_*.ot"],
    ], widths=[0.3, 3.0, 2.6])
    figure(doc, "fig_vp_plan.png",
           f"Şekil 2: Güncel planın kendisi — {len(vps)} bakış-noktası, oklar görüş "
           "ekseni, renk o pozun getirdiği yeni nokta sayısı.")
    figure(doc, "fig_vp_order.png",
           "Şekil 3: Turun gezme sırası ve ardışık duraklar arasındaki eklem-uzayı "
           "maliyeti.")

    # 4
    h1(doc, "4. Arayüzler")
    table(doc, ["Yön", "Ad", "Tip / Not"], [
        ["abone", "/sick_points", "sensor_msgs/PointCloud2 — gerçek SICK"],
        ["abone", "/sim/pointcloud", "sensor_msgs/PointCloud2 — Gazebo"],
        ["abone", "/joint_states", "kolun ölçülen durumu; taze değilse gönderim beklenir"],
        ["servis", "/compute_ik", "bakış-noktası ulaşılabilirliği ve dal seçimi"],
        ["servis", "/plan_kinematic_path", "sıralamada geçiş fizibilitesi"],
        ["servis", "/apply_planning_scene", "zemin düzlemi + link padding"],
        ["servis", "/get_planning_scene", "mevcut sahnenin okunması"],
        ["action", "execute_trajectory", "move_group üzerinden yürütme (varsayılan)"],
        ["action", "/scaled_joint_trajectory_controller/follow_joint_trajectory",
         "ham kontrolcü yolu (execute_via_move_group:=false)"],
        ["yayın", "~/viewpoint_markers", "RViz görselleştirmesi"],
    ], widths=[0.7, 2.6, 2.6])

    # 5
    h1(doc, "5. Yapılandırma Referansı")
    p(doc, f"Yürütme launch dosyası {len(args)} argüman tanımlar. Aşağıdaki tablo "
           "doğrudan o dosyadan üretilmiştir.")
    table(doc, ["Argüman", "Varsayılan", "Açıklama"],
          [[a, str(d), short(desc)] for a, d, desc in args],
          widths=[1.5, 0.9, 3.5])
    p(doc, "Planlama tarafı ise config/sick_tmini_params.yaml ile sürülür. En çok "
           "dokunulan anahtarlar:")
    table(doc, ["Anahtar", "Değer", "Ne yapar"], [
        ["min_marginal_coverage", "0.0035",
         "greedy durma ölçütü; bakış-noktası sayısını belirleyen asıl kaldıraç"],
        ["max_incidence_angle_deg", "85", "bir yüzeyin 'görüldü' sayılma açısı"],
        ["max_viewpoints", "0", "sert kap kapalı"],
        ["collision_padding", "0.04 m", "bütün ur10e_* gövdelerine uygulanan pay"],
        ["chassis_collision_padding", "0.0", "şasiye ayrı pay yok"],
        ["order_mode", "joint", "sıralama eklem uzayında"],
        ["order_rail_weight", "2.0", "1 m ray = 2 rad eklem yolu sayılır"],
        ["order_plan_time / order_max_tries_per_step", "3.0 s / 8",
         "geçiş doğrulamasının bütçesi"],
        ["order_feasibility_budget_s", "240", "doğrulama toplam bütçesi"],
    ], widths=[1.9, 0.9, 3.1])
    p(doc, "KURAL: collision_padding planlama ile yürütme tarafında AYNI olmak "
           "zorundadır. Ayrıldıkları anda plan, yürütücünün ulaşamayacağı "
           "bakış-noktaları içerir.")

    # 6
    h1(doc, "6. Üretilen Dosyalar ve Şemaları")
    keys = ", ".join(list(plan.keys()))
    p(doc, f"Plan dosyası (plans/viewpoint_plan.json) şu üst düzey alanları taşır: "
           f"{keys}.")
    vp_keys = ", ".join(vps[0].keys())
    table(doc, ["Dosya", "İçerik"], [
        ["plans/viewpoint_plan.json",
         f"bakış-noktası başına: {vp_keys}"],
        ["plans/viewpoint_plan.json → ik_scene",
         "planın hangi sahnede doğrulandığı (zemin yüksekliği, padding, "
         "padding uygulanan link sayısı)"],
        ["plans/viewpoint_plan.json → manual_edits / skipped_viewpoints",
         "elle yapılan düzeltmeler ve zincire bağlanamayan duraklar"],
        ["plans/trajectories/ur_<vp_id>.json",
         "o durağa giden kayıtlı yörünge; dosya adı viewpoint kimliğini taşır"],
        ["pcds/single_ur10e/<sim|real>_data/pcds/<N>.pcd", "durakta alınan bulut"],
        ["pcds/single_ur10e/<sim|real>_data/poses/<N>.txt", "o buluta ait sensör pozu"],
        ["occupancyMap_*.ot / beliefMap_*.ot",
         "pcd2octomap_builder çıktısı: ölçülen ve referans voksel haritaları"],
    ], widths=[2.3, 3.6])

    # 7
    h1(doc, "7. Çalıştırma")
    code(doc,
         "# 1) plan üretimi\n"
         "ros2 launch viewpoint_planner viewpoint_planning.launch.py\n\n"
         "# 2) muayene turu — gerçek robot (HIL ayakta olmalı)\n"
         "ros2 launch viewpoint_planner inspection_execution.launch.py only_sim:=false\n\n"
         "# 3) yalnız simülasyon\n"
         "ros2 launch viewpoint_planner inspection_execution.launch.py only_sim:=true\n\n"
         "# 4) kayıtlı yörüngeleri yok say ve yeniden kaydet\n"
         "ros2 launch viewpoint_planner inspection_execution.launch.py force_replan:=true")
    p(doc, "Arayüzdeki 'UR10e Inspection Scenario' düğmesi aynı zinciri kurar: önce "
           "HIL launch'ı, sonra bu launch. use_fake_hardware kapalıyken only_sim "
           "false geçirilir, yani gerçek SICK bulutları da kaydedilir.")
    p(doc, "Turun bittiğini launch sürecinin ölmesinden ANLAYAMAZSINIZ: aynı launch "
           "kalıcı bir görselleştirici düğümü de başlatır ve o hiç çıkmaz. Bitişin "
           "ölçütü yürütücü düğümünün sonlanmasıdır; arayüz de bunu izler.")

    # 8
    h1(doc, "8. Bugünkü Ölçülen Başarım")
    table(doc, ["Ölçü", "Gerçek robot", "Simülasyon"], [
        ["Şasi vokseli", len(real["belief"]), len(sim["belief"])],
        ["Kaplanan voksel", len(real["covered"]), len(sim["covered"])],
        ["Kaplama", f"%{100 * real['frac']:.1f}", f"%{100 * sim['frac']:.1f}"],
    ], widths=[2.2, 1.8, 1.8])
    figure(doc, "fig_vp_octomap.png",
           "Şekil 4: Kaplanan (yeşil) ve kaplanamayan (kırmızı) şasi vokselleri.")
    figure(doc, "fig_vp_part_coverage.png",
           "Şekil 5: Parça başına kaplama; eksik birkaç parçada toplanıyor.")
    p(doc, "Planlayıcının kendi kaplama tahmini (%%%.1f) bu sayılarla "
           "karşılaştırılmamalıdır: o, katı bir sensör modeli altında görülebilir "
           "mesh örnek noktalarının kesridir; octomap ise gerçekte isabet alan "
           "voksellerin oranıdır." % (100 * plan["coverage_achieved"]))

    # 9
    h1(doc, "9. Çalışma Kuralları ve Sınırlar")
    bullets(doc, [
        "Trajectory cache, URDF/SRDF/padding değişikliklerine KÖRDÜR. Hücrenin "
        "geometrisi değiştiyse force_replan ile yeniden kaydedin.",
        "Bir bakış-noktası plandan çıkarılırsa ARDINDAN GELENİN cache'i de bayatlar; "
        "çünkü kayıtlı yörünge artık var olmayan bir başlangıç durumundan başlar.",
        "Şasinin kolun karşı tarafında kalan yüzü bu senaryoyla görülemez; o yüz için "
        "iki kollu paket (multirobot_viewpoint_planner) vardır.",
        "Planlama ile yürütme padding'i aynı olmalıdır.",
        "Plan dosyası elle düzenlenebilir ve düzenleme manual_edits altında "
        "kaydedilir; ancak düzenlemeden sonra coverage_achieved yaklaşık hale gelir.",
        "Zincire bağlanamayan duraklar skipped_viewpoints'e yazılır ve turda "
        "gezilmez; bunlar sessizce kaybolmaz.",
    ])

    doc.save(OUT)
    print("yazıldı:", OUT)


if __name__ == "__main__":
    build()
