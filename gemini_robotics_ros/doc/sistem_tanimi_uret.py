#!/usr/bin/env python3
"""gemini_robotics_ros paketinin BUGÜNKÜ halini anlatan sistem tanımını üretir.

    python3 doc/kosu_figurleri.py       # koşu figürleri (bir kez yeterli)
    python3 doc/sistem_tanimi_uret.py

Bu belge bir gelişme raporu DEĞİLDİR: neyin ne zaman değiştiğini değil, sistemin
şu anda nasıl çalıştığını anlatır. Gelişme/karar kaydı ayrı dosyadadır
(gemini_robotics_ros_raporu.docx, build_report.py ile üretilir).

Parametre sayıları config/gemini_params.yaml'dan, koşu sonuçları
recordings/gemini_robotics/ altındaki kayıtlardan üretim anında okunur.
"""
import ast
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

import yaml
from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Inches, Pt, RGBColor

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import kosu_figurleri as K  # noqa: E402

OUT = os.path.join(HERE, "gemini_pick_place_sistem_tanimi.docx")
FIG = os.path.join(HERE, "figures")
LAUNCH = os.path.join(PKG, "launch", "gemini_pick_place.launch.py")
PARAMS = os.path.join(PKG, "config", "gemini_params.yaml")
FIG_WIDTH = Inches(5.9)
INK = "#22252a"


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
    path = os.path.join(FIG, filename)
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} yok — önce: python3 doc/kosu_figurleri.py")
    doc.add_picture(path, width=FIG_WIDTH)
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap = doc.add_paragraph()
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = cap.add_run(caption)
    run.italic = True
    run.font.size = Pt(8.5)
    run.font.color.rgb = RGBColor(0x55, 0x55, 0x55)


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


def param_count():
    d = yaml.safe_load(open(PARAMS))
    node = d.get("/**", d)
    params = node.get("ros__parameters", node) if isinstance(node, dict) else {}
    return len(params), params


# --------------------------------------------------------------------------- #
def fig_architecture():
    boxes = [
        (0.2, 6.2, 3.3, 0.75, "/gemini/command\n(doğal dil görev metni)", "#e7f5ff", "#1f6feb"),
        (0.2, 4.9, 3.3, 0.95, "pick_place_node\ndurum makinesi + görev akışı", "#fff4e6", "#d9480f"),
        (0.2, 3.5, 1.55, 0.95, "locator\nnokta + normal", "#f8f9fa", "#868e96"),
        (1.95, 3.5, 1.55, 0.95, "grasp\nemme pozu", "#f8f9fa", "#868e96"),
        (0.2, 2.2, 1.55, 0.9, "payload\nboyut ölçümü", "#f8f9fa", "#868e96"),
        (1.95, 2.2, 1.55, 0.9, "reachability\nIK sınaması", "#f8f9fa", "#868e96"),
        (0.2, 0.9, 3.3, 0.8, "recorder_node\nkoşu kaydı (events, csv, frames)", "#ebfbee", "#2f9e44"),
        (4.5, 6.2, 3.5, 0.75, "Gemini Robotics ER 2\n(bulut model; nokta döndürür)", "#f3f0ff", "#7048e8"),
        (4.5, 4.9, 3.5, 0.95, "depth_render\nToF derinliğinden model girdisi", "#fff9db", "#e8590c"),
        (4.5, 3.5, 3.5, 0.95, "MoveIt 2 (move_group)\nplanlama + çarpışma", "#f3f0ff", "#7048e8"),
        (4.5, 2.2, 3.5, 0.9, "UR10e + Festo rayı", "#fff5f5", "#c92a2a"),
        (4.5, 0.9, 3.5, 0.8, "VGC10 vakum kavrayıcı\n(Modbus TCP)", "#fff5f5", "#c92a2a"),
    ]
    arrows = [((1.85, 6.2), (1.85, 5.9)), ((1.85, 4.9), (1.85, 4.5)),
              ((1.85, 3.5), (1.85, 3.1)), ((1.85, 2.2), (1.85, 1.7)),
              ((3.5, 5.72), (4.5, 6.45)),     # düğüm -> model (sorgu)
              ((4.5, 6.22), (3.5, 5.52)),     # model -> düğüm (2D nokta)
              ((3.5, 5.25), (4.5, 5.45)),     # düğüm -> depth_render
              ((6.9, 5.85), (6.9, 6.2)),      # render -> model
              ((3.5, 5.05), (4.5, 4.25)),     # düğüm -> MoveIt
              ((6.25, 3.5), (6.25, 3.1)),     # MoveIt -> robot
              ((3.5, 4.92), (4.5, 1.5))]      # düğüm -> vakum
    fig, ax = plt.subplots(figsize=(8.4, 4.8), dpi=180)
    ax.set_xlim(0, 8.3)
    ax.set_ylim(0.6, 7.3)
    ax.axis("off")
    for x, y, w, hgt, label, fc, ec in boxes:
        ax.add_patch(FancyBboxPatch((x, y), w, hgt,
                                    boxstyle="round,pad=0.06,rounding_size=0.12",
                                    facecolor=fc, edgecolor=ec, lw=1.3))
        ax.text(x + w / 2, y + hgt / 2, label, ha="center", va="center", fontsize=7.6,
                color=INK)
    for a, b in arrows:
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=11,
                                     color="#868e96", lw=1.1))
    ax.text(3.95, 6.22, "sorgu + görüntü", fontsize=6.6, color="#868e96", ha="center")
    ax.text(3.95, 5.62, "2D nokta", fontsize=6.6, color="#868e96", ha="center")
    ax.text(3.95, 5.12, "görüntü iste", fontsize=6.6, color="#868e96", ha="center")
    ax.text(7.0, 5.95, "render", fontsize=6.6, color="#868e96", ha="left")
    ax.text(3.95, 4.42, "poz hedefi", fontsize=6.6, color="#868e96", ha="center")
    ax.text(3.95, 2.05, "tut / bırak", fontsize=6.6, color="#868e96", ha="center")
    ax.set_title("gemini_robotics_ros — dil komutundan kavramaya veri akışı",
                 fontsize=10.5, color=INK)
    fig.tight_layout()
    out = os.path.join(FIG, "fig_pickplace_mimari.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


# --------------------------------------------------------------------------- #
def build():
    os.makedirs(FIG, exist_ok=True)
    fig_architecture()
    runs = K.load_runs()
    n_par, params = param_count()
    args = launch_args(LAUNCH)
    ok = [r for r in runs if r["outcome"] == "DONE"]
    durs = [r["duration"] for r in runs if r["duration"] == r["duration"]]
    lat = [v for r in runs for v in r["latency"]]

    doc = Document()
    doc.add_heading("gemini_robotics_ros — Sistem Tanımı", 0)
    sub = doc.add_paragraph("Doğal Dil Komutuyla Pick & Place — Paketin Bugünkü Hali")
    sub.style = doc.styles["Subtitle"]
    meta = doc.add_paragraph()
    meta.add_run("12 Eylül 2026    |    Cem Süha Yılmaz    |    "
                 "ROS 2 Humble / MoveIt 2").bold = True
    p(doc, "Bu belge paketin ŞU ANKİ halini anlatır: parçaları, bir görevin baştan "
           "sona nasıl işlediği, arayüzleri, yapılandırması ve bugünkü ölçülen "
           "başarımı. Geliştirme sırasında bulunan hatalar, denenip geri alınanlar "
           "ve gerekçeler ayrı belgededir (gemini_robotics_ros_raporu.docx).")

    # 1
    h1(doc, "1. Sistem Tek Paragrafta")
    p(doc, "Operatör düz bir cümle yazar: 'konveyördeki parçayı al ve raftaki boş "
           "gözlerden birine koy'. Sistem bu cümleyi Gemini Robotics ER 2'ye, "
           "derinlik kamerasından üretilmiş bir görüntüyle birlikte sorar; model "
           "görüntü üzerinde bir NOKTA gösterir. Sistem o noktayı derinlikten 3B "
           "dünya koordinatına çevirir, çevresindeki yüzeye bir düzlem oturtarak "
           "emme yönünü bulur, MoveIt'e sorup hedefin gerçekten ulaşılabilir "
           "olduğunu doğrular, vakumla kavrar, parçayı sahneye iliştirir, hedefe "
           "taşır ve bırakır. Model aksiyon üretmez; aksiyonu bu paket üretir.")
    table(doc, ["Bileşen", "Bugünkü değer"], [
        ["Model", "Gemini Robotics ER 2 (bulut); sahte/mock arka uç da var"],
        ["Kamera", "SICK Visionary-T Mini — ToF, RGB YOK; modele derinlikten "
                   "üretilen render gider"],
        ["Kol", "UR10e + Festo lineer rayı"],
        ["Kavrayıcı", "OnRobot VGC10 vakum (aktüe eklem yok, Modbus TCP)"],
        ["Yapılandırma", f"{n_par} parametre (config/gemini_params.yaml)"],
        ["Durumlar", "STARTUP_SCAN → READY → START → PLANNED → PICK_LOCATED → "
                     "PLACE_LOCATED → GRASPED → RELEASED → DONE (hata: FAILED)"],
        ["Kayıtlı koşu", f"{len(runs)} (28 Ağustos 2026, gerçek hücre)"],
    ], widths=[1.7, 4.2])

    # 2
    h1(doc, "2. Mimari")
    figure(doc, "fig_pickplace_mimari.png",
           "Şekil 1: Dil komutundan kavramaya veri akışı. Sol sütun paketin kendi "
           "modülleri, sağ sütun model, MoveIt ve donanım.")
    table(doc, ["Modül", "Sorumluluk"], [
        ["pick_place_node.py",
         "Görev akışı ve durum makinesi; bütün parçaları sırayla kullanır."],
        ["er_client.py",
         "Gemini ER 2 istemcisi. Model robot aksiyonu üretmez; görüntü üzerinde "
         "nokta ve metin döndürür."],
        ["depth_render.py",
         "ToF derinliğinden modele verilecek görüntüyü üretir (kabartma/relief "
         "kipi dahil). Kameranın RGB'si olmadığı için bu adım zorunludur."],
        ["locator.py",
         "'Şunu bul' → dünya çerçevesinde 3B nokta ve yüzey normali. Hem algı "
         "düğümü hem görev düğümü bunu kullanır."],
        ["geometry.py", "2D piksel → 3B dünya dönüşümleri (deprojeksiyon + TF)."],
        ["grasp.py",
         "Modelin verdiği 2D noktadan emme noktası ve yaklaşma yönü üretir; "
         "oryantasyonu yüzeyin normalinden kurar."],
        ["reachability.py",
         "Hedefe gerçekten gidilebilir mi? /compute_ik ile sınar; kavramadan ÖNCE."],
        ["payload.py",
         "Kavranan parçanın boyutunu derinlikten ölçer; parça sahneye o boyutla "
         "iliştirilir."],
        ["vacuum.py", "VGC10 arayüzü: tutma/bırakma ve ölçülen vakum geri beslemesi."],
        ["tool_geometry.py",
         "Uç eleman geometrisini URDF ve mesh'ten ÖLÇER (elle sabit girmek yerine)."],
        ["recorder_node.py / telemetry.py",
         "Koşu kaydı: olaylar, ölçümler, kareler ve ham derinlik."],
        ["perception_node.py",
         "Yalnız algı: sorgu → 3B tespit + RViz işaretçisi; robotu hareket ettirmez."],
    ], widths=[1.6, 4.3])

    # 3
    h1(doc, "3. Bir Görev Baştan Sona")
    table(doc, ["#", "Adım", "Ayrıntı"], [
        ["1", "Açılışta tarama pozuna gidilir", "STARTUP_SCAN → READY"],
        ["2", "Görev metni /gemini/command'e gelir", "START"],
        ["3", "Model plan çağrısı ile görevi yorumlar", "PLANNED"],
        ["4", "Derinlikten render üretilip modele 'kaynağı göster' sorulur",
         "2D nokta"],
        ["5", "Nokta deprojekte edilir, çevresine düzlem oturtulur",
         "3B nokta + normal (PICK_LOCATED)"],
        ["6", "Hedef ulaşılabilir mi diye IK sınanır",
         "gerekirse yeni tarama pozu denenir"],
        ["7", "Aynısı bırakma hedefi için yapılır", "PLACE_LOCATED"],
        ["8", "Yaklaşma → temas → vakum kurulur", "GRASPED"],
        ["9", "Parça ölçülüp sahneye iliştirilir", "MoveIt parçayı taşırken görür"],
        ["10", "Hedefe taşınır, bırakılır", "RELEASED → DONE"],
    ], widths=[0.3, 2.4, 3.2])
    figure(doc, "fig_run_phases.png",
           "Şekil 2: Gerçek bir görevin aşama zaman çizgisi; turuncu oklar model "
           "çağrılarıdır.")
    p(doc, "Tarama pozları birden fazladır (örneğin konveyör ve raf için ayrı "
           "pozlar): model bir hedefi bulamazsa ya da hedef ulaşılamazsa sistem bir "
           "sonraki tarama pozuna geçip yeniden sorar. Kayıtlardaki scan_poses.csv "
           "tam olarak bu denemeleri tutar.")

    # 4
    h1(doc, "4. Arayüzler")
    table(doc, ["Yön", "Ad", "Not"], [
        ["abone", "/gemini/command", "doğal dil görev metni (std_msgs/String)"],
        ["yayın", "/gemini/status", "insan-okur durum akışı"],
        ["yayın", "/gemini/record", "ölçüm akışı (JSON); recorder_node CSV'ye açar"],
        ["yayın", "/gemini/detections", "algı sonuçları"],
        ["yayın", "/gemini/er_image", "modele giden görüntünün kendisi"],
        ["abone", "/depth, /intensity, /camera_info", "ToF kamerası"],
        ["abone", "/OnRobotVGInput", "vakum geri beslemesi"],
        ["servis", "/compute_ik", "ulaşılabilirlik sınaması"],
        ["MoveIt", "move_group", "planlama, sahne ve yürütme"],
    ], widths=[0.7, 2.3, 2.9])

    # 5
    h1(doc, "5. Yapılandırma")
    p(doc, f"Launch dosyası yalnızca {len(args)} argüman alır; asıl ayar yüzeyi "
           f"config/gemini_params.yaml'daki {n_par} parametredir.")
    table(doc, ["Argüman", "Varsayılan", "Açıklama"],
          [[a, str(d), (desc[:150] + "…") if len(desc) > 150 else desc]
           for a, d, desc in args], widths=[1.0, 0.8, 4.1])
    table(doc, ["Parametre grubu", "Örnek anahtarlar", "Ne yapar"], [
        ["Model", "backend, model, thinking_level, jpeg_quality",
         "hangi model, hangi arka uç, görüntü kalitesi"],
        ["Görüntü üretimi", "er_image_source, render_mode, render_min_m, render_max_m",
         "modele giden görüntünün nasıl üretileceği"],
        ["Yüzey ölçümü", "patch_radius_px, patch_depth_band, max_surface_rms, "
                         "max_surface_tilt_deg",
         "emme yüzeyinin kabul ölçütleri"],
        ["Hareket", "moveit_backend, planner_id, max_velocity, planning_time",
         "planlama ve hız ayarları"],
        ["Tarama", "scan_pose_names, scan_pose_joints, move_to_scan_on_start",
         "hangi pozlardan bakılacağı"],
        ["Kavrama", "tool_tip_offset, tool_approach_vector, approach_candidates, "
                    "touch_offset, lift_distance",
         "yaklaşma, temas ve kaldırma geometrisi"],
    ], widths=[1.2, 2.4, 2.3])

    # 6
    h1(doc, "6. Çalıştırma")
    code(doc,
         "# gerçek hücre (kayıt açık)\n"
         "ros2 launch gemini_robotics_ros gemini_pick_place.launch.py \\\n"
         "    mode:=real record:=true note:=\"kucuk kutu, bandin ortasinda\"\n\n"
         "# simülasyon\n"
         "ros2 launch gemini_robotics_ros gemini_pick_place.launch.py mode:=sim\n\n"
         "# görevi ver\n"
         "ros2 topic pub --once /gemini/command std_msgs/String \\\n"
         "    \"{data: 'Pick up the object on the conveyor belt and place it into an \"\n"
         "    \"empty bin on the topmost level of the toolkit rack'}\"")
    p(doc, "Arayüzdeki 'Pick & Place Scenario' düğmesi HIL'i use_vacuum_gripper:=true "
           "ile kaldırır, senaryoyu mode:=sim|real ile başlatır ve görev metnini "
           "yazabileceğiniz bir komut kutusu açar.")

    # 7
    h1(doc, "7. Bugünkü Ölçülen Başarım")
    p(doc, f"28 Ağustos 2026'da gerçek hücrede {len(runs)} görev koşuldu ve hepsi "
           f"kaydedildi. {len(ok)} koşuda zincir baştan sona hatasız yürüdü. "
           f"Ortalama süre {sum(durs) / len(durs):.0f} s; bunun koşu başına "
           f"ortalama {sum(lat) / len(runs):.0f} saniyesi model çağrısı beklemekle "
           f"geçiyor (çağrı başına {sum(lat) / len(lat):.1f} s).")
    table(doc, ["#", "Koşu", "Not", "Sonuç", "Süre", "ER", "Gecikme", "Tespit"],
          K.run_table(runs), widths=[0.25, 0.9, 1.9, 0.8, 0.5, 0.4, 0.6, 0.5])
    figure(doc, "fig_runs_outcome.png",
           "Şekil 3: Koşu süreleri, sonuçları ve başarısızlık nedenleri.")
    figure(doc, "fig_er_latency.png",
           "Şekil 4: Model çağrılarının gecikme dağılımı.")
    p(doc, "Tablodaki 'BAŞARILI', zincirin hatasız yürüdüğü anlamına gelir; hedefe "
           "doğru yere bırakıldığı anlamına GELMEZ. İki koşuda model raf konumunu "
           "yanlış gösterdiği için parça konveyöre geri bırakıldı ve akış yine "
           "tamamlanmış sayıldı. Bırakma noktasının hedef rafın sınırları içinde "
           "olup olmadığı bugün sınanmıyor.")

    # 8
    h1(doc, "8. Çalışma Kuralları ve Sınırlar")
    bullets(doc, [
        "Model aksiyon üretmez, yalnız nokta gösterir: yönelim, yaklaşma ve "
        "ulaşılabilirlik bu paketin işidir.",
        "Kamera RGB yayınlamaz; modele giden görüntü her zaman derinlikten "
        "üretilir. Render kipi değiştiğinde modelin gördüğü şey de değişir.",
        "Vakum düz ve gözeneksiz yüzey ister. Biçimsiz veya gözenekli cisimlerde "
        "kavrama kurulamıyor; kayıtlardaki en sık başarısızlık nedeni budur.",
        "Kavranan parça, ölçülen boyutuyla planlama sahnesine iliştirilir; aksi "
        "halde MoveIt taşınan parçayı görmez.",
        "Model çağrısı çevrim süresinin görünür bir bölümünü yer ve robot o süre "
        "boyunca bekler.",
        "Kayıt açıkken her koşu kendi klasörüne yazar; ham derinlik saklandığı için "
        "render kipi çevrimdışı olarak sonradan denenebilir.",
    ])

    doc.save(OUT)
    print("yazıldı:", OUT)


if __name__ == "__main__":
    build()
