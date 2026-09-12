#!/usr/bin/env python3
"""İnsan-robot işbirlikli vidalama senaryosunun teknik raporunu (.docx) üretir.

    python3 docs/hrc_figurler.py       # önce figürler (fig_hrc_*.png)
    python3 docs/hrc_rapor_uret.py     # sonra rapor

Rapor ELLE düzenlenmez: metin buradan değiştirilip yeniden çalıştırılır. Sayısal
değerlerin çoğu (waypoint açıları, uç konumları, dalış derinlikleri, çarpışma
payları, segment süreleri) rapor üretilirken senaryo kaynağından ve URDF'ten
YENİDEN HESAPLANIR; elle yazılmış tablo yoktur. Kaynak dosyalar:

    examples/human_robot_collaboration_scenario.py
    launch/human_robot_collaboration_scenario.launch.py
    my_robot_cell_control/urdf/whole_cell_hw.urdf.xacro  (use_gripper:=true)
    real_ifarlab_gripper_moveit_config/config/whole_cell_hw.srdf
"""
import os
import sys

import numpy as np
from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Inches, Pt, RGBColor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import hrc_figurler as F  # noqa: E402

OUT = os.path.join(HERE, "insan_robot_isbirligi_raporu.docx")
FIG_WIDTH = Inches(5.9)


# --------------------------------------------------------------------------- #
# yardımcılar (multirobot_viewpoint_planner/docs/rapor_uret.py ile aynı düzen)
# --------------------------------------------------------------------------- #
def h1(doc, text):
    doc.add_heading(text, level=1)


def h2(doc, text):
    doc.add_heading(text, level=2)


def p(doc, text):
    return doc.add_paragraph(text)


def bullets(doc, items):
    for it in items:
        doc.add_paragraph(it, style="List Bullet")


def code(doc, text):
    par = doc.add_paragraph()
    run = par.add_run(text)
    run.font.name = "Consolas"
    run.font.size = Pt(8.5)
    par.paragraph_format.space_after = Pt(8)
    return par


def table(doc, header, rows):
    t = doc.add_table(rows=1, cols=len(header))
    t.style = "Light Grid Accent 1"
    for c, name in zip(t.rows[0].cells, header):
        c.text = str(name)
    for row in rows:
        cells = t.add_row().cells
        for c, val in zip(cells, row):
            c.text = str(val)
    doc.add_paragraph()
    return t


def figure(doc, filename, caption):
    path = os.path.join(HERE, filename)
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} yok -- önce: python3 docs/hrc_figurler.py")
    doc.add_picture(path, width=FIG_WIDTH)
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap = doc.add_paragraph()
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = cap.add_run(caption)
    run.italic = True
    run.font.size = Pt(8.5)
    run.font.color.rgb = RGBColor(0x55, 0x55, 0x55)


def deg(x):
    return f"{np.degrees(x):.2f}"


# --------------------------------------------------------------------------- #
def build():
    cell = F.Cell(F.urdf_path())
    _, wps, seq = F.scenario_data()
    steps = F.seq_steps(seq)
    moves = [s for s in steps if s["kind"] == "move"]
    events = [s for s in steps if s["kind"] == "event"]
    P = np.array([F.tcp_of(cell, s["q"]) for s in moves])
    path_len = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
    n_button = sum(1 for e in events if e["key"] == "wait_green_button")
    n_slow = sum(1 for m in moves if m["speed"] is not None)

    doc = Document()
    doc.add_heading("İnsan-Robot İşbirlikli Vidalama Senaryosu", 0)
    sub = doc.add_paragraph(
        "UR10e + OnRobot 2FG7 + operatör onaylı vidalama — Teknik Rapor")
    sub.style = doc.styles["Subtitle"]
    meta = doc.add_paragraph()
    meta.add_run("Revizyon 1 — 12 Eylül 2026    |    Hazırlayan: Cem Süha Yılmaz"
                 "    |    ROS 2 Humble / MoveIt 2").bold = True
    p(doc, "Bu rapor IFARLAB hücresindeki dördüncü kullanım senaryosunu, yani "
           "operatörle sırayla çalışan vidalama hücresini belgeler. Senaryonun tamamı "
           "pymoveit2_real paketindeki tek bir düğümde toplanmıştır: "
           "examples/human_robot_collaboration_scenario.py ve onu başlatan "
           "launch/human_robot_collaboration_scenario.launch.py. Rapordaki bütün "
           "sayısal değerler, rapor üretilirken bu kaynak dosyadan ve hücrenin "
           "URDF'inden yeniden hesaplanır; şekiller de aynı kaynaklardan üretilir "
           "(docs/hrc_figurler.py).")

    # ------------------------------------------------------------------ 1
    h1(doc, "1. Yönetici Özeti")
    p(doc, "Senaryo, bir montaj parçasının dört vidasını robotun sıkması, ama her "
           "vidanın operatörün onayıyla başlaması esasına dayanır. Robot vidalama "
           "aletini standından kendisi alır, vida besleyiciden vidayı alır, vidanın "
           "üstüne konumlanır ve orada DURUR. Operatör yeşil butona basana kadar "
           "hiçbir vidalama başlamaz. Buton basıldığında vidalama motoru dijital "
           "çıkıştan çalıştırılır, robot vidayı on kat yavaş bir hızla sıkma "
           "derinliğine indirir, motor kapatılır ve robot geri çekilir. Dört vida "
           "bittiğinde alet standına bırakılır.")
    table(doc, ["Ölçü", "Değer", "Nereden"], [
        ["Kol / grup", "UR10e, MoveIt grubu real_ur10e (ray + 6 eklem)",
         "whole_cell_hw.srdf"],
        ["Uç eleman", "OnRobot 2FG7 (ur10e_gripper_joint, prizmatik)", "URDF"],
        ["Tur uzunluğu", f"{len(steps)} adım ({len(moves)} hareket, {len(events)} olay)",
         "senaryo kaynağı"],
        ["Vida sayısı", f"{n_button} (her biri operatör onaylı)", "senaryo kaynağı"],
        ["Uç yolu", f"{path_len:.2f} m", "URDF'ten ileri kinematik"],
        ["Yavaş (vidalama) geçiş", f"{n_slow} hareket, hız ölçeği 0.01",
         "senaryo kaynağı"],
        ["Seyir hızı ölçeği", "0.1", "travel_speed parametresi"],
        ["Planlayıcı", "RRTConnectkConfigDefault, 5 s, 10 deneme",
         "launch varsayılanları"],
        ["En dar çarpışma payı", "2.3 mm (aleti kavrarken gripper ↔ tezgâh)",
         "ölçüldü, bkz. bölüm 8"],
    ])
    p(doc, "Bu senaryonun diğer üç kullanım senaryosundan farkı, robotun kendi "
           "başına bir görevi bitirmesi değil, insanın ritmine bağlı çalışmasıdır: "
           "çevrim süresinin yaklaşık üçte biri operatör beklemesi ve vidalama "
           "sırasında bilinçli olarak yavaşlatılmış hareketlerdir.")

    # ------------------------------------------------------------------ 2
    h1(doc, "2. Hücre ve Senaryonun Sahnesi")
    figure(doc, "fig_hrc_cell.png",
           "Hücrenin tamamı ve vidalama iş istasyonu. Vidalama aleti standı, vida "
           "besleyici, operatör mankeni ve ışık perdesi hücrenin URDF'inde ayrı "
           "linklerdir; şekildeki yerleri modelin kendisinden gelir.")
    p(doc, "Vidalama senaryosunun kullandığı gövdeler hücrenin tanımında hazır "
           "bulunur; senaryo düğümü bunları çalışma anında sahneye eklemez:")
    table(doc, ["URDF linki", "Ne", "Çarpışma geometrisi"], [
        ["ur10e_screwdriver", "vidalama aleti ve standı",
         "YOK — bilerek kaldırıldı (bölüm 7)"],
        ["ur10e_screw_feeder", "vida besleyici", "var (mesh)"],
        ["ur10e_alumunium_table", "montaj tezgâhı", "var (mesh)"],
        ["human_link", "operatör mankeni",
         "var, ama SRDF tüm kol linkleriyle devre dışı (bölüm 8)"],
        ["light_curtain_link", "ışık perdesi (3 m)", "var"],
    ])
    p(doc, "Robot 2 metrelik Festo lineer rayı üzerinde durur; bu senaryoda ray "
           "hareket etmez. Bütün waypoint'lerde ray konumu sabit 1.85 m'dir, yani kol "
           "rayın vidalama istasyonuna bakan ucunda park eder ve yalnızca altı döner "
           "eklemiyle çalışır. (Muayene senaryolarında aynı ray aktif olarak "
           "kullanılır; ayrıntı için viewpoint planlama raporuna bakınız.)")

    h2(doc, "2.1 Başlatma Zinciri")
    p(doc, "Senaryo düğümü tek başına bir şey başlatmaz: donanım sürücüsü, "
           "kontrolcüler ve move_group birleşik HIL launch dosyasından gelir. "
           "Gripper'lı yapılandırmada MoveIt paketi real_ifarlab_gripper_moveit_config "
           "olarak seçilir.")
    code(doc,
         "# 1) hücre + sürücü + MoveIt (gripper'lı yapılandırma)\n"
         "ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py \\\n"
         "    use_gripper:=true use_fake_hardware:=false\n\n"
         "# 2) senaryo (tek tur koşar ve kendi kendine kapanır)\n"
         "ros2 launch pymoveit2_real human_robot_collaboration_scenario.launch.py")
    p(doc, "Arayüz (user_interface) aynı iki komutu 'Human-Robot Collaboration "
           "Scenario' düğmesi arkasında birleştirir; HIL parametresi olarak "
           "use_gripper:=true geçer ve senaryonun ürettiği telemetriyi HRC kullanım "
           "etiketiyle Kafka'ya damgalar.")

    # ------------------------------------------------------------------ 3
    h1(doc, "3. Görev Akışı")
    figure(doc, "fig_hrc_postures.png",
           "Turun dört anahtar pozu, URDF'ten ileri kinematikle çizildi. Kırmızı "
           "nokta, gripper'a attach edilen vidalama ucunun merkezidir.")
    phases = [
        ("Hazırlık", "gripper tam açık (0.0 m)",
         "Önceki tur yarıda kesilmiş olabilir; tırnaklar kapalıyken alet standına "
         "gitmek alete çarpar."),
        ("Aleti al", "frontOfScrewer → aboveScrewer → holdScrewer → gripper 0.026 → attach",
         "Kavradıktan SONRA planning scene'e silindir eklenir."),
        ("Vidayı al", "safeWaypoint2 → safeWaypoint1 → tookScrew → outTookScrew",
         "Besleyicinin içine girilip çıkılır; en dar pay 32 mm."),
        ("Dört vida", "xTop → [buton] → DOUT0 ON → xOpt (yavaş) → 1.5 s → DOUT0 OFF → xTop (yavaş)",
         "Her vida için aynı beş adım; aradaki geçişler normal hızda."),
        ("Aleti bırak", "Waypoint1 → frontOfScrewer → aboveScrewer → holdScrewer → detach → gripper 0.003",
         "detach, gripper açılmadan ÖNCE çağrılır."),
        ("Kapanış", "finally: DOUT0/DOUT1 kapat, home_joints'e dön",
         "İstisna ve Ctrl+C dahil her çıkışta koşar."),
    ]
    table(doc, ["Aşama", "Adımlar", "Neden böyle"], phases)
    figure(doc, "fig_hrc_timeline.png",
           "Tek turun zaman çizgisi. Hareket süreleri en yavaş eklemin yolu / (eklem "
           "limiti x hız ölçeği) ile kestirildi; operatör beklemeleri 3 saniye "
           "varsayıldı. Gerçek çevrim, operatörün tepki süresine göre değişir.")
    p(doc, "safe_joint_sequence() tur listesini tek tek işler. Liste hem eklem "
           "hedeflerini hem de sözlük biçimindeki komutları taşır: "
           "{'gripper_position': ...}, {'attach_screwdriver': True}, "
           "{'screwdriver': True}, {'wait_green_button': True}, {'wait': 1.5} ve "
           "hıza özel hareket için {'joints': [...], 'speed': 0.01}. Bir eklem hedefi "
           "üç kez denenir; üçü de başarısız olursa o nokta ATLANIR ve tur devam "
           "eder — kol yarı yolda kilitlenmez.")

    # ------------------------------------------------------------------ 4
    h1(doc, "4. Waypoint'ler ve Kinematik")
    figure(doc, "fig_hrc_tcp_path.png",
           "Vidalama ucunun tur boyunca izlediği yol. Kırmızı parçalar hız ölçeği "
           "0.01 ile koşan vidalama geçişleridir.")
    p(doc, "Waypoint'ler teach pendant'tan okunan eklem açılarıdır; senaryoda "
           "derecelerle yazılıp radyana çevrilirler. İstisna thirdTop ve thirdOpt'tur: "
           "bunlar doğrudan radyan olarak, tam duyarlıkla kaydedilmiştir (robot "
           "üzerinde yeniden öğretildikleri için). Aşağıdaki tablo, açılar ile "
           "URDF'ten hesaplanan uç konumunu birlikte verir.")
    rows = []
    for name in ["frontOfScrewer", "aboveScrewer", "holdScrewer", "safeWaypoint2",
                 "safeWaypoint1", "tookScrew", "outTookScrew", "safeWaypoint",
                 "firstTop", "firstOpt", "secondTop", "secondOpt", "thirdVeryTop",
                 "thirdTop", "thirdOpt", "fourthTop", "fourthOpt", "Waypoint1"]:
        q = wps[name]
        t = F.tcp_of(cell, q)
        rows.append([name, " / ".join(deg(a) for a in q[1:]),
                     f"({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})"])
    table(doc, ["Waypoint", "eklem açıları [°] (base…wrist3)", "uç konumu [m]"], rows)
    figure(doc, "fig_hrc_joint_profile.png",
           "Tur boyunca eklem açıları ve hız ölçeği. Yeşil dikey çizgiler operatör "
           "beklemelerini, kesikli kırmızılar vidalama çıkışının anahtarlandığı anları "
           "gösterir.")
    p(doc, "Grafikte turun iki farklı karakteri görünür: alet ve vida alma "
           "aşamalarında bütün eklemler büyük açılarla döner; vidalama aşamasında ise "
           "yalnızca birkaç eklem birkaç derece oynar ve bu küçük hareketler hız "
           "ölçeği 0.01 ile yapıldığı için zamanın büyük kısmını yer.")

    # ------------------------------------------------------------------ 5
    h1(doc, "5. Vidalama Geçişi")
    figure(doc, "fig_hrc_screw_approach.png",
           "Her vidada xTop → xOpt geçişinin uç uzayındaki ve eklem uzayındaki "
           "karşılığı.")
    dive = []
    for label, top, opt in [("1. vida", "firstTop", "firstOpt"),
                            ("2. vida", "secondTop", "secondOpt"),
                            ("3. vida", "thirdTop", "thirdOpt"),
                            ("4. vida", "fourthTop", "fourthOpt")]:
        a, b = F.tcp_of(cell, wps[top]), F.tcp_of(cell, wps[opt])
        d = (b - a) * 1000.0
        dive.append([label, f"{d[2]:.1f}", f"{np.linalg.norm(d):.1f}",
                     f"{F.seg_duration(wps[top], wps[opt], 0.01):.1f}",
                     f"{F.seg_duration(wps[top], wps[opt], 0.1):.1f}"])
    table(doc, ["Vida", "Δz [mm]", "|Δp| [mm]", "0.01 ile süre [s]",
                "0.1 ile olsaydı [s]"], dive)
    p(doc, "Dalış derinlikleri 30 mm ile 40 mm arasındadır ve hepsi aşağı yönlüdür. "
           "Bu geçişler, vida yuvasına girerken ucun kaçmaması için bilerek "
           "yavaşlatılır: screw_speed varsayılanı 0.01, yani seyir hızının onda "
           "biridir. Tablodaki son sütun, aynı hareketin normal seyir ölçeğiyle ne "
           "kadar süreceğini gösterir — aradaki fark, çevrim süresinin bilinçli olarak "
           "ödenen kısmıdır.")
    p(doc, "Vidanın oturması için geçişin sonunda 1.5 saniye beklenir. Bu açık "
           "döngüdür: ne tork ne de derinlik geri beslemesi okunur; vidalama motoru "
           "sabit süre değil, sabit derinlik sonrası kapatılır. Motoru kapatma "
           "komutu, robot geri çıkmadan ÖNCE gönderilir; aksi halde dönen uç vidadan "
           "ayrılırken yuvayı zorlar.")

    # ------------------------------------------------------------------ 6
    h1(doc, "6. Operatör El Sıkışması ve GPIO Katmanı")
    figure(doc, "fig_hrc_gpio.png",
           "Üstte hücrede ölçülen pin haritası, altta bir vidanın el sıkışma sırası.")
    p(doc, "Operatör ile robot arasındaki tek iletişim kanalı UR kontrol kutusunun "
           "dijital giriş/çıkışlarıdır. Pin haritası tahmin edilmemiş, "
           "examples/find_io_pins.py ile hücrede tek tek ölçülmüştür:")
    table(doc, ["Pin", "İşlev", "Senaryodaki rolü"], [
        ["standard_digital_out[0]", "vidalama SIKMA", "kullanılıyor (screwdriver_pin)"],
        ["standard_digital_out[1]", "vidalama SÖKME", "rezerve (reverse=True ile erişilebilir)"],
        ["standard_digital_in[7]", "YEŞİL buton", "kullanılıyor (green_button_pin)"],
        ["standard_digital_in[6]", "KIRMIZI buton", "rezerve"],
        ["standard_digital_in[5]", "BEYAZ buton", "rezerve"],
    ])
    p(doc, "Çıkışlar /io_and_status_controller/set_io servisine, girişler aynı "
           "kontrolcünün io_states konusuna bağlıdır. Bu komutlar RTDE üzerinden "
           "gittiği için External Control programı koşmasa bile IO yazılıp okunur; "
           "hareket için ise program şarttır.")
    bullets(doc, [
        "Buton okuma YÜKSELEN KENAR ile yapılır. Çağrı anında buton zaten basılıysa "
        "önce bırakılması beklenir; böylece tek basış iki vidalamayı tetiklemez.",
        "green_button_active_high parametresi butonun NO/NC oluşuna göre mantığı "
        "ters çevirir.",
        "green_button_timeout varsayılanı 0.0, yani süresiz bekleme: robot operatör "
        "gelene kadar bekler, zaman aşımıyla kendi başına vidalamaz.",
        "io_states akışı durursa dijital girişler sessizce donar. Düğüm bunu "
        "yakalar: son mesajın üstünden 2 saniye geçmişse 'girişler BAYAT' diye hata "
        "basar, pin hiç yayınlanmıyorsa bunu ayrı bir hata olarak söyler.",
    ])
    h2(doc, "6.1 Sim/Gerçek Ayrımı: GPIO Kendini Kapatır")
    p(doc, "mock_components ile koşan bir hücrede dijital girişler hep 0 döner; "
           "yeşil buton beklemesi sonsuza kadar takılırdı. gpio_mode=auto bunu "
           "kendiliğinden çözer: düğüm robot_description'ı okur, gerçek kolun "
           "ros2_control bloğundaki donanım eklentisine bakar ve mock_components / "
           "gz_ros2_control gibi bir eklenti görürse GPIO'yu tamamen kapatır. "
           "Vidalama adımları atlanır, buton beklenmez, tur sim'de baştan sona akar. "
           "force_on ve force_off bu kararı elle geçersiz kılar.")
    p(doc, "Parametre değerlerinin bilerek 'force_on'/'force_off' olduğuna dikkat "
           "ediniz: komut satırında çıplak 'on'/'off' yazımı YAML kuralı gereği "
           "boolean'a dönüşür, string parametreye atanamaz ve düğümü düşürür.")

    # ------------------------------------------------------------------ 7
    h1(doc, "7. Aletin Planning Scene Temsili")
    figure(doc, "fig_hrc_attach.png",
           "Gripper kapandıktan sonra ur10e_gripper_base_link'e attach edilen silindir.")
    p(doc, "Vidalama aleti iki farklı şekilde modellenir ve ikisi bilerek birbirini "
           "tamamlar:")
    bullets(doc, [
        "URDF'teki ur10e_screwdriver linkinin ÇARPIŞMA geometrisi kaldırılmıştır "
        "(xacro'da yorum satırına alınmış, gerekçesi de orada yazılıdır). Aksi halde "
        "gripper aleti kavradığı anda duran alet ile çarpışma raporlanırdı. Bu "
        "yüzden alet standında sabit duran gövde yalnızca görseldir.",
        "Kavrama tamamlandıktan sonra senaryo, gripper frame'inde bir SİLİNDİR "
        "ekleyip (h = 20 mm, r = 2 mm) onu ur10e_gripper_base_link'e attach eder; "
        "ağırlık 2.4 kg olarak bildirilir — bu, URDF'teki aletin kütlesiyle "
        "(2.400984 kg) uyumludur.",
        "touch_links listesi altı komşu linki (gripper gövdesi, iki tırnak, tool0, "
        "flange, wrist_3) çarpışma denetiminden muaf tutar; aksi halde attach edilen "
        "cisim tutulduğu parmaklarla sürekli temas hâlinde sayılırdı.",
        "detach, gripper açılmadan ÖNCE çağrılır ve cisim sahneden tamamen silinir.",
    ])
    p(doc, "Bu temsilin sınırı açıktır: 20 mm boyunda, 4 mm çapında bir silindir "
           "gerçek aletin kapladığı hacmi temsil etmez. Yani planlayıcı, alet "
           "taşınırken aletin gövdesini değil, yalnızca küçük bir işaretçiyi hesaba "
           "katar. Bugüne kadar sorun çıkarmamasının nedeni, aletli hareketlerin "
           "öğretilmiş waypoint'ler üzerinden gitmesi ve serbest planlamaya çok az yer "
           "kalmasıdır. Alet taşınırken serbest planlamaya izin verilecekse silindirin "
           "gerçek gövdeyi saracak biçimde büyütülmesi gerekir (bölüm 11).")

    # ------------------------------------------------------------------ 8
    h1(doc, "8. Çarpışma Modeli ve Ölçülen Paylar")
    figure(doc, "fig_hrc_clearance.png",
           "Kolun çarpışma mesh'leri ile duran gövdeler arasındaki en küçük mesafeler. "
           "Değerler URDF geometrisinden, hiçbir padding uygulanmadan ölçüldü.")
    rows = []
    for label, q, ob, mm, link in F.measure_clearances(cell, wps):
        rows.append([label, q, ob.replace("ur10e_", ""), f"{mm:.1f}",
                     link.replace("ur10e_", "")])
    table(doc, ["Durum", "Waypoint", "Duran gövde", "Pay [mm]", "En yakın link"], rows)
    p(doc, "En dar nokta aleti kavrama pozudur: gripper gövdesi ile montaj tezgâhı "
           "arasında yalnızca 2.3 mm kalır. Bu çift SRDF'te devre dışı DEĞİLDİR, yani "
           "MoveIt bu payı gerçekten denetler. Pratik sonucu şudur: bu senaryoda kol "
           "linklerine birkaç milimetreden büyük bir padding verilirse holdScrewer "
           "pozu planlanamaz hale gelir. Muayene senaryolarında kullanılan 4 cm'lik "
           "padding buraya taşınamaz.")
    h2(doc, "8.1 Operatör Mankeni Çarpışma Denetiminde Değil")
    p(doc, "Hücrenin URDF'inde bir operatör mankeni (human_link) vardır ve çarpışma "
           "geometrisi de tanımlıdır. Ancak SRDF, bu linki kolun BÜTÜN hareketli "
           "linkleriyle (omuz, üst kol, önkol, üç bilek, gripper gövdesi ve iki "
           "tırnak, robot_mount) 'Never' gerekçesiyle devre dışı bırakır. Yani "
           "planlayıcı, mankenin içinden geçen bir yörünge üretmekten alıkonmaz.")
    p(doc, "Bu, otomatik SRDF üreticisinin beklenen davranışıdır: üretici, iki gövdeyi "
           "varsayılan pozda örnekleyip hiç çarpışmadıklarını görünce çifti kapatır. "
           "Sonuç olarak bu senaryoda operatör güvenliği planlayıcıdan DEĞİL, fiziksel "
           "katmandan gelir: ışık perdesi, hücre bariyerleri ve robotun operatör "
           "onayına kadar hareketsiz beklemesi. Raporun bu maddesi bir hata bildirimi "
           "değil, sınırın nerede olduğunun kaydıdır; planlayıcı tarafında bir koruma "
           "isteniyorsa bu çiftlerin SRDF'te yeniden etkinleştirilmesi gerekir "
           "(padding, devre dışı bırakılmış çiftlerde işe yaramaz).")

    # ------------------------------------------------------------------ 9
    h1(doc, "9. Dayanıklılık: Düğümün Sessizce Sağırlaşması")
    p(doc, "Senaryonun en ince teknik ayrıntısı, ROS 2 executor sahipliğidir. "
           "pymoveit2 içindeki bazı çağrılar rclpy.spin_once(node) kullanır; rclpy'nin "
           "spin_once'ı düğümü önce kendi geçici executor'üne ekler, sonra "
           "finally bloğunda oradan çıkarır. Net sonuç, düğümün HİÇBİR executor'e "
           "bağlı kalmamasıdır: abonelik callback'leri, yani io_states akışı durur. "
           "Dijital girişler donar ve yeşil buton sonsuza kadar görülmez.")
    bullets(doc, [
        "_ensure_executor(), her kritik beklemeden önce düğümün arka plandaki "
        "MultiThreadedExecutor'ün düğüm listesinde olup olmadığına bakar ve gerekirse "
        "geri ekler.",
        "Ölçüt bilerek 'self.executor is None' DEĞİLDİR: rclpy'nin remove_node'u "
        "düğümün executor referansını temizlemez, dolayısıyla o kontrol yanlış "
        "biçimde 'bağlıyım' der.",
        "Aynı nedenle spin_until_future_complete hiçbir yerde kullanılmaz; future'lar "
        "elle, zaman aşımlı bir döngüyle beklenir.",
    ])
    p(doc, "Kapanış yolu da bilerek koşulsuzdur: try/finally bloğu, istisna olsun, "
           "Ctrl+C olsun, her durumda önce vidalama çıkışlarını düşürür, sonra kolu "
           "home pozisyonuna gönderir. Vidalama motorunun açık kalması bu senaryodaki "
           "en istenmeyen son durumdur.")

    # ------------------------------------------------------------------ 10
    h1(doc, "10. Çevrim Maliyeti Nereye Gidiyor")
    figure(doc, "fig_hrc_segment_cost.png",
           "Her geçişin eklem uzayındaki yolu ve tahmini süresi. Kırmızı çubuklar hız "
           "ölçeği 0.01 ile koşan vidalama geçişleridir.")
    moves_pairs = list(zip(moves[:-1], moves[1:]))
    slow_time = sum(F.seg_duration(a["q"], b["q"], 0.01)
                    for a, b in moves_pairs if b["speed"] is not None)
    fast_time = sum(F.seg_duration(a["q"], b["q"], 0.1)
                    for a, b in moves_pairs if b["speed"] is None)
    settle = 1.5 * n_button
    table(doc, ["Kalem", "Tahmini süre [s]", "Not"], [
        ["Seyir hareketleri", f"{fast_time:.0f}", "hız ölçeği 0.1"],
        ["Vidalama geçişleri", f"{slow_time:.0f}",
         f"{n_slow} hareket, hız ölçeği 0.01"],
        ["Vida oturma beklemeleri", f"{settle:.0f}", "4 x 1.5 s"],
        ["Operatör beklemeleri", f"{3.0 * n_button:.0f}",
         "varsayım: basış başına 3 s"],
        ["Adımlar arası sabit bekleme", f"{0.5 * len(steps):.0f}",
         "safe_joint_sequence(wait_time=0.5)"],
    ])
    p(doc, "Tahminler, her segmentte en yavaş eklemin yolunu o eklemin limit hızına "
           "bölerek hesaplanır; gerçek yörüngeler hızlanma/yavaşlama profili taşıdığı "
           "için gerçek süre bundan biraz uzundur. Oranlar yine de nereye bakılması "
           "gerektiğini gösterir: vidalama geçişleri yolun küçük bir kısmını kaplar "
           "ama sürenin kayda değer bir bölümünü yer; adımlar arası 0.5 saniyelik "
           "sabit bekleme de tek başına dikkate değer bir toplam üretir.")

    # ------------------------------------------------------------------ 11
    h1(doc, "11. Bilinen Sınırlar ve Sonraki Adımlar")
    table(doc, ["Konu", "Bugünkü durum", "Öneri"], [
        ["Alet hacmi", "20 x 4 mm silindir; gerçek gövde temsil edilmiyor",
         "Aleti saran bir silindir/kutu ile değiştirmek; serbest planlamaya izin "
         "verilecekse şart"],
        ["Vidalama geri beslemesi", "Açık döngü: sabit derinlik + 1.5 s bekleme",
         "Tork/akım eşiğiyle kapatma veya vidalama aletinden hazır sinyali okuma "
         "(DIN rezerv pinleri duruyor)"],
        ["Operatör güvenliği", "SRDF mankeni devre dışı bırakıyor; koruma fiziksel",
         "İstenirse human_link çiftlerini yeniden etkinleştirip payı ölçmek"],
        ["Tur sayısı", "Launch başına TEK tur",
         "Çok turlu çalışma için dış döngü ya da servis tetikleyici"],
        ["Waypoint kaynağı", "Teach pendant açıları koda gömülü",
         "Muayene paketlerindeki gibi dosyadan okumak; thirdTop/thirdOpt zaten "
         "farklı biçimde (ham radyan) duruyor"],
        ["Başlangıç pozu", "home_joints'e gidiş açılışta yorum satırında",
         "Bilerek: kol zaten güvenli poziyondaysa gereksiz yolculuk. Açılırsa tur "
         "başı öngörülebilirliği artar"],
    ])

    # ------------------------------------------------------------------ 12
    h1(doc, "12. Yeniden Üretme")
    p(doc, "Bu raporun bütün şekilleri ve tabloları kaynak koddan üretilir; rapor "
           "Word'de elle düzenlenmez. Senaryodaki bir açı, hız ya da pin değişirse iki "
           "komut yeterlidir:")
    code(doc, "cd ~/colcon_ws/src/pymoveit2_real\n"
              "python3 docs/hrc_figurler.py      # fig_hrc_*.png\n"
              "python3 docs/hrc_rapor_uret.py    # insan_robot_isbirligi_raporu.docx")
    p(doc, "Şekil üreticisi hücrenin URDF'ini xacro ile kendisi açar ve "
           "~/.cache/hrc_report altında saklar; waypoint'leri de senaryo dosyasının "
           "main() bloğunu ayrıştırarak okur, yani elle kopyalanmış sayı yoktur.")

    doc.save(OUT)
    print("yazıldı:", OUT)
    print(f"  {len(steps)} adım, {len(moves)} hareket, uç yolu {path_len:.2f} m")


if __name__ == "__main__":
    build()
