#!/usr/bin/env python3
"""Tek-kol UR10e şasi muayenesi teknik raporunu (.docx) üretir.

    python3 docs/figur_uret.py       # önce figürler (fig_vp_*.png)
    python3 docs/rapor_uret.py       # sonra rapor

Rapor ELLE düzenlenmez. Sayıların çoğu üretim anında plan dosyasından,
yapılandırmadan ve octomap çıktılarından YENİDEN okunur; böylece plan değişince
rapor da değişir. Kaynaklar:

    plans/viewpoint_plan.json
    config/sick_tmini_params.yaml
    launch/inspection_execution.launch.py
    ~/colcon_ws/src/pcds/single_ur10e/{real,sim}_data/*.ot
"""
import json
import os
import sys

import numpy as np
from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Inches, Pt, RGBColor

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import figur_uret as F  # noqa: E402

OUT = os.path.join(HERE, "viewpoint_planner_teknik_rapor.docx")
FIG_WIDTH = Inches(5.9)


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


def build():
    plan = json.load(open(F.PLAN))
    vps = plan["ur_viewpoints"]
    Q = [v["joint_positions"] for v in vps]
    hops = [F.joint_cost(Q[i], Q[i + 1]) for i in range(len(Q) - 1)]
    P = np.array([v["position"] for v in vps])
    real = F.coverage(
        os.path.join(F.SINGLE, "real_data", "beliefMap_single_ur10e_real.ot"),
        os.path.join(F.SINGLE, "real_data", "occupancyMap_single_ur10e_real.ot"))
    sim = F.coverage(
        os.path.join(F.SINGLE, "sim_data", "beliefMap_single_ur10e_sim.ot"),
        os.path.join(F.SINGLE, "sim_data", "occupancyMap_single_ur10e_sim.ot"))
    multi = F.coverage(os.path.join(F.PCDS, "real_pcds", "beliefMap_real.ot"),
                       os.path.join(F.PCDS, "real_pcds", "occupancyMap_real.ot"))

    doc = Document()
    doc.add_heading("Tek-Kol UR10e ile Şasi Muayenesi", 0)
    sub = doc.add_paragraph("viewpoint_planner paketi — Bakış-Noktası Planlama, "
                            "Sıralama ve Gerçek Robot Yürütmesi — Teknik Rapor")
    sub.style = doc.styles["Subtitle"]
    meta = doc.add_paragraph()
    meta.add_run("Revizyon 2 — 12 Eylül 2026    |    Hazırlayan: Cem Süha Yılmaz"
                 "    |    ROS 2 Humble / MoveIt 2").bold = True
    p(doc, "Revizyon 1 (17 Ağustos 2026) paketin ilk halini figürsüz olarak "
           "belgeliyordu. Bu revizyon, Eylül 2026'da yapılan büyük çalışmayı ekler: "
           "paket, iki kollu multirobot_viewpoint_planner ile aynı mühendislik "
           "seviyesine çekildi (ortak yürütücü temeli, trajectory cache, sahne "
           "eşleme, eklem-uzayı sıralaması) ve senaryo GERÇEK ROBOTLA baştan sona "
           "koşturuldu. Rapordaki sayılar plan dosyasından, yapılandırmadan ve "
           "koşunun octomap çıktısından üretim anında yeniden okunur.")

    # ------------------------------------------------------------------ 1
    h1(doc, "1. Yönetici Özeti")
    table(doc, ["Ölçü", "Değer"], [
        ["Kol", "UR10e, 2 m Festo lineer rayı üzerinde (MoveIt grubu real_ur10e)"],
        ["Sensör", "SICK TIM/Visionary ToF (ur10e_sick_optical_frame)"],
        ["Plandaki bakış-noktası", f"{len(vps)}"],
        ["Planlayıcı tahmini kaplama", f"%{100 * plan['coverage_achieved']:.1f}"],
        ["Gerçek robot octomap kaplaması", f"%{100 * real['frac']:.1f} "
         f"({len(real['covered'])}/{len(real['belief'])} voksel)"],
        ["Simülasyon octomap kaplaması", f"%{100 * sim['frac']:.1f}"],
        ["Turun toplam eklem yolu", f"{np.degrees(sum(hops)):.0f}° eşdeğer "
                                    f"(ray 1 m = 2 rad sayılarak)"],
        ["Bakış-noktası yükseklik aralığı", f"{P[:, 2].min():.2f} … {P[:, 2].max():.2f} m"],
        ["Atlanan bakış-noktası", ", ".join(plan.get("skipped_viewpoints") or ["yok"])],
    ])
    p(doc, "Tek cümlelik sonuç: paket, şasinin UR10e'nin ulaşabildiği yüzeyini "
           "gerçek donanımda %%%.1f oranında kaplayan, tekrarlanabilir "
           "(kayıt-ve-oynat) bir muayene turu üretiyor. Aynı şasi iki kolla "
           "koşturulduğunda kaplama %%%.1f'e çıkıyor; aradaki %.1f puan, ikinci kolun "
           "ölçülen katkısıdır (bölüm 8)." %
           (100 * real["frac"], 100 * multi["frac"],
            100 * (multi["frac"] - real["frac"])))

    # ------------------------------------------------------------------ 2
    h1(doc, "2. Planlama Hattı")
    p(doc, "Hat dört aşamalıdır ve her aşama bir sonrakinin girdisini üretir:")
    table(doc, ["Aşama", "Ne yapar"], [
        ["Hedef örnekleme",
         "Şasi mesh'i yüzey noktalarına örneklenir; her noktanın normali saklanır."],
        ["Aday üretimi",
         "Her hedef nokta için normali boyunca farklı mesafe ve eğimlerde kamera "
         "pozları önerilir (viewpoint_distances, tilt_variations)."],
        ["Görünürlük",
         "Bir adayın hangi hedefleri gerçekten gördüğü katı bir sensör modeliyle "
         "hesaplanır: FOV, menzil, geliş açısı ve occlusion."],
        ["Küme kapsama (set cover)",
         "Greedy seçim: her adımda en çok YENİ nokta getiren aday alınır; marjinal "
         "kazanç tabanın altına düşünce durulur."],
    ])
    figure(doc, "fig_vp_gain.png",
           "Şekil 1: Azalan getiri eğrisi. Çubuklar her bakış-noktasının getirdiği "
           "yeni nokta sayısı, kırmızı çizgi birikimli kaplama. Bakış-noktası "
           "sayısını elle seçilen bir kap değil, marjinal kazanç tabanı belirler.")
    table(doc, ["Parametre", "Değer", "Neden"], [
        ["min_marginal_coverage", "0.0035",
         "durma ölçütü: kazanç bu kesrin altına düşünce greedy biter"],
        ["max_incidence_angle_deg", "85",
         "daha eğik yüzeyler de görülmüş sayılır; kaplamayı açar"],
        ["max_viewpoints", "0 (kapalı)",
         "sert kap devre dışı; sayı azalan getiri eğrisinden gelir"],
        ["collision_padding", "0.04 m",
         "çok-robot hücresiyle AYNI; planlayıcı ile yürütücü aynı payı kullanmalı"],
        ["chassis_collision_padding", "0.0",
         "şasi de aynı padding'i alır (ayrı marj yok)"],
        ["order_mode", "joint", "sıralama eklem uzayında yapılır (bölüm 4)"],
    ])
    figure(doc, "fig_vp_plan.png",
           f"Şekil 2: Güncel plan — {len(vps)} bakış-noktası dört görünümde. Oklar "
           "görüş ekseni, renk o pozun getirdiği yeni nokta sayısı, gri voksel "
           "bulutu şasinin kendisidir. Kol rayın bir tarafında kaldığı için "
           "bakış-noktaları şasinin tek yüzünde toplanır; karşı yüz bu senaryonun "
           "erişemediği bölgedir.")

    # ------------------------------------------------------------------ 3
    h1(doc, "3. Planlayıcı ile Yürütücünün Aynı Sahneyi Görmesi")
    p(doc, "Planlayıcı bir bakış-noktasını 'ulaşılabilir' sayarken /compute_ik'e "
           "sorar. Yürütücü ise koşarken sahneye bir zemin düzlemi ekler ve bütün "
           "ur10e_* gövdelerine padding uygular. İki sahne ayrıldığında planlayıcının "
           "kabul ettiği poz, yürütücüde çarpışma yüzünden reddedilir — özellikle "
           "zemine yakın pozlarda.")
    bullets(doc, [
        "Planlayıcı artık IK'dan ÖNCE yürütücünün sahnesini kurar (ik_scene_setup): "
        "zemin düzlemi + aynı padding kuralları.",
        f"Kurulan sahne plana yazılır. Güncel planda: zemin z = "
        f"{plan['ik_scene']['ground_plane_z']} m, padding "
        f"{plan['ik_scene']['collision_padding']} m, padding uygulanan link sayısı "
        f"{plan['ik_scene']['padded_links']} (bunların "
        f"{plan['ik_scene']['padded_chassis_links']} tanesi şasi parçası).",
        "Bu sahne kurulumu, planlayıcı düğümünün KENDİ servis callback'i içinden "
        "yapılamaz: rclpy'nin spin_until_future_complete'i kendi düğümünde "
        "çağrıldığında future hiç tamamlanmaz ve çağrı sessizce takılır. Bu yüzden "
        "sahne istemcisi AYRI bir düğüm olarak kurulur. Aynı hata, sahne eşlemesini "
        "uzun süre sessizce devre dışı bırakmıştı.",
    ])

    # ------------------------------------------------------------------ 4
    h1(doc, "4. Tur Sıralaması — Eklem Uzayında")
    p(doc, "Bakış-noktalarının hangi sırayla gezileceği, kaplamayı değiştirmez ama "
           "tur süresini ve — daha önemlisi — turun tamamlanıp tamamlanmayacağını "
           "belirler. Kartezyen yakınlığa göre sıralamak yanıltıcıdır: uzayda yan "
           "yana duran iki poz, kolun bambaşka bir konfigürasyona geçmesini "
           "gerektirebilir.")
    bullets(doc, [
        "Sıralama EKLEM UZAYINDA yapılır. Maliyet, eklem yollarının toplamıdır; ray "
        "metresi order_rail_weight = 2.0 rad karşılığı sayılır.",
        "Her bakış-noktası için tek bir IK çözümü değil, birkaç DAL toplanır "
        "(order_ik_seeds = 10 rastgele tohum). Zincir kurulurken her durak için o "
        "durağın en ucuz dalı seçilir.",
        "Zincir FİZİBİLİTE DOĞRULAMALIDIR: bir sonraki durak seçilirken gerçek bir "
        "plan denenir (order_plan_time = 3 s, order_max_tries_per_step = 8). "
        "Planlanamayan geçiş zincire alınmaz; böylece tur, robotun gerçekten "
        "yapabildiği geçişlerden oluşur.",
        "Doğrulama bütçesi (order_feasibility_budget_s = 240 s) dolarsa kalan "
        "duraklar 'straggler' olarak en ucuz yerlerine yerleştirilir.",
        "Hiçbir şekilde bağlanamayan duraklar plana skipped_viewpoints olarak "
        f"yazılır; güncel planda bunlar: "
        f"{', '.join(plan.get('skipped_viewpoints') or ['yok'])}.",
        "Home'a uğrayarak dolaşma KALDIRILDI: eskiden zor geçişler için kol ara "
        "durak olarak başlangıç pozuna gidiyordu; bu, tur süresinin büyük kısmını "
        "boşa harcıyordu.",
    ])
    figure(doc, "fig_vp_order.png",
           "Şekil 3: Solda turun gezme sırası (numaralar), sağda ardışık duraklar "
           "arasındaki eklem-uzayı maliyeti. Kartezyen uzayda yol kesişmiş görünür; "
           "bu beklenendir, çünkü sıralama Kartezyen mesafeyi değil eklem yolunu ve "
           "geçişin planlanabilirliğini gözetir.")
    top = sorted(range(len(hops)), key=lambda i: -hops[i])[:5]
    table(doc, ["En pahalı beş geçiş", "maliyet [° eşdeğer]"],
          [[f"{vps[i]['id']} → {vps[i + 1]['id']}", f"{np.degrees(hops[i]):.0f}"]
           for i in top])

    # ------------------------------------------------------------------ 5
    h1(doc, "5. Yürütücü")
    p(doc, "Yürütücü düğümü, çok-robot paketiyle aynı desene çekildi: tekrarlanabilir "
           "olan her şey (plan okuma, trajectory cache, 2π açma, padding, sahne "
           "kurulumu, varış denetimi) ortak bir temel sınıftadır; bu dosyada yalnızca "
           "UR'ye özgü olan kalır — hangi MoveIt grubu, hangi kamera konuları, "
           "yörüngenin nasıl gönderildiği.")
    table(doc, ["Ayar", "Varsayılan", "Not"], [
        ["use_trajectory_cache", "true",
         "her viewpoint bir kez planlanır, <plans>/trajectories/ur_<id>.json olarak "
         "kaydedilir ve sonraki koşularda OYNATILIR"],
        ["force_replan", "false",
         "URDF/SRDF/padding değişince cache geçersizdir ama bunu kendisi anlamaz; "
         "geometri değiştiyse true ile yeniden kaydedin"],
        ["execute_via_move_group", "true",
         "yörünge move_group'un execute_trajectory action'ına gider; ham "
         "FollowJointTrajectory yolu rayı sarsıyordu"],
        ["use_pose_goal", "false",
         "plandaki eklem konfigürasyonuna gidilir; sıralama zaten bütün turu o "
         "dallar üzerinden doğruladı"],
        ["nearest_branch_ik", "false",
         "poz başına en yakın dalı seçmek AÇGÖZLÜdür ve planlanmış zincirden "
         "çıkarır; sıralama bu kararı tur genelinde verir"],
        ["ur_velocity / ur_acceleration", "0.1", "MoveIt ölçekleme çarpanları"],
        ["allowed_planning_time / attempts", "10 s / 10",
         "10 paralel deneme, en kısası tutulur"],
    ])
    p(doc, "En yakın dal araması ve en kısa yörüngenin seçilmesi artık senaryo "
           "kodunda değil, pymoveit2_real kütüphanesindedir (nearest_branch_ik, "
           "plan_nearest_branch, trajectory_travel). Trajectory cache bilerek "
           "kütüphaneye taşınmadı: o, muayene senaryosunun kendi sözleşmesidir.")

    # ------------------------------------------------------------------ 6
    h1(doc, "6. Plan Bakımı ve Cache Sözleşmesi")
    p(doc, "Gerçek koşularda birkaç bakış-noktası hücrede yürütülemedi ve plandan "
           "çıkarıldı. Bu düzeltmeler KAYNAK KODA DOKUNMADAN yalnız plan dosyasında "
           "yapıldı ve plana manual_edits bloğu olarak yazıldı:")
    edits = plan.get("manual_edits", {})
    rows = []
    for key, val in edits.items():
        if isinstance(val, dict):
            rows.append([key, ", ".join(val.get("ids", [])), val.get("why", "")])
        else:
            rows.append([key, "-", str(val)])
    table(doc, ["Düzenleme", "Bakış-noktaları", "Gerekçe"], rows)
    bullets(doc, [
        "Bir bakış-noktası çıkarıldığında onun cache dosyası da plandan alınır: "
        "trajectories_removed_<tarih>/ altına taşınır, silinmez.",
        "ÖNEMLİ: ardışık bir duraktan biri çıkarıldığında ARDINDAN GELENİN cache'i de "
        "bayatlar; çünkü kayıtlı yörünge artık var olmayan bir başlangıç pozundan "
        "başlar. Çıkarılan durağın ardılının cache'i de yenilenmelidir.",
        "vp_008, kablo kanalı blokları 5 cm yükseltildikten sonra hedef pozunda "
        "çarpışmaya düşmüştü; yeni bir IK dalı ile yeniden çözüldü ve komşu "
        "geçişleriyle (vp_012 → vp_008 → vp_013) birlikte yeniden planlandı.",
        "Cache, URDF ve padding değişikliklerine KÖRDÜR: kayıtlı yol yeni bir engelin "
        "içinden geçse bile geçerli sayılır. Hücre geometrisi her değiştiğinde ya "
        "force_replan ile yeniden kaydedin ya da ilgili dosyaları elle doğrulayın.",
    ])

    # ------------------------------------------------------------------ 7
    h1(doc, "7. Hücre Geometrisindeki Değişikliklerin Etkisi")
    p(doc, "Eylül 2026'da hücrenin modeline üç ekleme yapıldı ve üçü de bu senaryoyu "
           "doğrudan etkiledi: lineer eksenin motor çıkıntısı (20 x 15 x 14 cm kutu), "
           "kablo kanalını temsil eden iki bloğun 5 cm yükseltilmesi (üst yüz "
           "z = 0.647 m; taşıyıcı braketin alt yüzü z = 0.6524 m) ve bu blokların "
           "hücrenin geri kalanıyla aynı griye boyanması. İlk ikisi gerçek hücrede "
           "var olan ama modelde bulunmayan engellerdi; modele girdikleri anda bazı "
           "kayıtlı yörüngeler ve bir bakış-noktası (vp_008) geçersizleşti. Bu, "
           "yukarıdaki cache sözleşmesinin neden yazıldığının somut örneğidir.")

    # ------------------------------------------------------------------ 8
    h1(doc, "8. Ölçülen Sonuç — Octomap")
    p(doc, "Koşu sırasında her durakta hem gerçek SICK bulutu hem simülasyon bulutu "
           "kaydedilir; bunlar dünya çerçevesinde birleştirilip octomap'e çevrilir. "
           "Aşağıdaki kaplama şöyle tanımlanmıştır: belief haritasındaki her şasi "
           "vokseli 2 cm çözünürlüğe açılır (budanmış 4 cm yapraklar 8 alt voksele "
           "bölünür), occupancy haritasının DOLU vokselleri için aynısı yapılır ve "
           "kaplama = kesişim / şasi voksel sayısıdır.")
    table(doc, ["Koşu", "Şasi vokseli", "Kaplanan", "Kaplama"], [
        ["Tek kol — gerçek robot", len(real["belief"]), len(real["covered"]),
         f"%{100 * real['frac']:.1f}"],
        ["Tek kol — simülasyon", len(sim["belief"]), len(sim["covered"]),
         f"%{100 * sim['frac']:.1f}"],
        ["İki kol — gerçek robot (karşılaştırma)", len(multi["belief"]),
         len(multi["covered"]), f"%{100 * multi['frac']:.1f}"],
    ])
    figure(doc, "fig_vp_octomap.png",
           "Şekil 4: Tek-kol koşusunun octomap'i. Yeşil vokseller sensörle kaplanan "
           "şasi yüzeyi, kırmızılar kaplanmayan. Gerçek ve sim aynı kamera "
           "açılarıyla çizilmiştir.")
    figure(doc, "fig_vp_part_coverage.png",
           "Şekil 5: Parça başına kaplama. Eksik, bütün yüzeye yayılmış bir bozulma "
           "değil; birkaç parçada toplanır.")
    figure(doc, "fig_vp_single_vs_multi.png",
           "Şekil 6: Aynı şasi, aynı ölçüt — solda tek kol, sağda iki kol. İkinci "
           "kolun kapattığı kırmızı bölgeler, çok-robot senaryosunun varlık sebebidir.")
    p(doc, "Planlayıcının kaplama tahmini (%%%.1f) ile octomap kaplaması (%%%.1f) "
           "AYNI ŞEYİ ÖLÇMEZ ve eşleşmeleri beklenmez. Planlayıcı, katı bir sensör "
           "modeli altında görülebilir mesh örnek noktalarının kesridir ve muhafazakâr "
           "bir tahmindir; octomap ise gerçekte isabet alan voksellerin oranıdır "
           "(farklı payda, voksel başına 'herhangi bir isabet')." %
           (100 * plan["coverage_achieved"], 100 * real["frac"]))

    # ------------------------------------------------------------------ 9
    h1(doc, "9. Bilinen Sınırlar ve Sonraki Adımlar")
    table(doc, ["Konu", "Durum", "Öneri"], [
        ["Şasinin karşı yüzü", "Bu senaryo için erişilemez (ray tek tarafta)",
         "İki kollu senaryo; ölçülen kazanç bölüm 8'de"],
        ["Atlanan bakış-noktaları",
         ", ".join(plan.get("skipped_viewpoints") or ["yok"]) + " zincire bağlanamadı",
         "Farklı IK dalı ile yeniden denemek veya pozu biraz kaydırmak"],
        ["Cache geçerliliği", "URDF/padding değişikliğine kör",
         "Plan dosyasındaki ik_scene damgasını cache ile karşılaştıran bir denetim"],
        ["Alt raylar ve ayaklar", "Gerçek robotta en düşük kaplama orada",
         "Bu bölge için özel bakış-noktası üretimi; padding'i bölgeye göre gevşetmek"],
        ["Plan-zamanı kaplama tahmini", "Elle düzenlemelerden sonra yaklaşık",
         "Düzenlemeden sonra kaplamayı yeniden hesaplayan küçük bir araç"],
    ])

    # ------------------------------------------------------------------ 10
    h1(doc, "10. Çalıştırma ve Yeniden Üretme")
    code(doc,
         "# plan üretimi (planlayıcı düğümü)\n"
         "ros2 launch viewpoint_planner viewpoint_planning.launch.py\n\n"
         "# muayene turu (gerçek robot; HIL ayakta olmalı)\n"
         "ros2 launch viewpoint_planner inspection_execution.launch.py only_sim:=false\n\n"
         "# kayıtlı yörüngeleri yok sayıp yeniden kaydet\n"
         "ros2 launch viewpoint_planner inspection_execution.launch.py force_replan:=true")
    p(doc, "Arayüzden 'UR10e Inspection Scenario' düğmesi aynı komutu kurar; "
           "use_fake_hardware kapalıyken only_sim:=false olarak geçer, yani gerçek "
           "SICK bulutları da kaydedilir.")
    code(doc, "cd ~/colcon_ws/src/viewpoint_planner\n"
              "python3 docs/figur_uret.py     # fig_vp_*.png\n"
              "python3 docs/rapor_uret.py     # viewpoint_planner_teknik_rapor.docx")

    doc.save(OUT)
    print("yazıldı:", OUT)
    print(f"  {len(vps)} bakış-noktası, gerçek kaplama %{100 * real['frac']:.1f}")


if __name__ == "__main__":
    build()
