/**
 * build_doors_octomap.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * ROS 2 Humble — CAR DOOR inspection octomap builder.
 *
 * Same two-stage recipe as the chassis builder (pcd2octomap_builder/build_octomap):
 *
 *   AŞAMA 1 — Belief Map
 *     <parts_dir> altındaki .pcd dosyaları (doors_parts_prep'in ürettiği kapı
 *     yamaları) yüklenir.
 *     Her yamaya farklı renk atanır ve ColorOcTree'ye updateNode(false) ile FREE
 *     olarak eklenir: "var olan ve görülmesi gereken yüzey" referansı budur.
 *     → beliefMap_doors_<mode>.ot
 *
 *   AŞAMA 2 — Occupancy Map
 *     İki kolun yakaladığı bulutlar belief map üstüne damgalanır. insertRay
 *     KULLANILMAZ (ışın boyunca free hücre açmak belief map'i bozar); yalnızca
 *     endpoint occupied yapılır ve belief rengi korunur.
 *     → occupancyMap_doors_<mode>.ot
 *
 * Kapı verisi şaseden ÜÇ noktada ayrılır — bu dosyanın var olma sebebi:
 *
 *   1. BİRİM. Şase parçaları CAD'den mm geldiği için chassis builder noktaları
 *      sabit olarak 1000'e böler. Kapı STL'leri zaten METRE (URDF onları
 *      scale="1 1 1" ile yükler). Burada bölme --parts_scale ile parametre, ve
 *      kapılar için varsayılanı 1.0'dır. Şase builder'ını kapılara vermek
 *      kapıları 1.2 mm'lik bir zerreye indirirdi.
 *   2. VERİ KÖKÜ. Kapı yakalamaları ~/colcon_ws/src/pcds/doors/{sim,real}_pcds
 *      altındadır; şase köküne dokunulmaz.
 *   3. RAPOR. Kapılar 2 STL olduğu için parça başına rapor iki satır olurdu.
 *      doors_parts_prep her kapıyı yamalara böler, bu builder da yamaları
 *      "__" öncesindeki isme göre gruplayıp hem yama hem kapı toplamını basar.
 *
 * Kullanım (hepsi opsiyonel; argümansız çalışır):
 *   ros2 run doors_inspection build_doors_octomap \
 *     [--mode sim|real] [--data_dir <pcds/doors/sim_pcds>] \
 *     [--parts_dir <~/.cache/doors_inspection/door_parts>] [--parts_scale 1.0] \
 *     [--belief_out ...] [--occ_out ...] \
 *     [--res 0.02] [--z_min -10.0] [--use_ur 1] [--use_kawasaki 1] [--snap 0]
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Octomap
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

// Eigen
#include <Eigen/Geometry>

namespace fs = std::filesystem;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

// ─────────────────────────────────────────────────────────────────────────────
// Voxel anahtarı — octomap çözünürlüğüne göre tam sayı ızgara
// ─────────────────────────────────────────────────────────────────────────────
struct VoxelKey {
    int x, y, z;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        size_t h = std::hash<int>()(k.x);
        h ^= std::hash<int>()(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>()(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};
using VoxelPartMap = std::unordered_map<VoxelKey, int, VoxelKeyHash>;

static VoxelKey to_voxel_key(const octomap::point3d& pt, double res) {
    return { static_cast<int>(std::floor(pt.x() / res)),
             static_cast<int>(std::floor(pt.y() / res)),
             static_cast<int>(std::floor(pt.z() / res)) };
}

static octomap::point3d voxel_center(const VoxelKey& k, double res) {
    return octomap::point3d(static_cast<float>((k.x + 0.5) * res),
                            static_cast<float>((k.y + 0.5) * res),
                            static_cast<float>((k.z + 0.5) * res));
}

// ─────────────────────────────────────────────────────────────────────────────
// Renk paleti — pcd2octomap_builder ile AYNI sıra, böylece iki işin octovis
// görüntüleri yan yana karşılaştırılabilir kalır.
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t COLOR_PALETTE[][3] = {
    { 85,  26, 139}, {144, 238, 144}, {  0,   0, 139}, {190, 170,   8},
    {150,  10,   0}, {135, 206, 250}, {255, 165,   0}, {255, 182, 193},
    {  0, 100,   0}, {255,  20, 147}, {139,  71,  38}, {  0, 245, 255},
    {238, 149, 114}, {  0, 128, 128}, { 67, 205, 128}, {128,   0,   0},
    {220,  20,  60}, {255, 127,  80}, {205,  92,  92}, {255, 215,   0},
    {184, 134,  11}, {189, 183, 107}, {128, 128,   0}, { 85, 107,  47},
    {127, 255,   0}, { 34, 139,  34}, {143, 188, 143}, {  0, 250, 154},
    {102, 205, 170}, { 70, 130, 180}, { 25,  25, 112}, { 65, 105, 225},
    {138,  43, 226}, { 72,  61, 139}, {106,  90, 205}, {147, 112, 219},
    {139,   0, 139}, {153,  50, 204}, {221, 160, 221}, {218, 112, 214},
    {255,  20, 147}, {255, 182, 193}, {245, 222, 179}, {139,  69,  19},
    {210, 105,  30}, {200, 105,  30}, {195, 105,  30}, {190, 105,  30},
    {185, 105,  30}, {180, 105,  30},
};
static const int PALETTE_SIZE =
    static_cast<int>(sizeof(COLOR_PALETTE) / sizeof(COLOR_PALETTE[0]));

// ─────────────────────────────────────────────────────────────────────────────
// Config
// ─────────────────────────────────────────────────────────────────────────────
struct Config {
    std::string mode = "sim";       // sim | real  — sadece varsayılan yolları seçer
    std::string parts_dir;
    std::string data_dir;           // <data_dir>/{ur_data,kawasaki_data}/{pcds,poses}
    std::string belief_out;
    std::string occ_out;
    double      res         = 0.02;
    float       z_min       = -10.0f;
    // Kapı yamaları METRE cinsinden yazılır; şase builder'ındaki /1000 burada YOK.
    double      parts_scale = 1.0;
    bool        use_ur       = true;
    bool        use_kawasaki = true;
    // 0 = şase davranışının birebir aynısı (nokta tam olarak bir belief voxeline
    // düşmeliidr). 1 = düşmezse 26 komşuya bakılır; gerçek veride kalibrasyon
    // kayması yüzünden yüzeyin bir voxel yanına düşen noktaları kurtarır, ama
    // kaplamayı iyimser gösterir — bu yüzden varsayılan kapalı.
    bool        snap = false;
};

static std::string expand_tilde(const std::string& path) {
    if (!path.empty() && path[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) return std::string(home) + path.substr(1);
    }
    return path;
}

static bool parse_bool(const std::string& v) {
    return !(v == "0" || v == "false" || v == "False" || v == "no");
}

static Config parse_args(int argc, char** argv) {
    Config cfg;
    for (int i = 1; i < argc - 1; ++i) {
        std::string k = argv[i], v = argv[i + 1];
        if      (k == "--mode")         cfg.mode         = v;
        else if (k == "--parts_dir")    cfg.parts_dir    = expand_tilde(v);
        else if (k == "--data_dir")     cfg.data_dir     = expand_tilde(v);
        else if (k == "--belief_out")   cfg.belief_out   = expand_tilde(v);
        else if (k == "--occ_out")      cfg.occ_out      = expand_tilde(v);
        else if (k == "--res")          cfg.res          = std::stod(v);
        else if (k == "--z_min")        cfg.z_min        = std::stof(v);
        else if (k == "--parts_scale")  cfg.parts_scale  = std::stod(v);
        else if (k == "--use_ur")       cfg.use_ur       = parse_bool(v);
        else if (k == "--use_kawasaki") cfg.use_kawasaki = parse_bool(v);
        else if (k == "--snap")         cfg.snap         = parse_bool(v);
    }

    if (cfg.mode != "sim" && cfg.mode != "real") {
        std::cerr << "❌ --mode yalnizca 'sim' veya 'real' olabilir (verilen: "
                  << cfg.mode << ")\n";
        std::exit(1);
    }

    const std::string home = expand_tilde("~");
    if (cfg.parts_dir.empty())
        cfg.parts_dir = home + "/.cache/doors_inspection/door_parts";
    if (cfg.data_dir.empty())
        cfg.data_dir = home + "/colcon_ws/src/pcds/doors/" + cfg.mode + "_pcds";
    if (cfg.belief_out.empty())
        cfg.belief_out = cfg.data_dir + "/beliefMap_doors_" + cfg.mode + ".ot";
    if (cfg.occ_out.empty())
        cfg.occ_out = cfg.data_dir + "/occupancyMap_doors_" + cfg.mode + ".ot";

    if (!cfg.use_ur && !cfg.use_kawasaki) {
        std::cerr << "❌ Hem --use_ur hem --use_kawasaki kapali; islenecek veri yok.\n";
        std::exit(1);
    }
    return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
// Parça PCD'leri. İsim düzeni "<kapi>__y<j>z<j>.pcd"; rapor gruplaması "__"
// öncesindeki kapı adına göre yapılır.
// ─────────────────────────────────────────────────────────────────────────────
static std::string door_of(const fs::path& p) {
    std::string stem = p.stem().string();
    size_t pos = stem.find("__");
    return (pos == std::string::npos) ? stem : stem.substr(0, pos);
}

static std::vector<fs::path> collect_parts(const std::string& dir) {
    std::vector<fs::path> paths;
    if (!fs::is_directory(dir)) return paths;
    for (const auto& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".pcd") paths.push_back(e.path());
    // Alfabetik sıra yama ızgarasını kapı kapı, sütun sütun sıralar.
    std::sort(paths.begin(), paths.end());
    return paths;
}

// ─────────────────────────────────────────────────────────────────────────────
// Tarama (PCD + pose) eşleştirme — iki kolun dosya adları çakışır (ikisinde de
// 1.pcd, 2.pcd ...), bu yüzden stem kol etiketiyle benzersizleştirilir.
// ─────────────────────────────────────────────────────────────────────────────
struct ScanPair {
    fs::path    pcd_path;
    fs::path    pose_path;
    std::string stem;
};

static std::vector<ScanPair> match_scans_for_robot(const std::string& robot_dir,
                                                   const std::string& robot_tag) {
    std::vector<ScanPair> pairs;
    fs::path pcd_dir  = fs::path(robot_dir) / "pcds";
    fs::path pose_dir = fs::path(robot_dir) / "poses";

    if (!fs::is_directory(pcd_dir) || !fs::is_directory(pose_dir)) {
        std::cerr << "⚠️  " << robot_tag << ": klasor yok (" << pcd_dir.string()
                  << ") — atlaniyor\n";
        return pairs;
    }

    std::map<std::string, fs::path> pose_map;
    for (const auto& e : fs::directory_iterator(pose_dir))
        if (e.path().extension() == ".txt")
            pose_map[e.path().stem().string()] = e.path();

    for (const auto& e : fs::directory_iterator(pcd_dir)) {
        if (e.path().extension() != ".pcd") continue;
        std::string stem = e.path().stem().string();
        auto it = pose_map.find(stem);
        if (it != pose_map.end())
            pairs.push_back({e.path(), it->second, robot_tag + ":" + stem});
        else
            std::cerr << "⚠️  " << robot_tag << " pose bulunamadi: " << stem
                      << " — atlaniyor\n";
    }

    std::sort(pairs.begin(), pairs.end(), [](const ScanPair& a, const ScanPair& b) {
        auto num = [](const std::string& s) -> int {
            size_t pos = s.find_last_of(':');
            try { return std::stoi(pos == std::string::npos ? s : s.substr(pos + 1)); }
            catch (...) { return 0; }
        };
        return num(a.stem) < num(b.stem);
    });
    return pairs;
}

static std::vector<ScanPair> collect_scans(const Config& cfg) {
    std::vector<ScanPair> scans;
    if (cfg.use_ur) {
        auto ur = match_scans_for_robot(cfg.data_dir + "/ur_data", "ur");
        std::cout << "  UR taramasi       : " << ur.size() << "\n";
        scans.insert(scans.end(), ur.begin(), ur.end());
    }
    if (cfg.use_kawasaki) {
        auto kw = match_scans_for_robot(cfg.data_dir + "/kawasaki_data", "kawa");
        std::cout << "  Kawasaki taramasi : " << kw.size() << "\n";
        scans.insert(scans.end(), kw.begin(), kw.end());
    }
    return scans;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pose TXT → sensör konumu.  Format: tx ty tz qx qy qz qw
// PCD'ler zaten world frame'inde kaydedilir; pose yalnızca sensorOrigin içindir.
// ─────────────────────────────────────────────────────────────────────────────
struct SensorPose { Eigen::Vector3d t; Eigen::Quaterniond q; };

static SensorPose load_pose(const fs::path& p) {
    std::ifstream f(p);
    if (!f.is_open()) throw std::runtime_error("TXT acilamadi: " + p.string());
    double tx, ty, tz, qx, qy, qz, qw;
    if (!(f >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
        throw std::runtime_error("TXT parse hatasi: " + p.string());
    SensorPose pose;
    pose.t = {tx, ty, tz};
    pose.q = Eigen::Quaterniond(qw, qx, qy, qz);   // Eigen: (w, x, y, z)
    pose.q.normalize();
    return pose;
}

// ─────────────────────────────────────────────────────────────────────────────
// AŞAMA 1 — Belief Map
// ─────────────────────────────────────────────────────────────────────────────
static void build_belief_map(octomap::ColorOcTree* tree,
                             const std::vector<fs::path>& parts,
                             double parts_scale,
                             std::vector<int>& belief_counts,
                             VoxelPartMap& voxel_part_map) {
    std::cout << "\n━━━  AŞAMA 1: Belief Map  ━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    belief_counts.assign(parts.size(), 0);
    const double res = tree->getResolution();

    for (size_t i = 0; i < parts.size(); ++i) {
        PointCloud cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(parts[i].string(), cloud) == -1) {
            std::cerr << "  ⚠️  Yuklenemedi: " << parts[i].filename() << "\n";
            continue;
        }

        const int idx = static_cast<int>(i) % PALETTE_SIZE;
        const uint8_t R = COLOR_PALETTE[idx][0];
        const uint8_t G = COLOR_PALETTE[idx][1];
        const uint8_t B = COLOR_PALETTE[idx][2];

        std::unordered_set<size_t> part_voxels;
        VoxelKeyHash hasher;

        for (const auto& p : cloud.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;
            // parts_scale: kapılar için 1.0 (metre). Şase parçaları mm olduğu için
            // orada 0.001 verilir.
            octomap::point3d pt(static_cast<float>(p.x * parts_scale),
                                static_cast<float>(p.y * parts_scale),
                                static_cast<float>(p.z * parts_scale));
            // false = FREE  →  octovis'te gri; "görülmesi gereken ama henüz
            // görülmemiş yüzey" anlamı buradan gelir.
            if (octomap::ColorOcTreeNode* node = tree->updateNode(pt, false))
                node->setColor(R, G, B);

            VoxelKey vk = to_voxel_key(pt, res);
            voxel_part_map[vk] = static_cast<int>(i);
            part_voxels.insert(hasher(vk));
        }

        belief_counts[i] = std::max<int>(1, static_cast<int>(part_voxels.size()));
        std::cout << "  " << parts[i].filename().string() << "  →  " << cloud.size()
                  << " nokta  " << part_voxels.size() << " voxel\n";
    }

    int total = 0;
    for (int c : belief_counts) total += c;
    std::cout << "  ─────────────────────────────────────────────────\n";
    std::cout << "  Belief map toplam voxel : " << total << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// AŞAMA 2 — Occupancy Map
// ─────────────────────────────────────────────────────────────────────────────
static void build_occupancy_map(octomap::ColorOcTree* tree,
                                const std::vector<ScanPair>& scans,
                                const VoxelPartMap& voxel_part_map,
                                std::vector<int>& occ_counts,
                                float z_min, bool snap) {
    std::cout << "\n━━━  AŞAMA 2: Occupancy Map  ━━━━━━━━━━━━━━━━━━━━━━\n";

    const double res = tree->getResolution();
    long total_ins = 0, total_skip = 0, total_snap = 0;
    std::vector<std::unordered_set<size_t>> occ_voxel_sets(occ_counts.size());
    VoxelKeyHash vk_hasher;

    for (size_t i = 0; i < scans.size(); ++i) {
        const auto& scan = scans[i];
        std::cout << "  [" << (i + 1) << "/" << scans.size() << "]  " << scan.stem << "\n";

        PointCloud::Ptr cs(new PointCloud);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan.pcd_path.string(), *cs) == -1) {
            std::cerr << "    ⚠️  PCD yuklenemedi — atlaniyor\n";
            continue;
        }

        // Pose sadece kaydın bütünlüğünü doğrulamak için okunur: bulut zaten world
        // frame'inde yazılmıştır, transform UYGULANMAZ (şase işiyle aynı sözleşme).
        try { (void)load_pose(scan.pose_path); }
        catch (const std::exception& e) {
            std::cerr << "    ⚠️  " << e.what() << " — atlaniyor\n";
            continue;
        }

        long ins = 0, skip = 0, snapped = 0;
        for (const auto& p : cs->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                { ++skip; continue; }
            if (p.z < z_min) { ++skip; continue; }

            octomap::point3d pt(p.x, p.y, p.z);
            VoxelKey vk = to_voxel_key(pt, res);

            auto vit = voxel_part_map.find(vk);
            if (vit == voxel_part_map.end() && snap) {
                // 26 komşu: gerçek veride el-göz kalibrasyonu yüzeyin bir voxel
                // yanına düşürebilir. Bulunursa nokta o belief voxeline çekilir.
                for (int dx = -1; dx <= 1 && vit == voxel_part_map.end(); ++dx)
                for (int dy = -1; dy <= 1 && vit == voxel_part_map.end(); ++dy)
                for (int dz = -1; dz <= 1 && vit == voxel_part_map.end(); ++dz) {
                    if (!dx && !dy && !dz) continue;
                    auto nit = voxel_part_map.find({vk.x + dx, vk.y + dy, vk.z + dz});
                    if (nit != voxel_part_map.end()) {
                        vit = nit;
                        vk  = {vk.x + dx, vk.y + dy, vk.z + dz};
                        pt  = voxel_center(vk, res);
                        ++snapped;
                    }
                }
            }

            if (vit == voxel_part_map.end()) {
                // Belief map dışı nokta — kapı değil (zemin, şase, AGV ...), atla.
                ++skip;
                continue;
            }

            octomap::ColorOcTreeNode* existing = tree->search(pt);
            octomap::ColorOcTreeNode::Color color =
                existing ? existing->getColor() : octomap::ColorOcTreeNode::Color(255, 255, 255);

            const int pid = vit->second;
            if (occ_voxel_sets[pid].insert(vk_hasher(vk)).second)
                occ_counts[pid]++;

            // insertRay KULLANILMIYOR: origin→endpoint arasını free yapmak belief
            // map'i bozar. Sadece endpoint occupied, renk belief'ten korunur.
            if (octomap::ColorOcTreeNode* node = tree->updateNode(pt, true))
                node->setColor(color.r, color.g, color.b);
            ++ins;
        }

        std::cout << "    " << cs->size() << " nokta  |  " << ins << " kapi ici eklendi  |  "
                  << skip << " kapi disi/NaN atlandi";
        if (snap) std::cout << "  |  " << snapped << " komsu voxele cekildi";
        std::cout << "\n";
        total_ins  += ins;
        total_skip += skip;
        total_snap += snapped;
    }

    std::cout << "  ─────────────────────────────────────────────────\n";
    std::cout << "  Toplam eklenen : " << total_ins  << "\n";
    std::cout << "  Toplam atlanan : " << total_skip << "\n";
    if (snap) std::cout << "  Toplam cekilen : " << total_snap << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Coverage raporu — yama satırları + kapı toplamı + genel toplam
// ─────────────────────────────────────────────────────────────────────────────
static void print_coverage(const std::vector<fs::path>& parts,
                           const std::vector<int>& belief_counts,
                           const std::vector<int>& occ_counts) {
    std::cout << "\n━━━  COVERAGE RAPORU  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

    // parts alfabetik sıralı olduğu için aynı kapının yamaları bitişiktir.
    long g_belief = 0, g_occ = 0;
    size_t i = 0;
    while (i < parts.size()) {
        const std::string door = door_of(parts[i]);
        long d_belief = 0, d_occ = 0;
        int n_miss = 0;

        std::cout << "\n  ▸ " << door << "\n";
        while (i < parts.size() && door_of(parts[i]) == door) {
            const int b = belief_counts[i], o = occ_counts[i];
            const double pct = b > 0 ? 100.0 * o / b : 0.0;
            std::cout << (o > 0 ? "    ✅ " : "    ❌ ")
                      << parts[i].stem().string() << "  %" << static_cast<int>(pct)
                      << "  (" << o << " / " << b << ")"
                      << (o > 0 ? "" : "  MISSING") << "\n";
            if (o == 0) ++n_miss;
            d_belief += b;
            d_occ    += o;
            ++i;
        }
        const double d_pct = d_belief > 0 ? 100.0 * d_occ / d_belief : 0.0;
        std::cout << "    ── " << door << " toplam: %" << d_pct
                  << "  (" << d_occ << " / " << d_belief << " voxel)"
                  << (n_miss ? ("  |  " + std::to_string(n_miss) + " yama hic gorulmedi")
                             : "")
                  << "\n";
        g_belief += d_belief;
        g_occ    += d_occ;
    }

    const double g_pct = g_belief > 0 ? 100.0 * g_occ / g_belief : 0.0;
    std::cout << "\n  ═════════════════════════════════════════════════\n";
    std::cout << "  KAPILAR TOPLAM : %" << g_pct << "  (" << g_occ << " / " << g_belief
              << " voxel)\n";
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    std::cout << std::unitbuf;   // pipe'a yazarken satır satır aksın
    std::cerr << std::unitbuf;

    Config cfg = parse_args(argc, argv);

    std::cout << "══════════════════════════════════════════════════════\n";
    std::cout << "  Doors Octomap Builder — İki Aşamalı\n";
    std::cout << "  Mode        : " << cfg.mode        << "\n";
    std::cout << "  Parts dir   : " << cfg.parts_dir   << "\n";
    std::cout << "  Parts scale : " << cfg.parts_scale << "  (kapilar METRE)\n";
    std::cout << "  Data dir    : " << cfg.data_dir    << "\n";
    std::cout << "  Kollar      : " << (cfg.use_ur ? "UR " : "")
                                    << (cfg.use_kawasaki ? "Kawasaki" : "") << "\n";
    std::cout << "  Belief out  : " << cfg.belief_out  << "\n";
    std::cout << "  Occ out     : " << cfg.occ_out     << "\n";
    std::cout << "  Resolution  : " << cfg.res         << " m\n";
    std::cout << "  Snap        : " << (cfg.snap ? "acik (26 komsu)" : "kapali") << "\n";
    std::cout << "══════════════════════════════════════════════════════\n";

    auto parts = collect_parts(cfg.parts_dir);
    if (parts.empty()) {
        std::cerr << "❌ parts_dir'de .pcd yok: " << cfg.parts_dir
                  << "\n   Once uret:  ros2 run doors_inspection doors_parts_prep\n";
        return 1;
    }

    auto scans = collect_scans(cfg);
    if (scans.empty()) {
        std::cerr << "❌ Eslesen PCD+pose cifti yok: " << cfg.data_dir << "\n";
        return 1;
    }

    std::cout << "\n" << parts.size() << " kapi yamasi  |  " << scans.size()
              << " sensor taramasi\n";

    octomap::ColorOcTree* tree = new octomap::ColorOcTree(cfg.res);

    std::vector<int> belief_counts;
    VoxelPartMap voxel_part_map;
    build_belief_map(tree, parts, cfg.parts_scale, belief_counts, voxel_part_map);

    std::cout << "\n💾 Belief map: " << cfg.belief_out << " ... ";
    if (!tree->write(cfg.belief_out)) {
        std::cerr << "HATA! Dosya yazilamadi.\n"; delete tree; return 1;
    }
    std::cout << "OK\n";

    std::vector<int> occ_counts(parts.size(), 0);
    build_occupancy_map(tree, scans, voxel_part_map, occ_counts, cfg.z_min, cfg.snap);

    std::cout << "\n💾 Occupancy map: " << cfg.occ_out << " ... ";
    if (!tree->write(cfg.occ_out)) {
        std::cerr << "HATA! Dosya yazilamadi.\n"; delete tree; return 1;
    }
    std::cout << "OK\n";

    print_coverage(parts, belief_counts, occ_counts);

    std::cout << "\n✅ Tamamlandi.\n";
    delete tree;
    return 0;
}
