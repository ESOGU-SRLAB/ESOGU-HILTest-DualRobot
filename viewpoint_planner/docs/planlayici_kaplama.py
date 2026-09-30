#!/usr/bin/env python3
"""Plan dosyalarındaki planlayıcı kaplamasını KESİN sayılara çevirir ve gerçekte
koşan bakış-noktası kümesi için aynı modelle yeniden hesaplar.

    python3 docs/planlayici_kaplama.py

Çıktı: viewpoint_planner/docs/planlayici_kaplama.json ve
       multirobot_viewpoint_planner/docs/planlayici_kaplama.json
(rapor üreticileri sayıları buradan okur; raporda elle yazılmış yüzde yoktur).

NEDEN YENİDEN HESAP GEREKİYOR
  Planlayıcı `coverage_achieved`'i plan ÜRETİLDİĞİ ANDAKİ küme için yazar. Planlar
  sonradan elle düzenlendi (bakış-noktası çıkarıldı / taşındı) ve tek-kol planında
  zincire bağlanamayan iki durak (skipped_viewpoints) da o değere dahil. Yani dosyadaki
  sayı gerçekte koşan kümeyi anlatmaz.

NEDEN DOSYADAKİ SAYI BİREBİR YENİDEN ÜRETİLEMEZ
  Hedef noktalar her planlama koşusunda `trimesh.sample.sample_surface` ile TOHUMSUZ
  örneklenir ve plana kaydedilmez. Bu yüzden:
    * dosyadaki değer, birikimli değerlerin ortak paydasından TAM KESRE çevrilir
      (ör. 2994 / 4270 hedef nokta);
    * yeniden hesap, planlayıcının kendi görünürlük fonksiyonuyla
      (ViewpointGenerator.visible_mask: menzil, FOV, geliş açısı, occlusion) ve aynı
      örnekleme kuralıyla (normal_z >= -0.95), ama sabit tohumlu ve çok yoğun bir
      örneklemeyle yapılır;
    * yöntem, dosyadaki değerin ait olduğu ORİJİNAL küme üzerinde doğrulanır.
"""
import json
import math
import os
import sys
import time
from datetime import datetime
from fractions import Fraction

import numpy as np
import trimesh

SRC = os.path.expanduser("~/colcon_ws/src")
sys.path.insert(0, os.path.join(SRC, "viewpoint_planner"))
from viewpoint_planner.mesh_analyzer import MeshAnalyzer          # noqa: E402
from viewpoint_planner.viewpoint_generator import ViewpointGenerator  # noqa: E402

MESH = os.path.join(SRC, "Universal_Robots_ROS2_Description/meshes/ur10e/collision/chassis.stl")
MESH_SCALE = 0.001            # planlayıcı parametresi mesh_scale
MIN_NORMAL_Z = -0.95          # MeshAnalyzer.sample_surface varsayılanı
PLANNER_SAMPLES = 5000        # planlayıcı parametresi target_sample_points
DENSE_SAMPLES = 500_000
SEEDS = [0, 1, 2, 3, 4]

PLANS = {
    "viewpoint_planner": dict(
        out=os.path.join(SRC, "viewpoint_planner/docs/planlayici_kaplama.json"),
        executed=os.path.join(SRC, "viewpoint_planner/plans/viewpoint_plan.json"),
        original=os.path.join(SRC, "viewpoint_planner/plans/viewpoint_plan_before_remove_1742.json"),
        keys=["ur_viewpoints"]),
    "multirobot_viewpoint_planner": dict(
        out=os.path.join(SRC, "multirobot_viewpoint_planner/docs/planlayici_kaplama.json"),
        executed=os.path.join(SRC, "multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json"),
        original=os.path.join(SRC, "multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan_backup_2026-09-08.json"),
        keys=["ur_viewpoints", "kawasaki_viewpoints"]),
}


class _Quiet:
    def info(self, *a, **k): pass
    def debug(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k): print(*a)


def exact_written(plan, keys):
    """coverage_achieved -> k / N. N, bütün birikimli değerlerin ortak paydasıdır."""
    fr = [Fraction(plan["coverage_achieved"]).limit_denominator(PLANNER_SAMPLES)]
    for k in keys:
        for vp in plan.get(k, []):
            if vp.get("cumulative_coverage") is not None:
                fr.append(Fraction(vp["cumulative_coverage"]).limit_denominator(PLANNER_SAMPLES))
    N = 1
    for f in fr:
        N = N * f.denominator // math.gcd(N, f.denominator)
    consistent = all(abs(round(float(f) * N) / N - float(f)) < 1e-12 for f in fr)
    c = fr[0]
    k = c.numerator * (N // c.denominator)
    assert abs(k / N - plan["coverage_achieved"]) < 1e-12
    return dict(covered=k, targets=N, fraction=plan["coverage_achieved"], percent=100 * k / N,
                denominator_consistent_over=len(fr), consistent=consistent)


def sample_targets(mesh, n, seed):
    """MeshAnalyzer.sample_surface ile AYNI kural, tek farkla: tohum sabit."""
    pts, face_idx = trimesh.sample.sample_surface(mesh, n, seed=seed)
    nrm = mesh.face_normals[face_idx]
    keep = nrm[:, 2] >= MIN_NORMAL_Z
    return pts[keep], nrm[keep]


def union_coverage(vg, inter, vps, pts, nrm):
    covered = np.zeros(len(pts), dtype=bool)
    for vp in vps:
        covered |= vg.visible_mask(vp["position"], vp["rotation"], pts, nrm, inter)
    return int(covered.sum()), len(pts)


def viewpoints(plan, keys):
    return [vp for k in keys for vp in plan.get(k, [])]


def main():
    t_all = time.time()
    analyzer = MeshAnalyzer(MESH, scale=MESH_SCALE, logger=_Quiet())
    analyzer.load_mesh()
    mesh = analyzer.mesh
    dense = {s: sample_targets(mesh, DENSE_SAMPLES, s) for s in SEEDS}

    for name, cfg in PLANS.items():
        executed = json.load(open(cfg["executed"]))
        original = json.load(open(cfg["original"]))
        vg = ViewpointGenerator(analyzer, config=executed["camera_config"], logger=_Quiet())
        inter = vg.make_intersector()
        result = dict(
            generated_at=datetime.now().isoformat(timespec="seconds"),
            method=dict(mesh=MESH, mesh_scale=MESH_SCALE, min_normal_z=MIN_NORMAL_Z,
                        dense_samples=DENSE_SAMPLES, seeds=SEEDS,
                        visibility="viewpoint_planner.ViewpointGenerator.visible_mask",
                        camera_config=executed["camera_config"]),
            written=dict(exact_written(executed, cfg["keys"]),
                         plan_file=cfg["executed"],
                         refers_to_set_in=cfg["original"],
                         original_viewpoints=len(viewpoints(original, cfg["keys"])),
                         skipped_viewpoints=original.get("skipped_viewpoints") or []),
        )
        for label, plan in (("original", original), ("executed", executed)):
            vps = viewpoints(plan, cfg["keys"])
            per_seed = []
            for s in SEEDS:
                pts, nrm = dense[s]
                c, n = union_coverage(vg, inter, vps, pts, nrm)
                per_seed.append(dict(seed=s, covered=c, targets=n, percent=100 * c / n))
            pcts = np.array([r["percent"] for r in per_seed])
            result[label] = dict(
                plan_file=cfg["original"] if label == "original" else cfg["executed"],
                viewpoints=len(vps),
                viewpoint_ids=[vp["id"] for vp in vps],
                per_seed=per_seed,
                percent_mean=float(pcts.mean()),
                percent_min=float(pcts.min()),
                percent_max=float(pcts.max()),
            )
        # Atlanan duraklar: pozları planda saklanmaz ama planlayıcı örneklemesindeki
        # katkıları KESİN olarak bilinir: bakış-noktası başına 'yeni nokta' sayıları
        # toplamı kaplanan hedef sayısına eşittir, eksik kalan kısım atlananlarındır.
        w = result["written"]
        orig_vps = viewpoints(original, cfg["keys"])
        if all(vp.get("new_points_covered") is not None for vp in orig_vps):
            stored_new = sum(int(vp["new_points_covered"]) for vp in orig_vps)
            skipped_new = w["covered"] - stored_new
            w["stored_new_points_sum"] = stored_new
            w["skipped_new_points"] = skipped_new
            # Bir durağı çıkarmak, en fazla o durağın 'yeni' saydığı noktaları kaybettirir.
            lb = w["covered"] - skipped_new
            result["original"]["planner_sample_lower_bound"] = dict(
                covered=lb, targets=w["targets"], percent=100 * lb / w["targets"])

        # Doğrulama: dosyadaki değer, orijinal küme için 5000'lik tek bir örneklemeden
        # geldi. O örneklemenin binom standart sapması ile ne kadar uyumlu? (Atlanan
        # durak varsa karşılaştırma, saklanan kümenin alt sınırıyla yapılır.)
        ref = result["original"].get("planner_sample_lower_bound", dict(percent=w["percent"]))
        p = result["original"]["percent_mean"] / 100.0
        sigma = 100.0 * math.sqrt(p * (1 - p) / w["targets"])
        result["validation"] = dict(
            original_dense_percent=result["original"]["percent_mean"],
            written_percent=w["percent"],
            compared_planner_percent=ref["percent"],
            difference_points=ref["percent"] - result["original"]["percent_mean"],
            sampling_sigma_points=sigma,
            z_score=(ref["percent"] - result["original"]["percent_mean"]) / sigma,
            note=("Karşılaştırma, atlanan durakların katkısı çıkarılmış alt sınırla yapıldı. "
                  "Kalan pozitif fark örnekleme gürültüsüyle açıklanmıyorsa en olası sebep "
                  "seçim yanlılığıdır: planlayıcı bakış-noktalarını raporladığı örneklemenin "
                  "KENDİSİ üzerinde seçer (doğrulanmadı; seçim IK gerektirir).")
            if result["written"]["skipped_viewpoints"] else "",
        )
        with open(cfg["out"], "w") as fh:
            json.dump(result, fh, indent=2, ensure_ascii=False)
        v = result["validation"]
        print(f"\n{name}")
        if "skipped_new_points" in w:
            print(f"  atlananların katkısı: {w['skipped_new_points']} hedef nokta; saklanan küme alt sınırı "
                  f"%{result['original']['planner_sample_lower_bound']['percent']:.3f}")
        print(f"  dosyadaki değer : {w['covered']}/{w['targets']} = %{w['percent']:.4f} "
              f"({w['original_viewpoints']} bakış-noktası + atlanan {w['skipped_viewpoints']})")
        print(f"  orijinal küme   : %{result['original']['percent_mean']:.3f} "
              f"(tohumlar {result['original']['percent_min']:.3f}..{result['original']['percent_max']:.3f}) "
              f"| fark {v['difference_points']:+.3f} puan, 5000-örnek σ={v['sampling_sigma_points']:.3f}, z={v['z_score']:+.2f}")
        print(f"  KOŞAN küme      : %{result['executed']['percent_mean']:.3f} "
              f"(tohumlar {result['executed']['percent_min']:.3f}..{result['executed']['percent_max']:.3f}), "
              f"{result['executed']['viewpoints']} bakış-noktası")
        print(f"  yazıldı: {cfg['out']}")
    print(f"\ntoplam {time.time() - t_all:.0f} s")


if __name__ == "__main__":
    main()
