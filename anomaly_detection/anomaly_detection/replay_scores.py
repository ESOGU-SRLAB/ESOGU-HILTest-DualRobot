#!/usr/bin/env python3
"""
replay_scores.py
================
Kaydedilmiş bir oturumu arayüze GERİ YAYINLAR.

Ne işe yarar
------------
`skorlar_*.csv` her kararı sakladığı için bir oturum sonradan aynen
canlandırılabilir: robot çalışmadan, dedektör çalışmadan, FMU'ya gerek olmadan.
Arayüz canlı oturumdan ayırt edemez, çünkü aynı konulara aynı mesajlar gider.

Ekran görüntüsü almak, bir olayı tekrar tekrar izlemek ya da arayüz
değişikliklerini gerçek veriyle sınamak için.

Kayıt formatı
-------------
CSV sütunları düğümün `~/detail` dizisiyle BİREBİR aynı sırada:

    t_ros, s_kal, s_ham, z_kal, z_ham, birlesik, thr_mutlak, thr_uyarlanabilir,
    hit_mutlak, hit_uyarlanabilir, hit_kal, hit_ham, hareket, qd_tepe,
    taban_n, donmus, alarm

`t_ros` atlanır, `alarm` `~/detected` konusuna gider, aradaki 15 alan
`~/detail` dizisini oluşturur.

Kullanım
--------
    # tüm koşu, gerçek hızda
    ros2 run anomaly_detection replay_scores --ros-args \\
        -p csv:=~/anomali_kayit/.../skorlar_20260826_162028.csv

    # 2. olayın çevresindeki 30 s, sonsuz döngü (ekran görüntüsü için)
    ros2 run anomaly_detection replay_scores --ros-args \\
        -p csv:=.../skorlar_20260826_162028.csv -p event:=2 -p loop:=true

    # yavaşlatarak izle
    ... -p speed:=0.25
"""

from __future__ import annotations

import csv as _csv
import glob
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray

# CSV'de t_ros'tan sonra gelen ve `~/detail` dizisini oluşturan 15 alan.
DETAIL_COLS = ["s_kal", "s_ham", "z_kal", "z_ham", "birlesik", "thr_mutlak",
               "thr_uyarlanabilir", "hit_mutlak", "hit_uyarlanabilir",
               "hit_kal", "hit_ham", "hareket", "qd_tepe", "taban_n", "donmus"]


class ScoreReplay(Node):

    def __init__(self):
        super().__init__("ur10e_anomaly_detector")   # arayüz bu ismi bekliyor
        p = self.declare_parameter
        p("csv", "")
        p("event", 0)          # >0: olaylar_*.jsonl'deki o sıradaki olaya ortala
        p("pad_s", 15.0)       # olayın öncesine/sonrasına eklenecek saniye
        p("start_s", 0.0)
        p("duration_s", 0.0)   # 0 = sona kadar
        p("speed", 1.0)
        p("loop", False)
        p("rate", 20.0)        # kayıt hızı; CSV'de zaman damgası varsa o kullanılır

        g = lambda k: self.get_parameter(k).value          # noqa: E731
        path = str(g("csv")).strip()
        if not path:
            raise SystemExit("csv parametresi zorunlu: -p csv:=/yol/skorlar_*.csv")
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            raise SystemExit(f"bulunamadı: {path}")

        rows = self._load(path)
        t0 = rows[0]["t"]
        span = rows[-1]["t"] - t0

        a, b = float(g("start_s")), float(g("duration_s"))
        ev = int(g("event"))
        if ev > 0:
            a, b = self._event_window(path, ev, float(g("pad_s")), t0, span)
        lo = t0 + a
        hi = t0 + (a + b if b > 0 else span)
        self.rows = [r for r in rows if lo <= r["t"] <= hi]
        if not self.rows:
            raise SystemExit(f"seçilen aralıkta karar yok (start={a:.1f}s, süre={b:.1f}s)")

        self.speed = max(float(g("speed")), 1e-3)
        self.loop = bool(g("loop"))
        ns = "/" + self.get_name()
        self.pub_detail = self.create_publisher(Float32MultiArray, ns + "/detail", 20)
        self.pub_det = self.create_publisher(Bool, ns + "/detected", 20)
        self.pub_score = self.create_publisher(Float32, ns + "/score", 20)

        d = self.rows[-1]["t"] - self.rows[0]["t"]
        peak = max(r["detail"][4] for r in self.rows)
        self.get_logger().info(
            f"YENİDEN OYNATMA — {os.path.basename(path)}\n"
            f"  {len(self.rows):,} karar · {d:.1f} s · hız ×{self.speed:g}"
            f"{' · DÖNGÜ' if self.loop else ''}\n"
            f"  tepe birleşik {peak:.2f} · eşik {self.rows[0]['detail'][5]:.2f}\n"
            f"  Bu bir KAYIT; robot ya da dedektör çalışmıyor.")

        self.i = 0
        self.t_wall = time.monotonic()
        self.t_rec = self.rows[0]["t"]
        self.timer = self.create_timer(1.0 / (float(g("rate")) * self.speed), self._tick)

    # ── yükleme ──
    def _load(self, path: str) -> list[dict]:
        out = []
        with open(path, newline="") as f:
            rd = _csv.DictReader(f)
            miss = [c for c in DETAIL_COLS if c not in rd.fieldnames]
            if miss:
                raise SystemExit(f"CSV'de eksik sütun: {miss}\n"
                                 f"  (eski sürüm kayıt olabilir)")
            for r in rd:
                try:
                    out.append({"t": float(r["t_ros"]),
                                "detail": [float(r[c]) for c in DETAIL_COLS],
                                "alarm": float(r.get("alarm", 0)) > 0.5})
                except (TypeError, ValueError):
                    continue
        if not out:
            raise SystemExit("CSV boş ya da okunamadı")
        return out

    def _event_window(self, csv_path: str, n: int, pad: float,
                      t0: float, span: float) -> tuple[float, float]:
        """olaylar_*.jsonl'den n. olayı bulup çevresine pencere açar."""
        ev_path = csv_path.replace("skorlar_", "olaylar_").replace(".csv", ".jsonl")
        if not os.path.isfile(ev_path):
            self.get_logger().warn(f"{os.path.basename(ev_path)} yok → tüm koşu")
            return 0.0, 0.0
        beg = end = None
        for line in open(ev_path):
            e = json.loads(line)
            if e.get("sira") != n:
                continue
            if e.get("olay") == "anomali_basladi":
                beg = float(e["t_ros"])
            elif e.get("olay") == "anomali_bitti":
                end = float(e["t_ros"])
        if beg is None:
            self.get_logger().warn(f"olay #{n} bulunamadı → tüm koşu")
            return 0.0, 0.0
        end = end if end is not None else beg
        a = max(0.0, (beg - t0) - pad)
        b = (end - beg) + 2 * pad
        self.get_logger().info(
            f"  olay #{n} çevresi: t={beg-t0:.1f}s, süre {end-beg:.2f}s, "
            f"pencere {a:.1f}–{a+b:.1f}s")
        return a, min(b, span - a)

    # ── yayın ──
    def _tick(self) -> None:
        if self.i >= len(self.rows):
            if not self.loop:
                self.get_logger().info("Oynatma bitti.")
                self.timer.cancel()
                return
            self.i = 0
            self.t_wall = time.monotonic()
            self.t_rec = self.rows[0]["t"]

        # Kayıttaki zamana sadık kal: karar periyodu 50 ms nominal ama gerçek
        # kayıtta 495 Hz örnekleme yüzünden kayıyor; duvar saatiyle kıyaslayıp
        # geride kalınan kadar mesaj gönderiyoruz.
        target = (time.monotonic() - self.t_wall) * self.speed
        sent = 0
        while self.i < len(self.rows) and (self.rows[self.i]["t"] - self.t_rec) <= target:
            r = self.rows[self.i]
            m = Float32MultiArray()
            m.data = [float(x) for x in r["detail"]]
            self.pub_detail.publish(m)
            self.pub_det.publish(Bool(data=r["alarm"]))
            self.pub_score.publish(Float32(data=float(r["detail"][4])))
            self.i += 1
            sent += 1
            if sent > 200:            # patlamayı sınırla
                break


def main() -> None:
    # ExternalShutdownException: Ctrl-C ya da SIGTERM ile durdurulduğunda rclpy
    # bunu fırlatır. Normal sonlanma yolu; yığın izi basmak kullanıcıyı hataya
    # inandırır.
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = None
    try:
        node = ScoreReplay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except SystemExit as e:
        if e.code:
            print(e.code)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
