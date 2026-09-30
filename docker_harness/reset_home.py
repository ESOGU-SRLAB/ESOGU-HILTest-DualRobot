#!/usr/bin/env python3
"""
Harness reset: her STLC testinden sonra sim UR10e'yi home pozisyonuna döndürür.

entrypoint.sh bu dosyayı iki kez çağırır:
  1. Testten ÖNCE:  reset_home.py snapshot --out /tmp/scene_before.json
     Planning scene'deki nesnelerin listesini kaydeder. Test sonrası yalnızca
     testin eklediği nesneleri silebilmek için gerekli.
  2. Testten SONRA: reset_home.py reset --snapshot /tmp/scene_before.json

Reset kademeleri (her biri yalnızca bir öncekinin yetmediği durumda devreye girer):
  0. Hazırlık  : yığın ayakta mı, yarım kalan hareketleri iptal et, robot dursun.
  1. Temizlik  : testin eklediği / tutturduğu (attach) nesneleri planning scene'den sil.
  2. MoveIt    : çarpışma kontrollü plan ile home'a git.
  3. Kaçış     : MoveIt "başlangıç durumu çarpışmada" diye plan yapamıyorsa,
                 küçük adımlarla (her ara nokta check_state_validity ile sınanarak)
                 çarpışmadan çıkan kısa bir yol bulunur ve controller'a DOĞRUDAN
                 gönderilir. Sonra tekrar kademe 2.
Uzun yol her zaman MoveIt ile gider. Controller'a doğrudan giden tek şey,
en fazla ~13 derece / 13 cm'lik (10 adım + 3 adım pay), adım adım sınanmış kaçış hareketidir.

Çıktı: stdout'a tek satır, makinece okunabilir sonuç:
  [harness] reset: status=homed tiers=cleanup,moveit max_err_rad=0.0040 ...
Çıkış kodu: 0 = home'da (zaten ya da götürüldü), 3 = yığın kapalı, 4 = başarısız.

Kasıtlı olarak pymoveit2 KULLANILMIYOR: kütüphane beklerken rclpy.spin_once ile
node'u global executor'a devrediyor ve bu da sessiz sağırlığa yol açabiliyor.
Burada tek thread var ve her bekleme spin_until_future_complete ile yapılıyor.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# ---------------------------------------------------------------------------
# Hücreye özgü sabitler (hil_test_whole_unified.launch.py, digital_twin:=true)
# ---------------------------------------------------------------------------

# Eklem sırası: pymoveit2_sim/robots/ur.py ile aynı. İlk eklem lineer ray (metre),
# diğerleri döner eklem (radyan).
JOINTS = [
    "sim_ur10e_base_to_robot_mount",
    "sim_ur10e_shoulder_pan_joint",
    "sim_ur10e_shoulder_lift_joint",
    "sim_ur10e_elbow_joint",
    "sim_ur10e_wrist_1_joint",
    "sim_ur10e_wrist_2_joint",
    "sim_ur10e_wrist_3_joint",
]
RAIL = 0  # JOINTS içinde rayın indeksi

# Home: ray 1 m, kol sensing_robot.py / sim_robot_joint_goal.py'deki home ile aynı.
HOME = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

GROUP = "sim_ur10e"
JOINT_STATES_TOPIC = "/sim/joint_states"
MOVE_ACTION = "/sim/move_action"
EXECUTE_ACTION = "/sim/execute_trajectory"
JTC_ACTION = "/sim/sim_scaled_joint_trajectory_controller/follow_joint_trajectory"
GET_SCENE_SRV = "/sim/get_planning_scene"
APPLY_SCENE_SRV = "/sim/apply_planning_scene"
VALIDITY_SRV = "/sim/check_state_validity"

# "Home'da mı?" toleransları. Gazebo'daki PID durağan halde bile birkaç mrad
# sapma bırakabildiği için çok sıkı tutulmadı. Ortam değişkeniyle değiştirilebilir.
TOL_RAD = float(os.environ.get("HARNESS_HOME_TOL_RAD", "0.02"))   # ~1.1 derece
TOL_RAIL = float(os.environ.get("HARNESS_HOME_TOL_RAIL", "0.01"))  # 1 cm

# MoveIt ile home'a giderken hız/ivme ölçeği (0-1). Reset hızlı değil güvenli olmalı.
MOVEIT_SCALING = 0.1

# Kaçış (kademe 3) ayarları: her adım en fazla 1 derece / 1 cm, en fazla 10 adım.
ESCAPE_STEP_RAD = math.radians(1.0)
ESCAPE_STEP_RAIL = 0.01
ESCAPE_MAX_STEPS = 10
# Çarpışmadan çıkılan ilk noktada durmak yetmiyor: robot sınırın <1° dışında kalıyor
# ve MoveIt oradan çıkan planı INVALID_MOTION_PLAN (-2) ile reddediyor (canlı yığında
# ölçüldü, 29 Eyl 2026). Aynı yönde, hepsi geçerli olmak şartıyla, bu kadar adım daha
# gidilir. 3 adımlık (3°) pay ile MoveIt home'a planlayabildi.
ESCAPE_MARGIN_STEPS = 3
# Kaçış hareketinin controller'a gönderilirken kullanılacak hızları (yavaş).
ESCAPE_VEL_RAD = 0.1   # rad/s
ESCAPE_VEL_RAIL = 0.05  # m/s


# ---------------------------------------------------------------------------
# Saf fonksiyonlar (ROS'suz; sahte verilerle test edilebilir)
# ---------------------------------------------------------------------------

def home_error(q, home=HOME):
    """(en büyük döner eklem hatası [rad], ray hatası [m]) döndürür."""
    err_rad = max(abs(q[i] - home[i]) for i in range(len(q)) if i != RAIL)
    err_rail = abs(q[RAIL] - home[RAIL])
    return err_rad, err_rail


def is_home(q, home=HOME):
    err_rad, err_rail = home_error(q, home)
    return err_rad <= TOL_RAD and err_rail <= TOL_RAIL


def escape_directions(q0, home=HOME):
    """
    Denenecek kaçış yönlerini sırayla üretir. Her yön, TEK adımlık eklem
    değişimi vektörüdür (bir adımda hiçbir eklem 1 derece / 1 cm'den fazla oynamaz).

    Sıra:
      1. Home'a doğru (en doğal kaçış; işe yararsa yol zaten kısalır).
      2. Her eklem tek başına + ve - yönde (7 eklem x 2 = 14 yön).
         Robot bir yüzeye dayanmışsa genelde tek bir eklemi geri almak yeter.
    """
    n = len(q0)
    steps = [ESCAPE_STEP_RAIL if i == RAIL else ESCAPE_STEP_RAD for i in range(n)]

    # 1) Home'a doğru: en çok değişmesi gereken eklem tam bir adım atacak şekilde ölçekle.
    diff = [home[i] - q0[i] for i in range(n)]
    ratio = max(abs(diff[i]) / steps[i] for i in range(n))
    if ratio > 1e-9:
        yield "toward_home", [d / ratio for d in diff]

    # 2) Tek eklem hareketleri.
    for i in range(n):
        for sign, tag in ((+1, "+"), (-1, "-")):
            vec = [0.0] * n
            vec[i] = sign * steps[i]
            yield f"{JOINTS[i].replace('sim_ur10e_', '')}{tag}", vec


def find_escape(q0, validity_fn, home=HOME, max_steps=ESCAPE_MAX_STEPS,
                margin_steps=ESCAPE_MARGIN_STEPS):
    """
    Çarpışmadaki q0'dan çıkan kısa bir yol arar.

    validity_fn(q) -> (gecerli_mi: bool, temas_ciftleri: set)
        MoveIt'in check_state_validity servisini saran fonksiyon. Testte sahtesi verilir.

    Kural: yol boyunca hiçbir ara nokta, q0'da OLMAYAN yeni bir temas çifti
    (ör. kol<->masa) içeremez. Yani mevcut çarpışmadan çıkarken başka bir
    şeye çarpmıyoruz. İlk geçerli (çarpışmasız) noktaya ulaşınca, aynı yönde
    margin_steps adım daha pay alırız (her biri geçerli olmalı; olmayan ilk
    adımda pay alma biter ve o ana kadarki yol kullanılır).

    Döndürür: (yön_adı, [q1, q2, ..., qk]) ya da bulunamazsa (None, None).
    """
    valid0, pairs0 = validity_fn(q0)
    if valid0:
        return "none_needed", []

    for name, vec in escape_directions(q0, home):
        path = []
        for k in range(1, max_steps + 1):
            qk = [q0[i] + k * vec[i] for i in range(len(q0))]
            valid, pairs = validity_fn(qk)
            if not pairs <= pairs0:
                break  # yeni bir şeye çarpıyor: bu yön kullanılamaz
            path.append(qk)
            if valid:
                for m in range(1, margin_steps + 1):
                    qm = [q0[i] + (k + m) * vec[i] for i in range(len(q0))]
                    if not validity_fn(qm)[0]:
                        break
                    path.append(qm)
                return name, path
    return None, None


def segment_duration(qa, qb):
    """İki nokta arası yavaş hareket süresi (sn): en yavaş eklemi belirler."""
    t = 0.2
    for i in range(len(qa)):
        vel = ESCAPE_VEL_RAIL if i == RAIL else ESCAPE_VEL_RAD
        t = max(t, abs(qb[i] - qa[i]) / vel)
    return t


# ---------------------------------------------------------------------------
# ROS tarafı
# ---------------------------------------------------------------------------

class ResetNode(Node):
    def __init__(self):
        super().__init__("harness_reset_home")
        # Son görülen eklem konumları. /sim/joint_states birden fazla yayıncıdan
        # (UR, Kawasaki...) parça parça gelebildiği için isim bazlı birleştiriyoruz.
        self._positions = {}
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_js, 10)

        self.move_client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.jtc_client = ActionClient(self, FollowJointTrajectory, JTC_ACTION)
        self.get_scene = self.create_client(GetPlanningScene, GET_SCENE_SRV)
        self.apply_scene = self.create_client(ApplyPlanningScene, APPLY_SCENE_SRV)
        self.validity = self.create_client(GetStateValidity, VALIDITY_SRV)

    # --- yardımcılar -------------------------------------------------------

    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = pos

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, end - time.monotonic()))

    def wait(self, future, timeout):
        """Future'ı bekler; zaman aşımında None döner."""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def current(self):
        """Güncel eklem konumları (JOINTS sırasıyla) ya da eksik varsa None."""
        if all(j in self._positions for j in JOINTS):
            return [self._positions[j] for j in JOINTS]
        return None

    # --- kademe 0: hazırlık ------------------------------------------------

    def wait_joint_states(self, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.current() is not None:
                return True
            self.spin_for(0.1)
        return False

    def cancel_all(self, action_name):
        """
        Bir action sunucusundaki TÜM hedefleri iptal eder. Testi yarıda kesilmiş
        olsa bile controller, test gönderdiği son yörüngeyi yürütmeye devam eder.
        CancelGoal isteğinde goal_id ve stamp sıfır bırakılırsa "hepsini iptal et"
        anlamına gelir (action_msgs standardı).
        """
        client = self.create_client(CancelGoal, action_name + "/_action/cancel_goal")
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return 0
            resp = self.wait(client.call_async(CancelGoal.Request()), 3.0)
            return len(resp.goals_canceling) if resp else 0
        finally:
            self.destroy_client(client)

    def wait_stationary(self, timeout=20.0, window=0.5, thresh=1e-3):
        """Robot durana kadar bekler (window saniyede hiçbir eklem thresh'ten fazla oynamaz)."""
        end = time.monotonic() + timeout
        prev = self.current()
        while time.monotonic() < end:
            self.spin_for(window)
            cur = self.current()
            if prev and cur and max(abs(a - b) for a, b in zip(prev, cur)) < thresh:
                return True
            prev = cur
        return False

    # --- kademe 1: planning scene temizliği --------------------------------

    def scene_objects(self):
        """(dünya nesne id'leri, attach edilmiş nesne [(id, link)]) ya da None."""
        if not self.get_scene.wait_for_service(timeout_sec=5.0):
            return None
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        resp = self.wait(self.get_scene.call_async(req), 5.0)
        if resp is None:
            return None
        world = [co.id for co in resp.scene.world.collision_objects]
        attached = [
            (aco.object.id, aco.link_name)
            for aco in resp.scene.robot_state.attached_collision_objects
        ]
        return world, attached

    def cleanup(self, before):
        """
        Testten önce OLMAYIP şimdi olan nesneleri siler. Yığının kendi kalıcı
        nesnelerine (snapshot'ta olanlara) dokunmaz. Silinen id listesini döndürür.
        """
        now = self.scene_objects()
        if now is None:
            return None
        world_now, attached_now = now
        before_world = set(before.get("world", []))
        before_attached = {a[0] for a in before.get("attached", [])}

        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        removed = []

        # Attach edilmiş yeni nesneler: önce koldan çöz, sonra dünyadan da sil
        # (çözülen nesne dünyaya bırakılır; orada kalırsa yine engel olur).
        for obj_id, link in attached_now:
            if obj_id in before_attached:
                continue
            aco = AttachedCollisionObject(link_name=link)
            aco.object.id = obj_id
            aco.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(aco)
            co = CollisionObject(id=obj_id, operation=CollisionObject.REMOVE)
            co.header.frame_id = "world"
            scene.world.collision_objects.append(co)
            removed.append(obj_id)

        for obj_id in world_now:
            if obj_id in before_world or obj_id in removed:
                continue
            co = CollisionObject(id=obj_id, operation=CollisionObject.REMOVE)
            co.header.frame_id = "world"
            scene.world.collision_objects.append(co)
            removed.append(obj_id)

        if removed:
            if not self.apply_scene.wait_for_service(timeout_sec=3.0):
                return None
            resp = self.wait(
                self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)), 5.0
            )
            if resp is None or not resp.success:
                return None
        return removed

    # --- kademe 2: MoveIt ---------------------------------------------------

    def moveit_home(self, timeout=90.0):
        """MoveIt ile home'a planla + yürüt. MoveIt hata kodunu döndürür (1 = SUCCESS)."""
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return None
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = MOVEIT_SCALING
        req.max_acceleration_scaling_factor = MOVEIT_SCALING
        # is_diff=True: başlangıç = move_group'un gördüğü GÜNCEL durum
        # (attach edilmiş nesneler dahil).
        req.start_state.is_diff = True
        cons = Constraints()
        for name, pos in zip(JOINTS, HOME):
            cons.joint_constraints.append(JointConstraint(
                joint_name=name, position=pos,
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0,
            ))
        req.goal_constraints.append(cons)
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        handle = self.wait(self.move_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return None
        result = self.wait(handle.get_result_async(), timeout)
        if result is None:
            # Zaman aşımı: hareket hâlâ sürüyor olabilir, iptal et.
            self.wait(handle.cancel_goal_async(), 3.0)
            return None
        return result.result.error_code.val

    # --- kademe 3: kaçış ---------------------------------------------------

    def check_validity(self, q):
        """(geçerli_mi, {frozenset(gövde1, gövde2), ...}) döndürür."""
        req = GetStateValidity.Request()
        req.group_name = GROUP
        # is_diff=True: sadece UR eklemlerini değiştir; Kawasaki eklemleri ve
        # attach edilmiş nesneler move_group'un güncel durumundan alınır.
        req.robot_state = RobotState(is_diff=True)
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q)
        resp = self.wait(self.validity.call_async(req), 3.0)
        if resp is None:
            # Servis cevap vermediyse güvenli tarafta kal: geçersiz + "her şeye temas".
            return False, {frozenset(("?", "?"))}
        pairs = {frozenset((c.contact_body_1, c.contact_body_2)) for c in resp.contacts}
        return resp.valid, pairs

    def jtc_execute(self, start, path):
        """Kısa kaçış yolunu controller'a doğrudan gönderir. Başarılıysa True."""
        if not self.jtc_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        t = 0.0
        prev = start
        for q in path:
            t += segment_duration(prev, q)
            sec = int(t)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=list(q),
                time_from_start=Duration(sec=sec, nanosec=int((t - sec) * 1e9)),
            ))
            prev = q
        handle = self.wait(self.jtc_client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return False
        result = self.wait(handle.get_result_async(), t + 10.0)
        return result is not None and result.result.error_code == 0


# ---------------------------------------------------------------------------
# Komutlar
# ---------------------------------------------------------------------------

def report(status, t0, tiers, q=None, extra=""):
    parts = [f"status={status}", f"tiers={','.join(tiers) or '-'}"]
    if q is not None:
        err_rad, err_rail = home_error(q)
        parts += [f"max_err_rad={err_rad:.4f}", f"err_rail_m={err_rail:.4f}"]
    parts.append(f"duration={time.monotonic() - t0:.1f}s")
    if extra:
        parts.append(extra)
    print("[harness] reset: " + " ".join(parts), flush=True)


def cmd_snapshot(node, out_path):
    objs = node.scene_objects()
    if objs is None:
        print("[harness] snapshot: planning scene alınamadı (yığın kapalı?)", file=sys.stderr)
        return 3
    world, attached = objs
    with open(out_path, "w") as f:
        json.dump({"world": world, "attached": attached}, f)
    return 0


def cmd_reset(node, snapshot_path):
    t0 = time.monotonic()
    tiers = []

    # --- Kademe 0 ---
    if not node.wait_joint_states():
        report("stack_down", t0, tiers, extra="reason=no_joint_states")
        return 3
    canceled = sum(node.cancel_all(a) for a in (MOVE_ACTION, EXECUTE_ACTION, JTC_ACTION))
    if canceled:
        tiers.append(f"cancel{canceled}")
    if not node.wait_stationary():
        # Durmuyorsa (ör. Gazebo'da fizik titremesi) yine de devam et; MoveIt
        # start_state toleransı (0.1) içinde kalırsa planlayabilir.
        tiers.append("not_stationary")

    # --- Kademe 1 ---
    before = None
    if snapshot_path and os.path.exists(snapshot_path):
        with open(snapshot_path) as f:
            before = json.load(f)
    if before is not None:
        removed = node.cleanup(before)
        if removed is None:
            tiers.append("cleanup_failed")
        elif removed:
            tiers.append(f"cleanup{len(removed)}")
    else:
        # Snapshot yoksa hangi nesnelerin teste ait olduğunu bilemeyiz: hiçbir şey silme.
        tiers.append("no_snapshot")

    node.spin_for(0.3)
    if is_home(node.current()):
        report("already_home", t0, tiers, node.current())
        return 0

    # --- Kademe 2 (+ gerekirse 3, sonra tekrar 2) ---
    # En fazla 3 MoveIt denemesi; aralarında gerekirse kaçış. Son deneme de
    # MoveIt olduğu için kaçıştan sonra her zaman bir plan denemesi yapılır.
    for attempt in range(3):
        tiers.append("moveit")
        code = node.moveit_home()
        node.spin_for(0.5)
        if code == MoveItErrorCodes.SUCCESS and is_home(node.current()):
            report("homed", t0, tiers, node.current())
            return 0
        tiers[-1] = f"moveit({code})"

        # Plan başarısız: robot MoveIt'e göre çarpışmada mı?
        q0 = node.current()
        valid, _ = node.check_validity(q0)
        if valid or attempt == 2:
            continue  # çarpışma yok (planlayıcı bulamadı) ya da son tur: tekrar dene / bitir

        direction, path = find_escape(q0, node.check_validity)
        if path is None:
            report("failed", t0, tiers, q0, extra="reason=no_escape_found")
            return 4
        tiers.append(f"escape({direction},{len(path)})")
        if path and not node.jtc_execute(q0, path):
            report("failed", t0, tiers, node.current(), extra="reason=escape_exec_failed")
            return 4
        node.spin_for(0.5)

    report("failed", t0, tiers, node.current(), extra="reason=moveit_failed")
    return 4


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_snap = sub.add_parser("snapshot", help="testten önce planning scene nesnelerini kaydet")
    p_snap.add_argument("--out", required=True)
    p_reset = sub.add_parser("reset", help="testten sonra robotu home'a döndür")
    p_reset.add_argument("--snapshot", default=None)
    args = parser.parse_args()

    rclpy.init()
    node = ResetNode()
    try:
        if args.cmd == "snapshot":
            return cmd_snapshot(node, args.out)
        return cmd_reset(node, args.snapshot)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
