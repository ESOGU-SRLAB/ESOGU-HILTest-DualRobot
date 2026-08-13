#!/usr/bin/env python3
"""
OnRobot VGC10 vakum kavrayıcı arayüzü.

VGC10'da AKTÜE EDİLEN EKLEM YOKTUR. URDF'te vacuum_gripper ve suction_cup
link'leri sabit eklemlerle bağlı; vakum SRDF'inde de bir "gripper" planlama
grubu tanımlı değil. Bu yüzden MoveIt2Gripper bu donanımda tanımsızdır ve
kullanılamaz - kavrama, Modbus üzerinden pompa komutuyla yapılır.

Komut: vgc10_msgs/OnRobotVGOutput
    rmca/rmcb : kanal kontrol modu  0=Release, 1=Grip, 2=Idle
    rvca/rvcb : hedef vakum yüzdesi (yalnız Grip modunda; 80'i AŞMAMALI)
Durum:  vgc10_msgs/OnRobotVGInput
    gvca/gvcb : ölçülen vakum, bağıl vakumun 1/1000'i (yani % = gvca / 10)
"""

from __future__ import annotations

import time
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import Empty

from vgc10_msgs.msg import OnRobotVGInput, OnRobotVGOutput


# DİKKAT: bunlar OnRobot dokümanındaki 0/1/2 DEĞİL, ÜST BAYTA KAYDIRILMIŞ
# hâlleri. Sürücü (vgc10_control) rmca'yı ham register'ın üst baytı olarak
# alıyor ve düz topluyor:
#
#     comModbusTcp.sendCommand:  reg_a = rmca + rvca
#
# Yani %60 vakumla tutmak 0x0100 + 60 = 0x013C demek - mesaj dokümanındaki
# "0x014B = %75 grip" örneğiyle birebir aynı kodlama.
#
# 0/1/2 gönderilirse baseOnRobotVG.verifyCommand komutu REDDEDER ve hatayı
# bildirmek için ROS1'den kalma rclpy.signal_shutdown'ı çağırır; o da rclpy'de
# olmadığı için SÜRÜCÜ ÇÖKER (13 Ağu 2026, gerçek hücrede yaşandı:
# AttributeError: module 'rclpy' has no attribute 'signal_shutdown').
MODE_RELEASE = 0x0000
MODE_GRIP = 0x0100
MODE_IDLE = 0x0200

# OnRobot dokümanı hedef vakumun %80'i aşmamasını söylüyor.
MAX_VACUUM_PCT = 80


class VacuumGripper:
    """VGC10'u komutlar ve isteğe bağlı olarak kavramayı doğrular."""

    def __init__(
        self,
        node: Node,
        command_topic: str = "/OnRobotVGOutput",
        status_topic: str = "/OnRobotVGInput",
        vacuum_pct: int = 60,
        use_both_channels: bool = True,
        grip_confirm_pct: float = 15.0,
        grip_timeout_sec: float = 3.0,
        require_confirmation: bool = False,
        sim_attach_topic: str = "",
        sim_detach_topic: str = "",
    ):
        self._node = node
        self._vacuum_pct = max(0, min(MAX_VACUUM_PCT, int(vacuum_pct)))
        self._use_both = bool(use_both_channels)
        self._confirm_pct = float(grip_confirm_pct)
        self._timeout = float(grip_timeout_sec)
        self._require_confirmation = bool(require_confirmation)

        self._pub = node.create_publisher(OnRobotVGOutput, command_topic, 10)
        self._status: Optional[OnRobotVGInput] = None
        node.create_subscription(OnRobotVGInput, status_topic, self._on_status, 10)

        # Gazebo'da vakum diye bir şey yok: parçanın gerçekten taşınabilmesi
        # için emme kabı ile parça arasında koparılabilir bir eklem kurulur
        # (whole_cell_sim.urdf.xacro / DetachableJoint). Bu topic'ler ROS→Gazebo
        # köprüsüne gider.
        #
        # GERÇEK ROBOTTA ZARARSIZ: köprü yalnızca sim launch'ında var; gerçek
        # hücrede bu topic'i kimse dinlemez. Bu yüzden mode overlay'lerine yeni
        # bir anahtar eklemek gerekmedi - davranış kendi kendine kapanıyor.
        self._attach_pub = (
            node.create_publisher(Empty, sim_attach_topic, 10)
            if sim_attach_topic else None
        )
        self._detach_pub = (
            node.create_publisher(Empty, sim_detach_topic, 10)
            if sim_detach_topic else None
        )

        if int(vacuum_pct) > MAX_VACUUM_PCT:
            node.get_logger().warn(
                f"vacuum_pct={vacuum_pct} üst sınırı aşıyor, {MAX_VACUUM_PCT}'e kırpıldı."
            )
        node.get_logger().info(
            f"VacuumGripper hazır | komut={command_topic} durum={status_topic} "
            f"hedef={self._vacuum_pct}% onay={'açık' if require_confirmation else 'kapalı'}"
        )

    def _on_status(self, msg: OnRobotVGInput) -> None:
        self._status = msg

    @property
    def measured_pct(self) -> Optional[float]:
        """Ölçülen vakum yüzdesi (iki kanalın büyüğü); durum gelmemişse None."""
        if self._status is None:
            return None
        return max(self._status.gvca, self._status.gvcb) / 10.0

    def _send(self, mode: int, vacuum_pct: int) -> None:
        command = OnRobotVGOutput()
        command.rmca = mode
        command.rvca = vacuum_pct
        if self._use_both:
            command.rmcb = mode
            command.rvcb = vacuum_pct
        else:
            command.rmcb = MODE_IDLE
            command.rvcb = 0
        self._pub.publish(command)

    def grip(self) -> bool:
        """Pompayı çalıştırır. require_confirmation açıksa vakumun kurulmasını bekler.

        Onay kapalıyken True dönmesi "parça tutuldu" demek DEĞİLDİR, sadece
        komutun yayınlandığı anlamına gelir.
        """
        self._send(MODE_GRIP, self._vacuum_pct)
        self._node.get_logger().info(f"VAKUM AÇ (hedef {self._vacuum_pct}%)")
        # SIRA ÖNEMLİ: bağ, mesajın ULAŞTIĞI ANDAKİ göreli pozla kurulur.
        # Kap şu anda parçaya değiyor; hareket etmeden önce gönderilmeli.
        self._sim_attach()

        if not self._require_confirmation:
            time.sleep(0.5)
            return True

        deadline = time.time() + self._timeout
        while time.time() < deadline:
            measured = self.measured_pct
            if measured is not None and measured >= self._confirm_pct:
                self._node.get_logger().info(f"Vakum kuruldu: {measured:.1f}%")
                return True
            time.sleep(0.1)

        measured = self.measured_pct
        if measured is None:
            self._node.get_logger().warn(
                "Vakum onaylanamadı: OnRobotVGInput hiç gelmedi. Sim'de VGC10 "
                "sürücüsü çalışmıyorsa beklenen durum - require_confirmation'ı kapatın."
            )
        else:
            self._node.get_logger().warn(
                f"Vakum eşiğe ulaşmadı: {measured:.1f}% < {self._confirm_pct:.1f}%"
            )
        return False

    def release(self) -> bool:
        self._send(MODE_RELEASE, 0)
        self._node.get_logger().info("VAKUM BIRAK")
        self._sim_detach()
        time.sleep(0.5)
        return True

    def idle(self) -> bool:
        self._send(MODE_IDLE, 0)
        return True

    # --- Gazebo DetachableJoint -------------------------------------------

    def _sim_attach(self) -> None:
        if self._attach_pub is None:
            return
        self._attach_pub.publish(Empty())
        self._node.get_logger().info("SIM: parça kavrayıcıya bağlandı (attach)")

    def _sim_detach(self) -> None:
        if self._detach_pub is None:
            return
        self._detach_pub.publish(Empty())
        self._node.get_logger().info("SIM: parça bırakıldı (detach)")

    def sim_detach(self) -> None:
        """Dışarıdan çağrılabilir detach.

        Eklenti BAŞLANGIÇTA BAĞLI geliyor (child_model belirir belirmez eklemi
        kuruyor), o yüzden görev başlamadan önce bir kez ayırmak gerekiyor -
        yoksa kutu daha ilk taramada kolun peşinden sürüklenir. Launch da
        spawn'dan sonra bir detach yayınlıyor; bu ikinci kemer.
        """
        self._sim_detach()
