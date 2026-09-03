#!/usr/bin/env python3
"""
UR IO pin bulucu.

/io_and_status_controller/io_states konusunu dinler ve DEĞİŞEN her pini
ekrana basar. Kullanımı:

  1. Robot sürücüsü ayaktayken bu scripti başlatın.
  2. Pendant'tan (veya fiziksel olarak) vidalama çıkışını 0 -> 1 yapın.
     Script hangi pinin değiştiğini yazacaktır.
  3. Aynı şeyi yeşil butona basıp bırakarak tekrarlayın.
  4. Ctrl+C ile çıkın; script değişen tüm pinlerin özetini ve
     kullanılacak launch argümanlarını yazdırır.

Pin numaralandırması (SetIO servisi ve IOStates mesajı aynı indeksi kullanır):
   0 -  7  -> standard digital output / input
   8 - 15  -> configurable output / input
  16 - 17  -> tool digital output / input
"""

import argparse
import sys
import time
from collections import OrderedDict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from ur_msgs.msg import IOStates


def digital_pin_label(pin: int, is_input: bool) -> str:
    """Ham pin indeksini pendant'ta göründüğü isme çevirir."""
    kind = "IN" if is_input else "OUT"
    if 0 <= pin <= 7:
        return f"standard_digital_{kind.lower()}[{pin}]  (pin={pin})"
    if 8 <= pin <= 15:
        return f"configurable_{kind.lower()}[{pin - 8}]  (pin={pin})"
    if 16 <= pin <= 17:
        return f"tool_digital_{kind.lower()}[{pin - 16}]  (pin={pin})"
    return f"bilinmeyen_{kind.lower()}[{pin}]  (pin={pin})"


class IOPinFinder(Node):
    def __init__(self, topic: str, analog_threshold: float, show_baseline: bool):
        super().__init__("ur_io_pin_finder")

        self.topic = topic
        self.analog_threshold = analog_threshold
        self.show_baseline = show_baseline

        self._baseline = None          # ilk mesaj: referans durum
        self._previous = None          # bir önceki mesaj
        self._start_time = time.time()
        self._msg_count = 0
        # {("digital_in", pin): degisim_sayisi} — çıkışta özet için
        self._change_counts = OrderedDict()

        # GPIOController yayını rclcpp::SystemDefaultsQoS ile yapıyor
        self.create_subscription(
            IOStates, topic, self._io_states_callback, qos_profile_system_default
        )

        self.get_logger().info(f"'{topic}' dinleniyor... (Ctrl+C ile çıkın)")
        self.get_logger().info(
            "Şimdi test etmek istediğiniz çıkışı/butonu tetikleyin."
        )

    # ------------------------------------------------------------------
    @staticmethod
    def _digital_map(states):
        return {d.pin: bool(d.state) for d in states}

    @staticmethod
    def _analog_map(states):
        return {a.pin: float(a.state) for a in states}

    def _snapshot(self, msg: IOStates):
        return {
            "digital_in": self._digital_map(msg.digital_in_states),
            "digital_out": self._digital_map(msg.digital_out_states),
            "flag": self._digital_map(msg.flag_states),
            "analog_in": self._analog_map(msg.analog_in_states),
            "analog_out": self._analog_map(msg.analog_out_states),
        }

    # ------------------------------------------------------------------
    def _io_states_callback(self, msg: IOStates):
        self._msg_count += 1
        current = self._snapshot(msg)

        if self._baseline is None:
            self._baseline = current
            self._previous = current
            self._print_baseline(current)
            return

        elapsed = time.time() - self._start_time

        for group in ("digital_in", "digital_out", "flag"):
            is_input = group == "digital_in"
            for pin, value in current[group].items():
                old = self._previous[group].get(pin)
                if old is None or old == value:
                    continue
                self._record(group, pin)
                if group == "flag":
                    label = f"flag[{pin}]  (pin={pin})"
                    tag = "FLAG "
                else:
                    label = digital_pin_label(pin, is_input)
                    tag = "GİRİŞ" if is_input else "ÇIKIŞ"
                print(
                    f"[{elapsed:8.2f}s] {tag}  {int(old)} -> {int(value)}   {label}",
                    flush=True,
                )

        for group in ("analog_in", "analog_out"):
            for pin, value in current[group].items():
                old = self._previous[group].get(pin)
                if old is None or abs(old - value) < self.analog_threshold:
                    continue
                self._record(group, pin)
                tag = "ANLG-G" if group == "analog_in" else "ANLG-Ç"
                print(
                    f"[{elapsed:8.2f}s] {tag} {old:.3f} -> {value:.3f}   {group}[{pin}]",
                    flush=True,
                )

        self._previous = current

    def _record(self, group, pin):
        key = (group, pin)
        self._change_counts[key] = self._change_counts.get(key, 0) + 1

    # ------------------------------------------------------------------
    def _print_baseline(self, snapshot):
        print("", flush=True)
        print("=" * 72, flush=True)
        print("BAŞLANGIÇ DURUMU (referans alındı)", flush=True)
        print("=" * 72, flush=True)
        if not self.show_baseline:
            active_in = [p for p, v in sorted(snapshot["digital_in"].items()) if v]
            active_out = [p for p, v in sorted(snapshot["digital_out"].items()) if v]
            print(f"  Şu an HIGH olan girişler : {active_in if active_in else 'yok'}", flush=True)
            print(f"  Şu an HIGH olan çıkışlar : {active_out if active_out else 'yok'}", flush=True)
        else:
            for group, is_input in (("digital_in", True), ("digital_out", False)):
                print(f"  --- {group} ---", flush=True)
                for pin, value in sorted(snapshot[group].items()):
                    print(
                        f"    {int(value)}  {digital_pin_label(pin, is_input)}",
                        flush=True,
                    )
            for group in ("analog_in", "analog_out"):
                for pin, value in sorted(snapshot[group].items()):
                    print(f"    {value:.3f}  {group}[{pin}]", flush=True)
        print("=" * 72, flush=True)
        print("Değişiklikler aşağıda listelenecek:", flush=True)
        print("", flush=True)

    # ------------------------------------------------------------------
    def print_summary(self):
        print("", flush=True)
        print("=" * 72, flush=True)
        print("ÖZET", flush=True)
        print("=" * 72, flush=True)

        if self._msg_count == 0:
            print(
                f"  '{self.topic}' konusundan HİÇ mesaj gelmedi.\n"
                "  * Sürücü ayakta mı?  ros2 node list\n"
                "  * Kontrolcü aktif mi? ros2 control list_controllers\n"
                "    (io_and_status_controller 'active' görünmeli)",
                flush=True,
            )
            return

        if not self._change_counts:
            print(
                f"  {self._msg_count} mesaj alındı ama hiçbir pin değişmedi.\n"
                "  Vidalama çıkışını / butonu gerçekten tetiklediğinizden emin olun.",
                flush=True,
            )
            return

        print(f"  {self._msg_count} mesaj alındı. Değişen pinler:", flush=True)
        print("", flush=True)
        for (group, pin), count in sorted(
            self._change_counts.items(), key=lambda kv: -kv[1]
        ):
            if group in ("digital_in", "digital_out"):
                label = digital_pin_label(pin, group == "digital_in")
            else:
                label = f"{group}[{pin}]"
            print(f"    {count:3d} değişim   {label}", flush=True)

        changed_in = sorted(p for (g, p) in self._change_counts if g == "digital_in")
        changed_out = sorted(p for (g, p) in self._change_counts if g == "digital_out")

        print("", flush=True)
        print("  Senaryoda kullanacağınız argümanlar:", flush=True)
        button = changed_in[0] if len(changed_in) == 1 else None
        screw = changed_out[0] if len(changed_out) == 1 else None
        print(
            "    ros2 launch pymoveit2_real human_robot_collaboration_scenario.launch.py \\",
            flush=True,
        )
        print(
            f"        screwdriver_pin:={screw if screw is not None else '<ÇIKIŞ_PINI>'} "
            f"green_button_pin:={button if button is not None else '<GİRİŞ_PINI>'}",
            flush=True,
        )
        if len(changed_in) > 1:
            print(
                f"    (birden fazla giriş değişti: {changed_in} — testleri tek tek tekrarlayın)",
                flush=True,
            )
        if len(changed_out) > 1:
            print(
                f"    (birden fazla çıkış değişti: {changed_out} — testleri tek tek tekrarlayın)",
                flush=True,
            )
        print("=" * 72, flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="UR dijital giriş/çıkış pinlerini tetikleyerek tespit eder."
    )
    parser.add_argument(
        "--topic",
        default="/io_and_status_controller/io_states",
        help="Dinlenecek IOStates konusu.",
    )
    parser.add_argument(
        "--analog-threshold",
        type=float,
        default=0.05,
        help="Analog girişlerde 'değişti' saymak için gereken minimum fark.",
    )
    parser.add_argument(
        "--baseline-all",
        action="store_true",
        help="Başlangıçta sadece HIGH olanları değil, tüm pinleri listele.",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = IOPinFinder(args.topic, args.analog_threshold, args.baseline_all)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C: rclpy'nin sinyal işleyicisi ikisinden birini fırlatabilir.
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
