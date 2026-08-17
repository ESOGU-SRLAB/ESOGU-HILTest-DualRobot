#!/usr/bin/env python3
"""
Standalone linear-axis commissioning test. NO ROS, NO controller, NO MoveIt --
just this script and the Festo drive, so nothing else can explain the result.

It answers one question: does the drive honour the mdi_velocity word, and under
which telegram mode?

    Test A  plain position_task at three different velocities, no streaming.
            If the achieved speed scales with the commanded word, the drive
            honours mdi_velocity and the problem is in how we stream.
            If all three come out the same, mdi_velocity is being ignored and
            the cause is drive-side parameterisation, not our telegram.

    Test B  continuous-update streaming, the mode the adapter uses.
            Compares directly against Test A at the same velocity.

SAFETY
    * The full launch (and therefore linear_axis_adapter) must NOT be running:
      two masters on one Modbus link will fight.
    * Moves are bounded to +/- MOVE_M around the start position and refuse to
      run outside SAFE_MIN..SAFE_MAX.
    * Put the arm in a pose where +/- 10 cm of rail travel is harmless first.

    python3 linear_axis_diagnose.py [ip]
"""
import sys
import time

from edcon.edrive.com_modbus import ComModbus
from edcon.edrive.motion_handler import MotionHandler

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.3.1"
# "sweep" runs ONLY test E: the raw mdi_velocity word sweep.
ONLY_WORD_SWEEP = len(sys.argv) > 2 and sys.argv[2].lower() == "sweep"
WORD_SWEEP = [20, 50, 200, 1000, 5000, 50000]

MOVE_M = 0.08        # travel per test leg [m]
RAMP_M = 0.16        # travel for the ramped-setpoint tests [m]
RAMP_V = 0.08        # ramp rate of the streamed setpoint [m/s]
STREAM_HZ = 100.0    # setpoint stream rate, same as the adapter
BATCH_MM = 10.0      # Test D: only re-issue the target after this much change
SAFE_MIN, SAFE_MAX = 0.20, 1.80
SAMPLE_HZ = 50.0
SETTLE_S = 1.5
TEST_VELOCITIES = [0.05, 0.20, 0.50]   # m/s, as the adapter would command them


def banner(text):
    print(f"\n{'=' * 66}\n{text}\n{'=' * 66}")


class Axis:
    def __init__(self, ip):
        print(f"Connecting to {ip} ...")
        self.com = ComModbus(ip_address=ip, cycle_time=10, timeout_ms=1000)
        self.pos_scale = 10 ** (-self.com.read_pnu(11724, 0))
        self.vel_scale = 10 ** (-self.com.read_pnu(11725, 0))
        self.mh = MotionHandler(self.com, config_mode="write")
        self.mh.base_velocity = self.com.read_pnu(12345, 0)
        self.tg = self.mh.telegram

        print(f"  position_scaling = {self.pos_scale}")
        print(f"  velocity_scaling = {self.vel_scale}")
        print(f"  base_velocity    = {self.mh.base_velocity}")
        print(f"  over_v={self.mh.over_v:.0f}%  over_acc={self.mh.over_acc:.0f}%  "
              f"over_dec={self.mh.over_dec:.0f}%")

        self.mh.acknowledge_faults()
        self.mh.enable_powerstage()
        self.mh.update_inputs()
        print(f"  operation_enabled={self.tg.zsw1.operation_enabled}  "
              f"fault_present={self.tg.zsw1.fault_present}  "
              f"control_requested={self.tg.zsw1.control_requested}")
        if not self.tg.zsw1.operation_enabled:
            raise SystemExit("Drive is not operation_enabled -- cannot test.")

    def position(self):
        self.mh.update_inputs()
        return self.tg.xist_a.value / self.pos_scale

    def word_for(self, v_mps):
        return int(round(v_mps * 1000.0 * self.vel_scale))

    def measure(self, target_m, duration_s, stream=False, v_mps=0.0):
        """Sample position while a move runs; return (achieved_speed, travelled)."""
        start = self.position()
        t0 = time.time()
        samples = []
        while (time.time() - t0) < duration_s:
            if stream:
                self._stream_cycle(target_m, v_mps)
            samples.append((time.time() - t0, self.position()))
            time.sleep(1.0 / SAMPLE_HZ)

        # Peak speed over any 0.3 s window -- robust against accel/decel ramps.
        peak = 0.0
        for i in range(len(samples)):
            for j in range(i + 1, len(samples)):
                dt = samples[j][0] - samples[i][0]
                if dt < 0.3:
                    continue
                peak = max(peak, abs(samples[j][1] - samples[i][1]) / dt)
                break
        return peak, self.position() - start

    def _stream_cycle(self, target_m, v_mps):
        self.tg.mdi_tarpos.value = int(target_m * self.pos_scale)
        self.tg.mdi_velocity.value = self.word_for(v_mps)
        self.tg.pos_stw1.activate_setup = False
        self.tg.pos_stw1.activate_mdi = True
        self.tg.pos_stw1.absolute_position = True
        self.tg.pos_stw1.continuous_update = True
        self.tg.stw1.activate_traversing_task = True
        self.com.send_io(self.tg.output_bytes(), nonblocking=True)

    def ramp_stream(self, start_m, end_m, v_mps, batch_mm=0.0):
        """Stream a MOVING setpoint, exactly as the JTC feeds the adapter.

        This is the variable the earlier tests failed to control: they streamed a
        CONSTANT target. batch_mm > 0 re-issues the target only after it has moved
        that far, which is the candidate fix (let the drive's own profile generator
        cover each step instead of restarting it every cycle).

        Returns (max_following_error, achieved_mean_speed, final_error).
        """
        span = end_m - start_m
        duration = abs(span) / v_mps
        t0 = time.time()
        last_written = None
        max_err = 0.0
        first_pos = self.position()

        while True:
            t = time.time() - t0
            if t > duration + 1.5:
                break
            frac = min(1.0, t / duration)
            target = start_m + span * frac

            if batch_mm <= 0.0 or last_written is None or \
                    abs(target - last_written) * 1000.0 >= batch_mm or frac >= 1.0:
                self._stream_cycle(target, max(v_mps, 0.05))
                last_written = target
            else:
                self.com.send_io(self.tg.output_bytes(), nonblocking=True)

            actual = self.position()
            max_err = max(max_err, abs(target - actual))
            time.sleep(1.0 / STREAM_HZ)

        final = self.position()
        return max_err, abs(final - first_pos) / (duration + 1.5), abs(end_m - final)

    def simple_move(self, target_m, v_mps=None, word=None):
        """One plain traversing task -- no continuous update, no streaming.

        `word` bypasses the m/s conversion so the raw mdi_velocity value can be
        swept directly: that is the only way to learn the drive's actual velocity
        unit, rather than assuming ours matches it.
        """
        self.tg.pos_stw1.continuous_update = False
        if word is None:
            word = self.word_for(v_mps)
        ok = self.mh.position_task(int(target_m * self.pos_scale), word,
                                   absolute=True, nonblocking=True)
        if not ok:
            print("  !! position_task refused (drive not ready for motion)")
        return ok

    def stop(self):
        self.mh.stop_motion_task()
        time.sleep(0.5)

    def shutdown(self):
        try:
            self.mh.disable_powerstage()
        finally:
            self.com.shutdown()


def main():
    axis = Axis(IP)
    home = axis.position()
    print(f"\nStart position: {home:.4f} m")
    if not (SAFE_MIN < home < SAFE_MAX):
        raise SystemExit(f"Start position outside {SAFE_MIN}..{SAFE_MAX} m -- refusing.")
    reach = max(MOVE_M, RAMP_M / 2.0)
    if not (SAFE_MIN < home - reach and home + reach < SAFE_MAX):
        raise SystemExit("Not enough safe travel around the start position -- refusing.")

    print(f"\nWill move +/-{MOVE_M * 1000:.0f} mm around {home:.4f} m.")
    if input("Area clear and arm in a safe pose? [yes/N] ").strip().lower() != "yes":
        axis.shutdown()
        raise SystemExit("Aborted.")

    results = {}
    try:
        if ONLY_WORD_SWEEP:
            banner("TEST E -- raw mdi_velocity word sweep")
            print("  Maps the velocity word directly onto achieved speed, with no")
            print("  assumption about the drive's velocity unit. If the low words")
            print("  produce proportionally low speeds, our word is simply scaled")
            print("  wrong and the fix is the velocity_command_scale parameter.\n")
            direction = 1
            for word in WORD_SWEEP:
                target = home + direction * MOVE_M
                print(f"  word {word:>8}  ->  {target:.4f} m")
                axis.simple_move(target, word=word)
                peak, moved = axis.measure(target, 6.0)
                results[word] = (peak, moved)
                print(f"      peak {peak * 1000:7.1f} mm/s | travelled "
                      f"{moved * 1000:+7.1f} mm")
                axis.stop()
                time.sleep(SETTLE_S)
                direction *= -1

            banner("SUMMARY -- word -> speed")
            print(f"{'word':>10} {'peak speed':>13} {'mm/s per word':>16}")
            for word, (peak, _) in results.items():
                print(f"{word:>10} {peak * 1000:10.1f} mm/s "
                      f"{peak * 1000 / word:15.6f}")
            print("\n  A constant right-hand column = the drive honours the word and")
            print("  that number is the conversion. A column that collapses toward the")
            print("  top = everything above that word saturates the drive maximum.")
            return

        banner("TEST A -- plain position_task (no streaming, no continuous update)")
        direction = 1
        for v in TEST_VELOCITIES:
            target = home + direction * MOVE_M
            print(f"\n  commanding {v:.3f} m/s  (word {axis.word_for(v)})  "
                  f"to {target:.4f} m")
            axis.simple_move(target, v)
            peak, moved = axis.measure(target, 4.0)
            results[f"A/{v}"] = peak
            print(f"  -> peak speed {peak * 1000:7.1f} mm/s | travelled "
                  f"{moved * 1000:+7.1f} mm")
            axis.stop()
            time.sleep(SETTLE_S)
            direction *= -1

        banner("TEST B -- continuous-update streaming (what the adapter does)")
        for v in TEST_VELOCITIES:
            target = home + direction * MOVE_M
            print(f"\n  streaming target {target:.4f} m at {v:.3f} m/s "
                  f"(word {axis.word_for(v)})")
            peak, moved = axis.measure(target, 4.0, stream=True, v_mps=v)
            results[f"B/{v}"] = peak
            print(f"  -> peak speed {peak * 1000:7.1f} mm/s | travelled "
                  f"{moved * 1000:+7.1f} mm")
            axis.stop()
            time.sleep(SETTLE_S)
            direction *= -1

        banner("TEST C/D -- MOVING setpoint (this is what the real system does)")
        print(f"  ramping the target {RAMP_M * 1000:.0f} mm at {RAMP_V * 1000:.0f} mm/s\n")
        ramp_results = {}
        for label, batch in (("C cycle-by-cycle", 0.0), (f"D batched {BATCH_MM:.0f}mm", BATCH_MM)):
            start = axis.position()
            end = home + direction * (RAMP_M / 2.0)
            if abs(end - start) < 0.02:
                end = home - direction * (RAMP_M / 2.0)
            print(f"  [{label}] {start:.4f} -> {end:.4f} m")
            max_err, speed, final_err = axis.ramp_stream(start, end, RAMP_V, batch_mm=batch)
            ramp_results[label] = (max_err, speed, final_err)
            print(f"  -> max following error {max_err * 1000:6.1f} mm | "
                  f"mean speed {speed * 1000:6.1f} mm/s | "
                  f"final error {final_err * 1000:5.1f} mm")
            axis.stop()
            time.sleep(SETTLE_S)
            direction *= -1

        banner("SUMMARY")
        print(f"{'mode':>6} {'commanded':>12} {'achieved':>12}   ratio")
        for key, peak in results.items():
            mode, v = key.split("/")
            v = float(v)
            print(f"{mode:>6} {v * 1000:9.0f} mm/s {peak * 1000:9.1f} mm/s   "
                  f"{peak / v if v else 0:6.3f}")
        print(f"\n{'moving-setpoint mode':>20} {'max err':>10} {'mean speed':>12} {'final err':>10}")
        for label, (max_err, speed, final_err) in ramp_results.items():
            print(f"{label:>20} {max_err * 1000:7.1f} mm {speed * 1000:9.1f} mm/s "
                  f"{final_err * 1000:7.1f} mm")
        print(f"\n  (the setpoint ramped at {RAMP_V * 1000:.0f} mm/s -- a healthy mode "
              "tracks that\n   speed with a small, non-growing following error)")
        print("\nIf C creeps but D tracks, a cycle-by-cycle moving target is what stalls")
        print("the drive, and the adapter must batch its setpoints.")
        print("If BOTH creep, streaming a moving target is unusable on this drive and")
        print("the adapter must send segment endpoints instead.")

    finally:
        print("\nReturning to start position ...")
        try:
            axis.simple_move(home, 0.05)
            time.sleep(4.0)
            axis.stop()
        except Exception as exc:
            print(f"  return move failed: {exc}")
        axis.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
