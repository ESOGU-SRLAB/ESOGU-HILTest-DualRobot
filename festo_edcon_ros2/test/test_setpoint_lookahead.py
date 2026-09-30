#!/usr/bin/env python3
"""_write_setpoint'in ileri-tasima mantigini SAHTE telegram ile dogrular.

Kritik guvenlik ozelligi: yorunge bitince yazilan hedef GERCEK hedefe
sabitlenmeli. Aksi halde ray hedefi bir 'lead' boyu asar.
"""
import sys, types, time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from festo_edcon_ros2.linear_axis_adapter import LinearAxisControllerAdapter as LA


class Word:
    def __init__(self): self.value = 0


class Stw:
    activate_setup = False
    activate_mdi = False
    absolute_position = False
    continuous_update = False
    activate_traversing_task = False


class Telegram:
    def __init__(self):
        self.mdi_tarpos = Word()
        self.mdi_velocity = Word()
        self.pos_stw1 = Stw()
        self.stw1 = Stw()


def make():
    """Gercek __init__'i calistirmadan sadece gerekli alanlari kuran ornek."""
    a = LA.__new__(LA)
    a._lock = __import__("threading").Lock()
    a.telegram = Telegram()
    a.feedforward_gain = 1.0
    a.position_kp = 1.0
    a.min_velocity = 0.01
    a.max_velocity = 0.30
    a.velocity_scaling = 1000.0
    a.velocity_command_scale = 1.0
    a.position_scaling = 1000000.0
    a.setpoint_batch_m = 0.01
    a.settle_write_sec = 0.15
    a.lookahead_sec = 0.4
    a.max_lookahead = 0.06
    a.command_timeout_sec = 0.5
    a._last_written_target = None
    a._traversing_edge_done = True
    a._warned_zero_vel = False
    a._cmd_velocity = 0.0
    a._cmd_vel_word = 0
    a.creep_velocity = 0.002
    a.min_lookahead = 0.03
    a.final_tolerance = 0.0002
    return a


def run(a, setpoint, sp_vel, measured, age):
    a._setpoint_m = setpoint
    a._setpoint_stamp = time.monotonic() - age
    a._setpoint_vel = sp_vel
    a._measured_m = measured
    a._write_setpoint()
    return a.telegram.mdi_tarpos.value / a.position_scaling, a._cmd_velocity


fails = []

# --- 1) Seyir halinde: hedef ileri tasinmali ---
a = make()
aim, v = run(a, setpoint=0.500, sp_vel=0.053, measured=0.498, age=0.0)
lead = aim - 0.500
print(f"1) seyir      : aim={aim:.4f} lead={lead*1000:+.1f} mm v_cmd={v:.3f}")
if abs(lead - 0.030) > 1e-9:
    fails.append(f"0.053*0.4=21.2mm taban 30mm'e yukselmeliydi, {lead*1000:.1f} geldi")
if abs(v - (0.053 + 0.002)) > 1e-6:
    fails.append(f"geride: v_cmd = v_ff + kp*err olmali, {v} geldi")

# --- 2) Lead tavani ---
a = make()
aim, v = run(a, setpoint=0.500, sp_vel=0.300, measured=0.500, age=0.0)
print(f"2) hizli      : aim={aim:.4f} lead={(aim-0.5)*1000:+.1f} mm (tavan 60)")
if abs((aim - 0.500) - 0.060) > 1e-9:
    fails.append(f"lead 60mm'de kirpilmaliydi, {(aim-0.5)*1000:.1f} geldi")

# --- 3) GUVENLIK: akis durunca hedef GERCEK hedefe sabitlenmeli ---
a = make()
run(a, setpoint=0.500, sp_vel=0.053, measured=0.498, age=0.0)      # once ileri bak
aim, v = run(a, setpoint=0.500, sp_vel=0.053, measured=0.500, age=0.30)  # akis durdu
print(f"3) akis durdu : aim={aim:.4f} (gercek hedef 0.5000)")
if abs(aim - 0.500) > 1e-9:
    fails.append(f"akis durunca aim gercek hedefe sabitlenmeliydi, {aim} geldi")

# --- 4) Yavaslayan yorunge: lead sifira gitmeli VE son artik yazilmali
#        (kontrolcu akisi surdurse bile: crawling dali)
a = make()
print("4) yavaslama  :", end=" ")
for spv in (0.053, 0.030, 0.012, 0.003, 0.0):
    aim, _ = run(a, setpoint=0.500, sp_vel=spv, measured=0.500, age=0.0)
    print(f"v_ff={spv:.3f}->lead={(aim-0.5)*1000:+.0f}mm", end="  ")
print()
if abs(aim - 0.500) > 1e-9:
    fails.append("v_ff=0'da lead sifir olmaliydi")

# --- 5) lookahead kapaliyken eski davranis ---
a = make(); a.lookahead_sec = 0.0
aim, _ = run(a, setpoint=0.500, sp_vel=0.053, measured=0.498, age=0.0)
print(f"5) lookahead=0: aim={aim:.4f} (eski davranis = gercek hedef)")
if abs(aim - 0.500) > 1e-9:
    fails.append("lookahead=0 eski davranisi vermeliydi")

# --- 6) Geri yonde hareket: lead de geri gitmeli ---
a = make()
aim, _ = run(a, setpoint=0.500, sp_vel=-0.053, measured=0.502, age=0.0)
print(f"6) geri yon   : aim={aim:.4f} lead={(aim-0.5)*1000:+.1f} mm")
if (aim - 0.500) > 0:
    fails.append("geri giderken lead de negatif olmaliydi")

print()
if fails:
    print("BASARISIZ:")
    for f in fails:
        print("  -", f)
    sys.exit(1)

# --- 7) PARK HATASI: akis surerken son 2.6 mm yazilmali ---
a = make()
run(a, setpoint=0.4700, sp_vel=0.053, measured=0.4700, age=0.0)   # seyir
aim, _ = run(a, setpoint=0.4592, sp_vel=0.0, measured=0.4566, age=0.0)  # akis SURUYOR, hiz 0
print(f"7) park       : aim={aim:.4f} (gercek hedef 0.4592, eskiden bayat kalirdi)")
if abs(aim - 0.4592) > 1e-9:
    print("   -> BASARISIZ: son artik hala yazilmiyor")
    sys.exit(1)

# --- 8) ISARETLI HATA: ray ONDEYKEN hiz DUSMELI (titremenin sebebi) ---
a = make()
_, v_behind = run(a, setpoint=0.500, sp_vel=0.124, measured=0.4664, age=0.0)  # 33.6mm GERIDE
a = make()
_, v_ahead  = run(a, setpoint=0.500, sp_vel=0.124, measured=0.5336, age=0.0)  # 33.6mm ONDE
print(f"8) isaretli   : geride v_cmd={v_behind:.3f}  onde v_cmd={v_ahead:.3f}  (v_ff=0.124)")
if not (v_behind > 0.124 > v_ahead):
    print("   -> BASARISIZ: onde olmak hizi dusurmeli, artirmamali")
    sys.exit(1)

# --- 9) Geri yonde de isaret dogru olmali ---
a = make()
_, v_b = run(a, setpoint=0.500, sp_vel=-0.124, measured=0.5336, age=0.0)  # geride (- yonde)
a = make()
_, v_a = run(a, setpoint=0.500, sp_vel=-0.124, measured=0.4664, age=0.0)  # onde
print(f"9) geri isaret: geride v_cmd={v_b:.3f}  onde v_cmd={v_a:.3f}")
if not (v_b > 0.124 > v_a):
    print("   -> BASARISIZ: geri yonde isaret ters")
    sys.exit(1)

# --- 10) GUVENLIK: min lead, DURAN setpoint'te devreye GIRMEMELI ---
a = make()
run(a, setpoint=0.500, sp_vel=0.053, measured=0.500, age=0.0)      # once seyir (lead 30mm)
aim, _ = run(a, setpoint=0.500, sp_vel=0.0005, measured=0.500, age=0.0)  # hiz creep altinda
print(f"10) taban+dur : aim={aim:.4f} (gercek hedef 0.5000 olmali)")
if abs(aim - 0.500) > 1e-9:
    print("   -> BASARISIZ: min_lookahead rayi hedefin otesinde birakiyor")
    sys.exit(1)
print()
print("HEPSI GECTI - 3/4/7/10 guvenlik, 8/9 isaretli hata.")
