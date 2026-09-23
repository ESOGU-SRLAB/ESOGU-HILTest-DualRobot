"""
fmu_backend.py
==============
UR10e ters dinamik modeline erişim — İKİ yol, AYNI doğrulanmış C++ çekirdeği.

    backend="so"   → resources/ur10_solver_py...so doğrudan import edilir.
                     UniFMU/zmq katmanı atlanır. Aynı binary, aynı fizik, ~1000× hızlı.
    backend="fmu"  → fmpy ile FMU2Slave üzerinden, orijinal FMU yolu.

Fizik modeline HİÇBİR müdahale yok; her iki yol da senin doğrulanmış
`InverseDynamicsSolverUR10` sınıfını çağırır. `so` yolu sadece RPC sarmalayıcısını
atlar. İkisinin sayısal olarak aynı sonucu verdiği `check_fmu.py` ile doğrulanır.

ÖNEMLİ: `resources/model.py` solver yüklenemezse hatayı yutup sıfır tork döndürüyor.
Buradaki arka uçlar bunu YAPMAZ — yükleyemezlerse yüksek sesle hata verirler.
"""

from __future__ import annotations

import platform
import sys
from pathlib import Path

import numpy as np


class SolverUnavailable(RuntimeError):
    """Ters dinamik çekirdeği yüklenemedi — sessizce sıfır döndürmek yerine patla."""


# ─────────────────────── doğrudan .so arka ucu ───────────────────────

class SoBackend:
    """`ur10_solver_py` uzantı modülünü doğrudan çağırır."""

    name = "so"

    def __init__(self, resources_dir: str | Path):
        rd = str(Path(resources_dir).resolve())
        if rd not in sys.path:
            sys.path.insert(0, rd)
        try:
            import ur10_solver_py  # noqa: F401
        except ImportError as e:
            raise SolverUnavailable(
                f"`ur10_solver_py` import edilemedi: {e}\n"
                f"  arama yolu : {rd}\n"
                f"  platform   : {platform.system()} {platform.machine()}, "
                f"Python {sys.version_info.major}.{sys.version_info.minor}\n"
                f"  Mevcut derleme Linux x86-64 / CPython 3.10 içindir "
                f"(ur10_solver_py.cpython-310-x86_64-linux-gnu.so).\n"
                f"  → WSL2 Ubuntu 22.04 (python3.10) altında çalıştır."
            ) from e
        self._m = ur10_solver_py
        self._solver = ur10_solver_py.InverseDynamicsSolverUR10()

    def torques(self, q, qd, qdd, progress=None) -> np.ndarray:
        q = np.ascontiguousarray(q, dtype=np.float64)
        qd = np.ascontiguousarray(qd, dtype=np.float64)
        qdd = np.ascontiguousarray(qdd, dtype=np.float64)
        n = len(q)
        out = np.empty((n, 6), dtype=np.float64)
        g = self._solver.getTorques
        for k in range(n):
            out[k, :] = g(q[k], qd[k], qdd[k])
            if progress is not None and (k & 0xFFFF) == 0:
                progress(k, n)
        if progress is not None:
            progress(n, n)
        return out

    def close(self):
        pass


# ─────────────────────────── FMU arka ucu ───────────────────────────

class FmuBackend:
    """Orijinal FMU yolu: fmpy → UniFMU → backend_schemaless_rpc → model.py → C++."""

    name = "fmu"

    def __init__(self, fmu_path: str | Path):
        try:
            from fmpy import read_model_description, extract
            from fmpy.fmi2 import FMU2Slave
        except ImportError as e:
            raise SolverUnavailable(
                f"fmpy yüklü değil: {e}\n  → pip install fmpy pyzmq") from e

        fmu_path = str(Path(fmu_path).resolve())
        self._md = read_model_description(fmu_path)
        unzipdir = extract(fmu_path)
        self._fmu = FMU2Slave(
            guid=self._md.guid, unzipDirectory=unzipdir,
            modelIdentifier=self._md.coSimulation.modelIdentifier,
            instanceName="ur10e_id",
        )
        self._fmu.instantiate()
        self._fmu.setupExperiment(startTime=0.0)
        self._fmu.enterInitializationMode()
        self._fmu.exitInitializationMode()

        vr = {v.name: v.valueReference for v in self._md.modelVariables}
        self._vr_in = ([vr[f"q{j+1}"] for j in range(6)] +
                       [vr[f"qd{j+1}"] for j in range(6)] +
                       [vr[f"qdd{j+1}"] for j in range(6)])
        self._vr_tau = [vr[f"tau{j+1}"] for j in range(6)]

    def torques(self, q, qd, qdd, dt: float = 0.002, progress=None) -> np.ndarray:
        n = len(q)
        out = np.empty((n, 6), dtype=np.float64)
        f = self._fmu
        for k in range(n):
            f.setReal(self._vr_in,
                      q[k].tolist() + qd[k].tolist() + qdd[k].tolist())
            f.doStep(currentCommunicationPoint=float(k * dt),
                     communicationStepSize=float(dt))
            out[k, :] = f.getReal(self._vr_tau)
            if progress is not None and (k & 0x3FFF) == 0:
                progress(k, n)
        if progress is not None:
            progress(n, n)
        return out

    def close(self):
        try:
            self._fmu.terminate()
            self._fmu.freeInstance()
        except Exception:
            pass


# ──────────────────────────── fabrika ────────────────────────────

def make_backend(kind: str, resources_dir="resources",
                 fmu_path="UR10e_InverseDynamics.fmu"):
    if kind == "so":
        return SoBackend(resources_dir)
    if kind == "fmu":
        return FmuBackend(fmu_path)
    raise ValueError(f"bilinmeyen arka uç: {kind}")


def assert_nonzero(tau: np.ndarray, where: str = "") -> None:
    """Ters dinamik çıktısının gerçekten hesaplandığını doğrular."""
    if not np.isfinite(tau).all():
        raise SolverUnavailable(f"τ_model NaN/Inf içeriyor {where}")
    amax = float(np.abs(tau).max())
    if amax < 1e-9:
        raise SolverUnavailable(
            f"τ_model TAMAMEN SIFIR {where} — solver yüklenmemiş demektir. "
            f"Sessizce devam etmiyorum; `check_fmu.py` çalıştır."
        )
